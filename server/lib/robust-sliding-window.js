'use strict';

function clamp(v,lo,hi){return Math.max(lo,Math.min(hi,v));}
function normal(tag){let x=Number(tag?.nx),y=Number(tag?.ny),z=Number(tag?.nz),m=Math.hypot(x,y,z);if(!(m>1e-9)){x=0;y=1;z=0;m=1;}return{x:x/m,y:y/m,z:z/m};}
function predictedRange(tag,anchor,d){const n=normal(tag);const x=Number(tag.x)+n.x*d-Number(anchor.x),y=Number(tag.y)+n.y*d-Number(anchor.y),z=Number(tag.z)+n.z*d-Number(anchor.z);const r=Math.hypot(x,y,z);const h=r>1e-9?(x*n.x+y*n.y+z*n.z)/r:0;return{r,h};}
function solve2(a00,a01,a11,b0,b1){const det=a00*a11-a01*a01;if(!(det>1e-12))return null;return{x0:(b0*a11-b1*a01)/det,x1:(b1*a00-b0*a01)/det,inv00:a11/det,inv11:a00/det,inv01:-a01/det};}

function median(values){const a=[...values].filter(Number.isFinite).sort((x,y)=>x-y);if(!a.length)return NaN;const m=Math.floor(a.length/2);return a.length%2?a[m]:(a[m-1]+a[m])/2;}
function mad(values,center){const c=Number.isFinite(center)?center:median(values);return median(values.map(v=>Math.abs(v-c)));}

// robust inlier-subset consensus.
function assessMeasurementConsensus(tag,measurements,dPred,state,{vibrationEvent=false}={}){
  if(!state.link_residual_baseline_m||typeof state.link_residual_baseline_m!=='object')state.link_residual_baseline_m={};
  const rows=[];
  for(const m of Array.isArray(measurements)?measurements:[]){
    if(!m?.anchor||!Number.isFinite(Number(m.corrected_m)))continue;
    const id=String(m.anchorId||m.anchor?.node_id||''),pr=predictedRange(tag,m.anchor,dPred);
    if(!Number.isFinite(pr.h)||Math.abs(pr.h)<0.08)continue;
    const residual=Number(m.corrected_m)-pr.r,base=Number(state.link_residual_baseline_m[id]||0),innovation=residual-base;
    const step=innovation/pr.h,sigma=clamp(Number(m.sigma_m||0.02),0.006,0.25),quality=1/(sigma*sigma);
    if(Number.isFinite(step))rows.push({id,step,residual,innovation,h:pr.h,sigma,quality});
  }
  if(rows.length<2)return {confirmed:false,step_m:0,consensus:0,spread_m:Infinity,count:rows.length,rows,inliers:[],outliers:rows,inlier_ids:[],outlier_ids:rows.map(r=>r.id),ambiguous:true};
  let best=null;
  for(const c of rows){
    const tol=clamp(0.010+0.18*Math.min(Math.abs(c.step),0.080),0.010,0.024),subset=rows.filter(r=>Math.abs(r.step-c.step)<=tol);
    const med=median(subset.map(r=>r.step)),spread=1.4826*(mad(subset.map(r=>r.step),med)||0);
    const score=subset.length*1e6+subset.reduce((a,r)=>a+Math.min(r.quality,10000),0)-spread*1e5;
    if(!best||score>best.score)best={score,med,spread};
  }
  const tol=clamp(Math.max(0.009,2.8*best.spread,0.006+0.16*Math.min(Math.abs(best.med),0.10)),0.009,0.026);
  const inliers=rows.filter(r=>Math.abs(r.step-best.med)<=tol);
  if(inliers.length){best.med=median(inliers.map(r=>r.step));best.spread=1.4826*(mad(inliers.map(r=>r.step),best.med)||0);}
  const ids=new Set(inliers.map(r=>r.id)),outliers=rows.filter(r=>!ids.has(r.id)),large=Math.abs(best.med)>=0.025;
  const required=(vibrationEvent||large)&&rows.length>=3?3:2,ambiguous=rows.length>=4&&inliers.length===2&&Math.abs(best.med)>=0.012;
  return {confirmed:inliers.length>=required&&best.spread<=(large?0.018:0.010)&&!ambiguous,step_m:best.med,consensus:inliers.length/rows.length,
    spread_m:best.spread,count:rows.length,inlier_count:inliers.length,outlier_count:outliers.length,required_count:required,rows,inliers,outliers,
    inlier_ids:[...ids],outlier_ids:outliers.map(r=>r.id),ambiguous,large_step:large};
}
function updateLinkQuarantine(state,consensus,now){
  if(!state.link_health||typeof state.link_health!=='object')state.link_health={};
  // If the whole RF environment is extremely noisy, do not blame individual
  // Anchors. Quarantine is for isolated link failures, not global poor SNR.
  const batchSigma=median((consensus?.rows||[]).map(r=>Number(r.sigma)).filter(Number.isFinite));
  if(Number.isFinite(batchSigma)&&batchSigma>0.025){
    for(const h of Object.values(state.link_health))h.bad_count=Math.max(0,Number(h.bad_count||0)-1);
    return Object.fromEntries(Object.entries(state.link_health).filter(([,h])=>now<Number(h.quarantine_until_ms||0)).map(([id,h])=>[id,Number(h.quarantine_until_ms)]));
  }
  const good=new Set(consensus?.inlier_ids||[]),bad=new Set(consensus?.outlier_ids||[]);
  for(const r of consensus?.rows||[]){
    const h=state.link_health[r.id]||(state.link_health[r.id]={bad_count:0,good_count:0,quarantine_until_ms:0});
    if(bad.has(r.id)){
      const severe=Math.abs(Number(r.step||0)-Number(consensus?.step_m||0))>0.025 || Math.abs(Number(r.innovation||0))>Math.max(0.050,6*Number(r.sigma||0.02));
      if(severe){
        h.bad_count=Math.min(20,(h.bad_count||0)+1);h.good_count=0;
        const otherActive=Object.entries(state.link_health).some(([oid,oh])=>oid!==r.id&&now<Number(oh?.quarantine_until_ms||0));
        if(h.bad_count>=3&&!otherActive)h.quarantine_until_ms=Math.max(h.quarantine_until_ms||0,now+2200);
      } else h.bad_count=Math.max(0,(h.bad_count||0)-1);
    }
    else if(good.has(r.id)){h.good_count=Math.min(20,(h.good_count||0)+1);h.bad_count=Math.max(0,(h.bad_count||0)-1);if(h.good_count>=4&&now>=(h.quarantine_until_ms||0)-800)h.quarantine_until_ms=0;}
    h.last_update_ms=now;
  }
  return Object.fromEntries(Object.entries(state.link_health).filter(([,h])=>now<Number(h.quarantine_until_ms||0)).map(([id,h])=>[id,Number(h.quarantine_until_ms)]));
}
function isQuarantined(state,id,now){return now<Number(state?.link_health?.[id]?.quarantine_until_ms||0);}
function isSevereConsensusOutlier(consensus,id){
  if(!(consensus?.outlier_ids||[]).includes(id))return false;
  const r=(consensus?.rows||[]).find(x=>x.id===id);if(!r)return false;
  return Math.abs(Number(r.step||0)-Number(consensus?.step_m||0))>0.025 ||
    Math.abs(Number(r.innovation||0))>Math.max(0.050,6*Number(r.sigma||0.02));
}

// Distinguish a real normal displacement from a packet-wide UWB range shift.
// Around the predicted state, a real displacement follows residual_i ~= H_i*delta,
// while a radio/common-clock burst looks like residual_i ~= constant.  With 3+
// Anchors we fit both delta and a nuisance common offset and softly reject batches
// dominated by the latter.  This preserves real structural motion but prevents a
// short +range burst on every Anchor from masquerading as a 10+ cm bridge jump.
function assessPacketCommonMode(tag,measurements,dPred){
  if(!Array.isArray(measurements)||measurements.length<3||!Number.isFinite(dPred))return {suspect:false,common_m:0,delta_m:0};
  let a00=0,a01=0,a11=0,b0=0,b1=0,meanAbsH=0,n=0;
  const rows=[];
  for(const m of measurements){
    if(!m?.anchor||!Number.isFinite(Number(m.corrected_m))||!Number.isFinite(Number(m.sigma_m)))continue;
    const pr=predictedRange(tag,m.anchor,dPred),res=Number(m.corrected_m)-pr.r,sigma=clamp(Number(m.sigma_m),0.006,0.25),w=1/(sigma*sigma),h=pr.h;
    a00+=w*h*h;a01+=w*h;a11+=w;b0+=w*h*res;b1+=w*res;meanAbsH+=Math.abs(h);n++;rows.push({h,res,w});
  }
  if(n<3)return {suspect:false,common_m:0,delta_m:0};
  const sol=solve2(a00,a01,a11,b0,b1);if(!sol)return {suspect:false,common_m:0,delta_m:0};
  const delta=sol.x0,common=sol.x1;meanAbsH/=n;
  let sq=0,ws=0;for(const r of rows){const e=r.res-r.h*delta-common;sq+=r.w*e*e;ws+=r.w;}
  const fitRms=ws>0?Math.sqrt(sq/ws):Infinity;
  const displacementEffect=Math.abs(delta)*Math.max(meanAbsH,0.08);
  const suspect=Math.abs(common)>=0.035 && Math.abs(common)>=Math.max(0.025,1.6*displacementEffect) && fitRms<=0.025;
  return {suspect,common_m:common,delta_m:delta,fit_rms_m:fitRms};
}

function assessAbruptStep(tag,measurements,dPred){
  const steps=[];
  for(const m of Array.isArray(measurements)?measurements:[]){
    if(!m?.anchor||!Number.isFinite(Number(m.corrected_m)))continue;
    const pr=predictedRange(tag,m.anchor,dPred);
    if(!Number.isFinite(pr.h)||Math.abs(pr.h)<0.08)continue;
    const step=(Number(m.corrected_m)-pr.r)/pr.h;
    if(Number.isFinite(step))steps.push(step);
  }
  if(steps.length<2)return {large:false,step_m:0,consensus:0};
  const sorted=[...steps].sort((a,b)=>a-b),mid=Math.floor(sorted.length/2),med=sorted.length%2?sorted[mid]:(sorted[mid-1]+sorted[mid])/2;
  const sign=Math.sign(med),same=steps.filter(x=>Math.sign(x)===sign).length,consensus=same/steps.length;
  return {large:Math.abs(med)>=0.060&&consensus>=0.75,step_m:med,consensus};
}

function updateRobustSlidingWindow({state={},tag,measurements=[],now=Date.now(),windowMs=1200,imu={}}={}){
  if(!Array.isArray(state.samples))state.samples=[];
  if(!Number.isFinite(Number(state.started_ms)))state.started_ms=now;
  const prevT=Number(state.t||now),dtPred=Math.max(0,Math.min(2,(now-prevT)/1000));
  const prevD=Number.isFinite(Number(state.d_m))?Number(state.d_m):0;
  const prevV=Number.isFinite(Number(state.v_mps))?Number(state.v_mps):0;

  // One continuous motion score, not discrete modes. It decays smoothly after an
  // event so the same [d, v, covariance] state is preserved at all times.
  const dg=Math.abs(Number(imu.dynamic_g||0));
  const vibrationEvent=Boolean(imu.vibration)||dg>=0.055;
  const dgScore=clamp((dg-0.020)/0.14,0,1);
  const tilt=Number(imu.tilt_deg||0);
  const lastTilt=Number.isFinite(Number(state.last_tilt_deg))?Number(state.last_tilt_deg):tilt;
  const tiltDt=Math.max(.05,Math.min(2,(now-Number(state.last_tilt_ms||now))/1000));
  const tiltRate=Math.abs(tilt-lastTilt)/tiltDt;
  const tiltScore=clamp((tiltRate-.15)/2.4,0,1);
  const prevResidualMm=Math.abs(Number(state.last_residual_rms_m||0))*1000;
  if(!Number.isFinite(Number(state.residual_baseline_m))&&Number.isFinite(Number(state.last_residual_rms_m)))state.residual_baseline_m=Number(state.last_residual_rms_m);
  const baselineResidualMm=Math.max(12,Number(state.residual_baseline_m||0.024)*1000);
  const residualExcessMm=Math.max(0,prevResidualMm-baselineResidualMm);
  const residualScore=clamp(residualExcessMm/35,0,1);
  const velocityScore=clamp((Math.abs(prevV)*1000-2.0)/50,0,1);

  // vibration is a verification trigger, NOT proof of permanent displacement.
  // Multi-anchor UWB must agree on a normal-direction change before we loosen the
  // motion model. This prevents footsteps/impacts from unlocking a runaway state.
  const consensus=assessMeasurementConsensus(tag,measurements,prevD,state,{vibrationEvent});
  const consensusMedianSigma=median((consensus?.rows||[]).map(r=>Number(r.sigma)).filter(Number.isFinite));
  const globalNoiseBatch=Number.isFinite(consensusMedianSigma)&&consensusMedianSigma>0.025;
  // A packet-wide common-mode radio/timestamp burst is not evidence that individual
  // Anchors became bad. Detect it before updating quarantine or motion consensus.
  const preliminaryCommon=assessPacketCommonMode(tag,measurements,prevD);
  const quarantined=preliminaryCommon.suspect
    ?Object.fromEntries(Object.entries(state.link_health||{}).filter(([,h])=>now<Number(h.quarantine_until_ms||0)).map(([id,h])=>[id,Number(h.quarantine_until_ms)]))
    :updateLinkQuarantine(state,consensus,now);
  const usableMeasurements=(Array.isArray(measurements)?measurements:[]).filter(m=>{
    const id=String(m?.anchorId||m?.anchor?.node_id||'');
    if(isQuarantined(state,id,now))return false;
    if(!globalNoiseBatch && !preliminaryCommon.suspect && consensus.inlier_count>=2 && (consensus.outlier_ids||[]).includes(id))return false;
    return true;
  });
  if(!preliminaryCommon.suspect && consensus.confirmed){
    const dir=Math.sign(consensus.step_m);
    if(state.motion_consensus_dir===dir && now-Number(state.motion_consensus_last_ms||0)<=850)state.motion_consensus_count=Number(state.motion_consensus_count||0)+1;
    else {state.motion_consensus_dir=dir;state.motion_consensus_count=1;}
    state.motion_consensus_last_ms=now;
  }else if(now-Number(state.motion_consensus_last_ms||0)>850){state.motion_consensus_count=0;state.motion_consensus_dir=0;}
  const consensusConfirmed=consensus.confirmed && Number(state.motion_consensus_count||0)>=2;
  const consensusScore=consensusConfirmed?clamp(Math.abs(consensus.step_m)/0.030,0.15,1):0;
  const kinematicScore=clamp(Math.max(consensusScore,.45*tiltScore,.12*residualScore,.12*velocityScore),0,1);
  const verificationOnly=vibrationEvent && !consensusConfirmed;
  const instantMotion=verificationOnly?Math.max(0.30,Math.min(0.35,kinematicScore)):Math.max(kinematicScore,.25*dgScore);
  const decay=Math.exp(-dtPred/(verificationOnly?0.30:0.85));
  const motionScore=clamp(Math.max(instantMotion,Number(state.motion_score||0)*decay),0,1);
  state.motion_score=motionScore;state.last_tilt_deg=tilt;state.last_tilt_ms=now;

  // Quiet structure: long memory and tight process prior. Motion: shorter temporal
  // memory and larger process uncertainty. Both change continuously.
  const effectiveWindowMs=Math.max(650,windowMs*(1-.82*motionScore));

  // stationary-drift control.
  // Velocity is allowed to persist while genuine motion is present, but decays
  // naturally while the structure is quiet.  This prevents a tiny persistent UWB
  // bias from being integrated forever as bridge velocity.
  const quietResidualLimitMm=Math.max(30,baselineResidualMm*1.55+4);
  const quietInstant=!vibrationEvent && dg<0.025 && tiltRate<0.22 && prevResidualMm<=quietResidualLimitMm;
  if(quietInstant){
    if(!Number.isFinite(Number(state.quiet_since_ms)))state.quiet_since_ms=now;
  }else state.quiet_since_ms=null;
  const quietDurationMs=Number.isFinite(Number(state.quiet_since_ms))?Math.max(0,now-Number(state.quiet_since_ms)):0;
  const zeroVelocityActive=quietInstant && quietDurationMs>=900;

  const velocityTauSec=verificationOnly?0.22:(motionScore>0.35?2.8:0.65);
  const velocityLeak=Math.exp(-dtPred/velocityTauSec);
  const leakedV=prevV*velocityLeak;
  const velocityUse=verificationOnly?0.05:(.10+.75*motionScore);
  let vPrior=(zeroVelocityActive||verificationOnly?0:leakedV*velocityUse);
  let dPrior=prevD; // R14.2: do not propagate absolute position from stale velocity

  // automatic large-step reacquisition. A genuine step can be much larger
  // than the ordinary per-cycle tracking gate. If an inlier subset of independent
  // Anchors repeatedly agrees on the same new displacement, reset the old temporal
  // window and acquire the new state instead of freezing forever.
  let reacquired=false,reacquireTargetM=null;
  const reacquireMedianSigma=median((consensus.inliers||[]).map(r=>Number(r.sigma)).filter(Number.isFinite));
  const reacquireQualityOk=Number.isFinite(reacquireMedianSigma)&&reacquireMedianSigma<=0.030;
  if(consensus.confirmed && consensus.large_step && consensus.inlier_count>=3 && reacquireQualityOk && Math.abs(Number(consensus.step_m||0))<=0.090){
    const target=clamp(prevD+Number(consensus.step_m||0),-0.160,0.160);
    const same=Number.isFinite(Number(state.reacquire_target_m)) && Math.abs(target-Number(state.reacquire_target_m))<=0.015 &&
      now-Number(state.reacquire_last_ms||0)<=950;
    state.reacquire_count=same?Number(state.reacquire_count||0)+1:1;
    state.reacquire_target_m=target;state.reacquire_last_ms=now;
    if(state.reacquire_count>=2){
      dPrior=target;vPrior=0;state.samples=[];reacquired=true;reacquireTargetM=target;
      state.reacquire_count=0;state.reacquire_accept_until_ms=now+1200;
    }
  }else if(now-Number(state.reacquire_last_ms||0)>950){
    state.reacquire_count=0;state.reacquire_target_m=null;
  }

  const geometryHs=usableMeasurements.map(m=>m?.anchor?Math.abs(predictedRange(tag,m.anchor,dPrior).h):NaN).filter(Number.isFinite);
  const meanAbsH=geometryHs.length?geometryHs.reduce((a,b)=>a+b,0)/geometryHs.length:1;
  const weakGeometryProcess=meanAbsH<0.14?clamp((0.14-meanAbsH)/0.10,0,1):0;
  const priorSigmaD=reacquired?0.0045:(verificationOnly?0.0032:(.0030+(.009-.0030)*motionScore+.0008*dtPred+.0018*weakGeometryProcess));
  const priorSigmaV=(zeroVelocityActive||verificationOnly||reacquired)?0.0028:(.007+(.045-.007)*motionScore);

  const commonAssessment=preliminaryCommon.suspect?preliminaryCommon:assessPacketCommonMode(tag,measurements,dPrior);
  const abrupt=assessAbruptStep(tag,usableMeasurements,dPrior);
  if(abrupt.large){
    const dir=Math.sign(abrupt.step_m);
    if(state.large_step_dir===dir && now-Number(state.large_step_last_ms||0)<=700)state.large_step_count=Number(state.large_step_count||0)+1;
    else {state.large_step_dir=dir;state.large_step_count=1;}
    state.large_step_last_ms=now;
    if(state.large_step_count>=4)state.large_step_accept_until_ms=now+1800;
  }else if(now-Number(state.large_step_last_ms||0)>700){state.large_step_count=0;state.large_step_dir=0;}
  const transientLarge=!reacquired && vibrationEvent && abrupt.large && now>Number(state.large_step_accept_until_ms||0);
  const rejectPacket=!reacquired && (commonAssessment.suspect || transientLarge);
  if(!rejectPacket){
    for(const m of usableMeasurements){
      if(!m?.anchor||!Number.isFinite(Number(m.corrected_m))||!Number.isFinite(Number(m.sigma_m)))continue;
      const id=String(m.anchorId||m.anchor?.node_id||'');
      state.samples.push({t:now,anchorId:id,anchor:m.anchor,corrected_m:Number(m.corrected_m),sigma_m:clamp(Number(m.sigma_m),0.006,0.25),nlos_score:Number(m.nlos_score||0),common_mode_suspect:false});
    }
  }
  const cutoff=now-windowMs;
  state.samples=state.samples.filter(s=>s.t>=cutoff&&!isQuarantined(state,s.anchorId,now)&&(globalNoiseBatch||!(consensus.outlier_ids||[]).includes(s.anchorId))).slice(-96);
  const unique=new Set(state.samples.map(s=>s.anchorId));
  if(unique.size<2||state.samples.length<2)return {ready:false,state,reason:'window_warmup',sampleCount:state.samples.length,anchorCount:unique.size,motion_score:motionScore,effective_window_ms:effectiveWindowMs,process_sigma_d_mm:priorSigmaD*1000,process_sigma_v_mm_s:priorSigmaV*1000,common_mode_rejected:commonAssessment.suspect,packet_rejected:rejectPacket,abrupt_step_rejected:transientLarge};

  let d=dPrior,v=vPrior;
  const huberK=2.5;

  // Samples from the same Anchor are correlated.  Without this cap, five recent
  // packets from A2 could be treated as five independent votes and slowly drag the
  // state in one direction.  Limit the TOTAL information contributed by repeated
  // observations from one Anchor inside the window.
  const anchorSampleCounts=new Map();
  for(const s of state.samples)anchorSampleCounts.set(s.anchorId,(anchorSampleCounts.get(s.anchorId)||0)+1);
  const repeatScale=id=>1/Math.sqrt(Math.max(1,anchorSampleCounts.get(id)||1));

  for(let iter=0;iter<8;iter++){
    let a00=1/(priorSigmaD*priorSigmaD),a01=0,a11=1/(priorSigmaV*priorSigmaV);
    let b0=(dPrior-d)/(priorSigmaD*priorSigmaD),b1=(vPrior-v)/(priorSigmaV*priorSigmaV);
    if(zeroVelocityActive||verificationOnly){
      const zvSigma=verificationOnly?0.0018:0.0025;
      a11+=1/(zvSigma*zvSigma);
      b1+=(0-v)/(zvSigma*zvSigma);
    }
    for(const s of state.samples){
      const dt=(s.t-now)/1000,ds=d+v*dt,pr=predictedRange(tag,s.anchor,ds),res=s.corrected_m-pr.r,z=res/s.sigma_m;
      const robust=Math.abs(z)<=huberK?1:huberK/Math.max(Math.abs(z),1e-9),ageMs=Math.max(0,now-s.t),ageWeight=Math.exp(-.82*ageMs/Math.max(effectiveWindowMs,1));
      const geometryRelief=clamp(0.16/Math.max(Math.abs(pr.h),0.02),1,5),repetitionWeight=Math.min(1,repeatScale(s.anchorId)*geometryRelief),w=robust*ageWeight*repetitionWeight/(s.sigma_m*s.sigma_m),j0=pr.h,j1=pr.h*dt;
      a00+=w*j0*j0;a01+=w*j0*j1;a11+=w*j1*j1;b0+=w*j0*res;b1+=w*j1*res;
    }
    const sol=solve2(a00,a01,a11,b0,b1);if(!sol)return {ready:false,state,reason:'singular_window',sampleCount:state.samples.length,anchorCount:unique.size,motion_score:motionScore,effective_window_ms:effectiveWindowMs,process_sigma_d_mm:priorSigmaD*1000,process_sigma_v_mm_s:priorSigmaV*1000};
    // limit the TOTAL state movement relative to the prediction, not every
    // Gauss-Newton iteration independently. The old 63 mm/iteration * 8 iterations
    // allowed a single bad batch to generate >0.5 m jumps.
    const totalStepLimit=verificationOnly?0.0025:(.004+.012*motionScore);
    const targetD=clamp(d+sol.x0,dPrior-totalStepLimit,dPrior+totalStepLimit);
    d=targetD;
    const velAbsLimit=verificationOnly?0.006:(.018+.040*motionScore);
    v=clamp(v+sol.x1,-velAbsLimit,velAbsLimit);
    if(Math.abs(sol.x0)<1e-6&&Math.abs(sol.x1)<1e-5)break;
  }

  let a00=1/(priorSigmaD*priorSigmaD),a01=0,a11=1/(priorSigmaV*priorSigmaV),sq=0,ws=0;
  if(zeroVelocityActive||verificationOnly){const zvSigma=verificationOnly?0.0018:0.0025;a11+=1/(zvSigma*zvSigma);}
  const details=[];
  for(const s of state.samples){
    const dt=(s.t-now)/1000,ds=d+v*dt,pr=predictedRange(tag,s.anchor,ds),res=s.corrected_m-pr.r,z=res/s.sigma_m;
    const robust=Math.abs(z)<=huberK?1:huberK/Math.max(Math.abs(z),1e-9),ageMs=Math.max(0,now-s.t),ageWeight=Math.exp(-.82*ageMs/Math.max(effectiveWindowMs,1));
    const geometryRelief=clamp(0.16/Math.max(Math.abs(pr.h),0.02),1,5),repetitionWeight=Math.min(1,repeatScale(s.anchorId)*geometryRelief),w=robust*ageWeight*repetitionWeight/(s.sigma_m*s.sigma_m),j0=pr.h,j1=pr.h*dt;
    a00+=w*j0*j0;a01+=w*j0*j1;a11+=w*j1*j1;sq+=w*res*res;ws+=w;
    details.push({anchor_id:s.anchorId,residual_m:res,sigma_m:s.sigma_m,robust_weight:robust,age_weight:ageWeight,repetition_weight:repetitionWeight,total_weight:robust*ageWeight*repetitionWeight,h:pr.h,nlos_score:s.nlos_score,common_mode_suspect:s.common_mode_suspect});
  }
  const cov=solve2(a00,a01,a11,0,0);if(!cov)return {ready:false,state,reason:'singular_covariance',sampleCount:state.samples.length,anchorCount:unique.size,motion_score:motionScore,effective_window_ms:effectiveWindowMs};
  const residualRms=ws>0?Math.sqrt(sq/ws):Infinity;
  const normalizedScale=Number.isFinite(residualRms)?Math.max(1,residualRms/.012):1;
  const sigmaD=Math.sqrt(Math.max(cov.inv00,1e-12))*Math.sqrt(normalizedScale);
  const latestByAnchor=new Map();for(const x of details)latestByAnchor.set(x.anchor_id,x);
  // Surface hard-rejected/quarantined current links in diagnostics with zero weight.
  for(const r of consensus.rows||[]){
    const q=isQuarantined(state,r.id,now),hard=isSevereConsensusOutlier(consensus,r.id);
    if(hard||q){
      latestByAnchor.set(r.id,{anchor_id:r.id,residual_m:r.residual,sigma_m:r.sigma,robust_weight:0,age_weight:1,repetition_weight:0,total_weight:0,
        h:r.h,nlos_score:null,hard_rejected:hard,quarantined:q,quarantine_until_ms:Number(state.link_health?.[r.id]?.quarantine_until_ms||0),
        candidate_step_m:r.step,innovation_m:r.innovation});
    }
  }

  // state-commit guard. A result can be mathematically finite yet physically
  // implausible. Previously we marked such a result invalid later in server.js but
  // had already written it into state.d_m/state.v_mps, poisoning all following
  // predictions. Keep the last trusted state whenever the window is grossly bad.
  const candidateDelta=Math.abs(d-dPrior);
  const absDisplacement=Math.abs(d);
  const grossResidual=!Number.isFinite(residualRms)||residualRms>0.075;
  const grossSigma=!Number.isFinite(sigmaD)||sigmaD>0.060;
  const impossibleState=absDisplacement>0.160 || candidateDelta>(verificationOnly?0.0035:(.006+.018*motionScore));
  const consensusRequired=verificationOnly && candidateDelta>0.0025;
  const consensusRejected=consensusRequired&&!consensusConfirmed;
  const stateGuardRejected=grossResidual||grossSigma||impossibleState||consensusRejected;
  let committedD=d,committedV=v;
  if(stateGuardRejected){
    committedD=prevD;
    committedV=clamp(prevV*(verificationOnly?.12:.45),-.018,.018);
  }
  // residual reference is learned only during startup and then frozen.
  // It is a geometry/model-mismatch reference, not an adaptive zero; otherwise
  // a real permanent structural displacement can be slowly learned away.
  if(!state.link_residual_baseline_m||typeof state.link_residual_baseline_m!=='object')state.link_residual_baseline_m={};
  if(!state.link_residual_baseline_count||typeof state.link_residual_baseline_count!=='object')state.link_residual_baseline_count={};
  if(!state.link_residual_baseline_frozen && !vibrationEvent&&!stateGuardRejected&&motionScore<0.20){
    for(const m of usableMeasurements){
      if(!m?.anchor||!Number.isFinite(Number(m.corrected_m)))continue;
      const id=String(m.anchorId||m.anchor?.node_id||'');
      const pr=predictedRange(tag,m.anchor,committedD);
      const r=Number(m.corrected_m)-pr.r;
      const old=Number(state.link_residual_baseline_m[id]);
      const cnt=Number(state.link_residual_baseline_count[id]||0);
      const alpha=cnt<4?1/(cnt+1):0.10;
      state.link_residual_baseline_m[id]=Number.isFinite(old)?((1-alpha)*old+alpha*r):r;
      state.link_residual_baseline_count[id]=cnt+1;
    }
    if(Number.isFinite(residualRms))state.residual_baseline_m=Number.isFinite(Number(state.residual_baseline_m))?(.95*Number(state.residual_baseline_m)+.05*residualRms):residualRms;
    const counts=Object.values(state.link_residual_baseline_count).map(Number).filter(Number.isFinite);
    if(now-Number(state.started_ms||now)>=3000 && counts.length>=2 && counts.filter(x=>x>=15).length>=2)state.link_residual_baseline_frozen=true;
  }
  state.d_m=committedD;state.v_mps=committedV;state.t=now;state.posterior_sigma_m=sigmaD;state.last_residual_rms_m=residualRms;
  state.last_common_mode_m=commonAssessment.common_m||0;state.last_common_mode_rejected=commonAssessment.suspect;state.last_packet_rejected=rejectPacket;state.last_abrupt_step_rejected=transientLarge;
  state.last_state_guard_rejected=stateGuardRejected;
  return {ready:true,state,d_m:committedD,d_mm:committedD*1000,v_mps:committedV,v_mm_s:committedV*1000,posterior_sigma_m:sigmaD,posterior_sigma_mm:sigmaD*1000,
    residual_rms_m:residualRms,sampleCount:state.samples.length,anchorCount:unique.size,windowSpanMs:state.samples.length?now-state.samples[0].t:0,windowAgeMs:Math.max(0,now-Number(state.started_ms||now)),
    motion_score:motionScore,effective_window_ms:effectiveWindowMs,process_sigma_d_mm:priorSigmaD*1000,process_sigma_v_mm_s:priorSigmaV*1000,
    tilt_rate_deg_s:tiltRate,zero_velocity_active:zeroVelocityActive,quiet_duration_ms:quietDurationMs,velocity_leak:velocityLeak,verification_only:vibrationEvent&&!consensusConfirmed,
    displacement_consensus_confirmed:consensusConfirmed,displacement_consensus_count:Number(state.motion_consensus_count||0),displacement_consensus_mm:Number(consensus.step_m||0)*1000,displacement_consensus_spread_mm:Number.isFinite(consensus.spread_m)?consensus.spread_m*1000:null,
    residual_baseline_mm:baselineResidualMm,quiet_residual_limit_mm:quietResidualLimitMm,common_mode_rejected:commonAssessment.suspect,packet_rejected:rejectPacket,abrupt_step_rejected:transientLarge,state_guard_rejected:stateGuardRejected,consensus_rejected:consensusRejected,abrupt_step_m:abrupt.step_m||0,common_mode_m:commonAssessment.common_m||0,common_mode_fit_rms_m:commonAssessment.fit_rms_m??null,
    inlier_count:Number(consensus.inlier_count||0),outlier_count:Number(consensus.outlier_count||0),inlier_anchors:consensus.inlier_ids||[],outlier_anchors:consensus.outlier_ids||[],
    quarantined_anchors:Object.keys(quarantined||{}),quarantine_until:quarantined,reacquired,reacquire_target_mm:reacquireTargetM!=null?reacquireTargetM*1000:null,
    residual_baseline_frozen:Boolean(state.link_residual_baseline_frozen),
    link_details:[...latestByAnchor.values()]};
}

module.exports={updateRobustSlidingWindow,assessPacketCommonMode,assessAbruptStep};
