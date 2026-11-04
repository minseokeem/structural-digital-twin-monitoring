'use strict';

function clamp(v,lo,hi){return Math.max(lo,Math.min(hi,v));}
function unitNormal(tag){
  let x=Number(tag?.nx),y=Number(tag?.ny),z=Number(tag?.nz);let m=Math.hypot(x,y,z);
  if(!(Number.isFinite(m)&&m>1e-9)){x=0;y=1;z=0;m=1;}
  return {x:x/m,y:y/m,z:z/m};
}
function pointAt(tag,d=0){const n=unitNormal(tag);return {x:Number(tag?.x||0)+n.x*d,y:Number(tag?.y||0)+n.y*d,z:Number(tag?.z||0)+n.z*d};}
function jacobian(tag,anchor,d=0){
  const p=pointAt(tag,d),n=unitNormal(tag);
  const dx=p.x-Number(anchor?.x||0),dy=p.y-Number(anchor?.y||0),dz=p.z-Number(anchor?.z||0);
  const r=Math.hypot(dx,dy,dz);return r>1e-9?(dx*n.x+dy*n.y+dz*n.z)/r:0;
}
function suffixIndex(id,fallback){const m=String(id||'').match(/(\d+)$/);const n=m?Number(m[1]):NaN;return Number.isInteger(n)&&n>=1&&n<=4?n-1:fallback;}

// Continuous uncertainty model: CIR/NLOS never hard-deletes a link.  It inflates
// measurement sigma, so the estimator and scheduler naturally trust that link less.
function measurementSigmaM({range,link}={}){
  const noise=Math.max(0.006,Number(link?.runtime_noise_m||0),Number(link?.noise_m||range?.noise_m||0.018));
  const rateRaw=Number(range?.success_rate ?? range?.attempt_success_rate ?? 85);
  const success=clamp(Number.isFinite(rateRaw)?rateRaw/100:0.85,0.15,1);
  const nlos=clamp(Number(range?.nlos_score||0),0,1);
  let sigma=noise/Math.sqrt(success);
  sigma*=1+3.2*nlos*nlos;
  if(range?.nlos_suspect===true)sigma*=1.45;
  if(range?.ok===false)sigma=Math.max(sigma,0.12);
  if(range?.scheduled===false)sigma*=1.10;
  return clamp(sigma,0.006,0.25);
}

function expectedPosteriorSigmaM(priorSigmaM, selected){
  let precision=1/Math.max(priorSigmaM*priorSigmaM,1e-10);
  for(const x of selected)precision+=Math.max(0,x.information||0);
  return Math.sqrt(1/Math.max(precision,1e-12));
}

function planInformationRanging({tag,anchors,sensor,calibration,state={},now=Date.now(),calibrating=false,
  fullSweepMs=2800,targetSigmaMm=4.0,minLinks=2,maxLinks=4}={}){
  const ordered=[...(anchors||[])].filter(a=>a?.node_id).sort((a,b)=>String(a.node_id).localeCompare(String(b.node_id))).slice(0,4);
  if(ordered.length<minLinks){
    return {profile:'info4',reason:'layout_pending_full_sweep',targetLinks:Math.min(4,ordered.length||4),anchorMask:0x0F,
      anchorIds:ordered.map(a=>String(a.node_id||'')),projectedSigmaMm:null,targetSigmaMm,scores:[]};
  }
  const ranges=new Map((Array.isArray(sensor?.ranges)?sensor.ranges:[]).map(r=>[String(r?.anchor_id||''),r]));
  const previousD=Number.isFinite(Number(sensor?.dz_mm))?clamp(Number(sensor.dz_mm)/1000,-0.160,0.160):0;
  const currentSigmaMm=Number(sensor?.uwb_window_sigma_mm ?? sensor?.uwb_uncertainty_mm);
  let priorSigmaMm=Number.isFinite(currentSigmaMm)?clamp(currentSigmaMm,1.5,60):14;
  const motionScore=clamp(Number(sensor?.uwb_motion_score||0),0,1);
  priorSigmaMm=Math.max(priorSigmaMm,3.5+14*motionScore);
  const dynamicG=Math.abs(Number(sensor?.accel_g||0));
  const vibrationEvent=Boolean(sensor?.vibration)||dynamicG>=0.055;
  // vibration does not mean the displacement state itself is uncertain.
  // It asks for more independent UWB evidence instead of inflating the process prior.
  if(!vibrationEvent && dynamicG>=0.025)priorSigmaMm=Math.max(priorSigmaMm,9);
  const residualMm=Math.abs(Number(sensor?.uwb_residual_rms_m||0))*1000;
  if(Number.isFinite(residualMm)&&residualMm>28)priorSigmaMm=Math.max(priorSigmaMm,18);
  else if(Number.isFinite(residualMm)&&residualMm>16)priorSigmaMm=Math.max(priorSigmaMm,10);

  const quarantinedIds=new Set(Array.isArray(sensor?.uwb_quarantined_anchors)?sensor.uwb_quarantined_anchors.map(String):[]);
  const scored=ordered.map((anchor,index)=>{
    const id=String(anchor.node_id||'');
    const r=ranges.get(id)||{};
    const link=calibration?.links?.[id]||{};
    const h=jacobian(tag,anchor,previousD);
    let sigma=measurementSigmaM({range:r,link});
    if(link?.calibrated!==true)sigma=Math.max(sigma,0.16); // health sweeps can still discover/recover it.
    const quarantined=quarantinedIds.has(id);
    if(quarantined)sigma=Math.max(sigma,0.20);
    const information=(h*h)/(sigma*sigma);
    return {id,anchor,index,maskIndex:suffixIndex(id,index),h,sigma_m:sigma,information,nlos_score:clamp(Number(r?.nlos_score||0),0,1),
      success_rate:Number.isFinite(Number(r?.success_rate))?Number(r.success_rate):null,calibrated:link?.calibrated===true,quarantined};
  }).sort((a,b)=>(Number(a.quarantined)-Number(b.quarantined))||(b.information-a.information));

  if(!Number.isFinite(Number(state.lastFullSweepMs)))state.lastFullSweepMs=0;
  const periodic=!calibrating && now-Number(state.lastFullSweepMs||0)>=fullSweepMs;
  if(calibrating||periodic){
    state.lastFullSweepMs=now;
    const selected=[...scored];
    return {profile:calibrating?'calibration':'info4',reason:calibrating?'calibration_full_sweep':'periodic_health_sweep',
      targetLinks:Math.min(maxLinks,ordered.length),anchorMask:0x0F,anchorIds:selected.map(x=>x.id),
      priorSigmaMm,projectedSigmaMm:expectedPosteriorSigmaM(priorSigmaMm/1000,selected)*1000,targetSigmaMm,
      scores:selected.map(x=>({anchor_id:x.id,h:Number(x.h.toFixed(4)),sigma_mm:Number((x.sigma_m*1000).toFixed(2)),information:Number(x.information.toFixed(1)),nlos_score:Number(x.nlos_score.toFixed(3)),calibrated:x.calibrated,quarantined:Boolean(x.quarantined)}))};
  }

  const residualHard=Number.isFinite(residualMm)&&residualMm>40;
  const uncertaintyHard=Number.isFinite(currentSigmaMm)&&currentSigmaMm>35;
  const verificationMinLinks=vibrationEvent?3:minLinks;
  const forceFullVerification=vibrationEvent&&(residualHard||uncertaintyHard||scored.some(x=>x.nlos_score>=0.72));
  if(forceFullVerification){
    const selected=[...scored].slice(0,Math.min(maxLinks,scored.length));
    state.lastPlanMs=now;state.lastPlan={profile:'info4',reason:'vibration_quality_full_verify',targetLinks:selected.length,anchorIds:selected.map(x=>x.id)};
    return {profile:'info4',reason:'vibration_quality_full_verify',targetLinks:selected.length,anchorMask:0x0F,anchorIds:selected.map(x=>x.id),priorSigmaMm,
      projectedSigmaMm:expectedPosteriorSigmaM(priorSigmaMm/1000,selected)*1000,targetSigmaMm:Math.min(targetSigmaMm,3.5),
      scores:scored.map(x=>({anchor_id:x.id,h:Number(x.h.toFixed(4)),sigma_mm:Number((x.sigma_m*1000).toFixed(2)),information:Number(x.information.toFixed(1)),nlos_score:Number(x.nlos_score.toFixed(3)),calibrated:x.calibrated,quarantined:Boolean(x.quarantined)}))};
  }

  const healthy=scored.filter(x=>!x.quarantined);
  const planningPool=healthy.length>=minLinks?healthy:scored;
  const selected=[];
  const limit=Math.min(maxLinks,planningPool.length);
  for(const x of planningPool){
    selected.push(x);
    if(selected.length<Math.min(verificationMinLinks,limit))continue;
    const projected=expectedPosteriorSigmaM(priorSigmaMm/1000,selected)*1000;
    if(projected<=targetSigmaMm || selected.length>=limit)break;
  }
  while(selected.length<Math.min(verificationMinLinks,limit))selected.push(planningPool[selected.length]);
  const projectedSigmaMm=expectedPosteriorSigmaM(priorSigmaMm/1000,selected)*1000;
  let mask=0;for(const x of selected)if(x.maskIndex>=0&&x.maskIndex<4)mask|=1<<x.maskIndex;
  const targetLinks=selected.length;
  const reason=vibrationEvent?'vibration_independent_verify':(projectedSigmaMm<=targetSigmaMm?'target_uncertainty_met':'uncertainty_requires_more');
  const profile=`info${targetLinks}`;
  state.lastPlanMs=now;state.lastPlan={profile,reason,targetLinks,projectedSigmaMm,anchorIds:selected.map(x=>x.id)};
  return {profile,reason,targetLinks,anchorMask:mask,anchorIds:selected.map(x=>x.id),priorSigmaMm,projectedSigmaMm,targetSigmaMm,
    scores:scored.map(x=>({anchor_id:x.id,h:Number(x.h.toFixed(4)),sigma_mm:Number((x.sigma_m*1000).toFixed(2)),information:Number(x.information.toFixed(1)),nlos_score:Number(x.nlos_score.toFixed(3)),calibrated:x.calibrated,quarantined:Boolean(x.quarantined)}))};
}

module.exports={planInformationRanging,measurementSigmaM,jacobian,expectedPosteriorSigmaM};
