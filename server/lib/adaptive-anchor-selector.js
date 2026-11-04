'use strict';

function clamp(v,lo,hi){return Math.max(lo,Math.min(hi,v));}
function normal(tag){
  let x=Number(tag?.nx),y=Number(tag?.ny),z=Number(tag?.nz);let m=Math.hypot(x,y,z);
  if(!(Number.isFinite(m)&&m>1e-9)){x=0;y=1;z=0;m=1;}
  return {x:x/m,y:y/m,z:z/m};
}
function jacobian(tag,anchor){
  const n=normal(tag),dx=Number(tag.x)-Number(anchor.x),dy=Number(tag.y)-Number(anchor.y),dz=Number(tag.z)-Number(anchor.z);
  const r=Math.hypot(dx,dy,dz);return r>1e-9?(dx*n.x+dy*n.y+dz*n.z)/r:0;
}
function combinations(a,k){
  const out=[];const walk=(start,p)=>{if(p.length===k){out.push([...p]);return;}for(let i=start;i<a.length;i++){p.push(a[i]);walk(i+1,p);p.pop();}};walk(0,[]);return out;
}
function suffixIndex(id,fallback){const m=String(id||'').match(/(\d+)$/);const n=m?Number(m[1]):NaN;return Number.isInteger(n)&&n>=1&&n<=4?n-1:fallback;}

function scoreAnchors({tag,anchors,sensor,calibration}){
  const ranges=new Map((Array.isArray(sensor?.ranges)?sensor.ranges:[]).map(r=>[String(r?.anchor_id||''),r]));
  const details=new Map((Array.isArray(sensor?.uwb_link_details)?sensor.uwb_link_details:[]).map(r=>[String(r?.anchor_id||''),r]));
  return anchors.map((anchor,index)=>{
    const id=String(anchor.node_id||'');const r=ranges.get(id)||{};const d=details.get(id)||{};const l=calibration?.links?.[id]||{};
    const j=jacobian(tag,anchor),geom=clamp(Math.abs(j),0,1);
    const rate=Number.isFinite(Number(r.success_rate))?Number(r.success_rate):Number.isFinite(Number(r.attempt_success_rate))?Number(r.attempt_success_rate):85;
    const success=clamp(rate/100,0.05,1);
    const noise=Math.max(0.006,Number(l.runtime_noise_m||0),Number(l.noise_m||r.noise_m||0.025));
    const noiseScore=1/(1+(noise/0.025)**2);
    const nlos=clamp(Number(r.nlos_score||0),0,1);const diagScore=1-nlos;
    const residual=Math.abs(Number(d.residual_m||r.residual_m||0));
    const residualPenalty=clamp((residual-0.018)/0.055,0,1);
    const outlierPenalty=(d.outlier||r.outlier)?0.22:0;
    const stalePenalty=r.scheduled===false?0.02:0;
    const total=clamp(0.42*geom+0.23*success+0.20*noiseScore+0.15*diagScore-0.20*residualPenalty-outlierPenalty-stalePenalty,0,1);
    return {id,anchor,index,maskIndex:suffixIndex(id,index),scheduled:r.scheduled!==false,range_ok:r.ok===false?false:(r.m==null?null:Number.isFinite(Number(r.m))),j,geom,success,noise_m:noise,nlos_score:nlos,residual_m:residual,total};
  });
}
function comboScore(combo){
  const avg=combo.reduce((s,x)=>s+x.total,0)/combo.length;
  const js=combo.map(x=>x.j),spread=Math.max(...js)-Math.min(...js);
  const signs=new Set(js.filter(x=>Math.abs(x)>.02).map(Math.sign));
  const minGeom=Math.min(...combo.map(x=>x.geom));
  return avg+0.18*clamp(spread/0.7,0,1)+0.07*(signs.size>1?1:0)+0.05*minGeom;
}
function choose(scores,count){
  if(count>=scores.length)return [...scores].sort((a,b)=>b.total-a.total);
  let best=null,bestScore=-Infinity;
  for(const c of combinations(scores,count)){const s=comboScore(c);if(s>bestScore){bestScore=s;best=c;}}
  return (best||scores.slice(0,count)).sort((a,b)=>b.total-a.total);
}

function planAdaptiveRanging({tag,anchors,sensor,calibration,state={},now=Date.now(),calibrating=false,fullSweepMs=2600}){
  const ordered=[...anchors].filter(a=>a?.node_id).sort((a,b)=>String(a.node_id).localeCompare(String(b.node_id))).slice(0,4);
  const scores=scoreAnchors({tag,anchors:ordered,sensor,calibration});
  if(scores.length<2){
    return {profile:'verify4',reason:'layout_pending_full_sweep',targetLinks:4,anchorMask:0x0F,anchorIds:ordered.map(a=>String(a.node_id||'')),scores:[]};
  }
  if(!Number.isFinite(Number(state.lastFullSweepMs)))state.lastFullSweepMs=now;
  const recent=Number(sensor?.last_measurement_ms||0)>0 && now-Number(sensor.last_measurement_ms)<6000;
  const residual=Number(sensor?.uwb_residual_rms_m),uncertainty=Number(sensor?.uwb_uncertainty_mm),accel=Number(sensor?.accel_g||0);
  // Link-quality escalation must not trap the system in VERIFY4 just because a known
  // bad Anchor was observed during a deliberate 4-Anchor health sweep. Only quality
  // failures on the previous *requested* working set trigger immediate escalation; a
  // bad non-selected Anchor is simply scored down and reconsidered at the next sweep.
  const requestedMask=Number(sensor?.anchor_mask||0)&0x0F;
  const requestedLinks=Number(sensor?.requested_links||0);
  const requested=scores.filter(x=>requestedMask?(requestedMask&(1<<x.maskIndex)):x.scheduled);
  const working=requested.length?requested:scores.filter(x=>x.scheduled);
  const qualityEscalationAllowed=requestedLinks>0?requestedLinks<4:true;
  const linkNlos=qualityEscalationAllowed&&working.some(x=>x.nlos_score>=0.72);
  const linkSoft=qualityEscalationAllowed&&working.some(x=>x.nlos_score>=0.45);
  const weakLink=qualityEscalationAllowed&&working.some(x=>x.success<0.70||x.range_ok===false);
  const localReason=String(sensor?.adaptive_reason||'');
  const localHard=Boolean(sensor?.adaptive_event)&&['imu_event_verify','range_jump_verify','link_recovery'].includes(localReason);
  const localSoft=Boolean(sensor?.adaptive_event)&&localReason==='nlos_backup';
  const hard=Boolean(sensor?.vibration)||accel>=0.055||localHard||(recent&&sensor?.measurement_valid===false)||
    (Number.isFinite(residual)&&residual>0.040)||(Number.isFinite(uncertainty)&&uncertainty>55)||linkNlos;
  const soft=accel>=0.025||localSoft||(Number.isFinite(residual)&&residual>0.022)||(Number.isFinite(uncertainty)&&uncertainty>30)||linkSoft||weakLink;
  const periodic=!calibrating && now-state.lastFullSweepMs>=fullSweepMs;
  let count=2,reason='stable_best_two',profile='normal';
  if(calibrating){count=4;reason='calibration_full_sweep';profile='calibration';state.lastFullSweepMs=now;}
  else{
    if(hard){state.force4Until=Math.max(Number(state.force4Until||0),now+1100);state.force3Until=Math.max(Number(state.force3Until||0),now+2300);}
    if(periodic){state.lastFullSweepMs=now;state.force3Until=Math.max(Number(state.force3Until||0),now+450);}
    if(hard||now<Number(state.force4Until||0)||periodic){count=4;profile='verify4';reason=hard?'event_or_quality_verify':periodic?'periodic_health_sweep':'verify_hold';}
    else if(soft||now<Number(state.force3Until||0)){count=3;profile='verify3';reason=soft?'soft_quality_degrade':'recovery_hysteresis';}
  }
  const selected=choose(scores,Math.min(count,scores.length));
  let mask=0;for(const x of selected)if(x.maskIndex>=0&&x.maskIndex<4)mask|=1<<x.maskIndex;
  if(count===4)mask=0x0F;
  const plan={profile,reason,targetLinks:count,anchorMask:mask,anchorIds:selected.map(x=>x.id),scores:scores.map(x=>({anchor_id:x.id,score:Number(x.total.toFixed(3)),j:Number(x.j.toFixed(3)),success:Number(x.success.toFixed(3)),noise_m:Number(x.noise_m.toFixed(4)),nlos_score:Number(x.nlos_score.toFixed(3))}))};
  state.lastProfile=profile;state.lastReason=reason;state.lastPlan=plan;return plan;
}
module.exports={planAdaptiveRanging,scoreAnchors};
