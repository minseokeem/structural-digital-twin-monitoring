'use strict';
const assert=require('assert');
const {updateRobustSlidingWindow}=require('../lib/robust-sliding-window');
const tag={x:0,y:0.42,z:0,nx:0,ny:1,nz:0};
const anchors=[
  {node_id:'ANCHOR-01',x:-0.55,y:0,z:-0.45},
  {node_id:'ANCHOR-02',x:0.50,y:0,z:-0.30},
  {node_id:'ANCHOR-03',x:-0.30,y:0,z:0.55},
  {node_id:'ANCHOR-04',x:0.60,y:0,z:0.42},
];
function range(a,d=0){return Math.hypot(tag.x-a.x,tag.y+d-a.y,tag.z-a.z)}
function meas(d=0,noise=[0,0,0,0]){return anchors.map((a,i)=>({anchorId:a.node_id,anchor:a,corrected_m:range(a,d)+noise[i],sigma_m:.007,nlos_score:0.03}));}
let state={samples:[],d_m:0,v_mps:0,t:0},out,now=1000;
// Quiet warmup learns per-link residual baselines and zero velocity.
for(let k=0;k<30;k++){
  now+=100;out=updateRobustSlidingWindow({state,tag,measurements:meas(0,[.002,-.001,.0015,-.001]),now,windowMs:1200,imu:{vibration:false,dynamic_g:.004,tilt_deg:0}});state=out.state;
}
assert(Math.abs(out.d_mm)<8,`warmup drift ${out.d_mm}`);
// Strong vibration / footsteps: range perturbations are not coherent with one normal displacement.
let peak=0,verify=0,consReject=0;
for(let k=0;k<24;k++){
  now+=100;
  const n=[.010*Math.sin(k*.8),-.012*Math.cos(k*.55),.009*Math.sin(k*1.2+.7),-.011*Math.cos(k*.9)];
  out=updateRobustSlidingWindow({state,tag,measurements:meas(0,n),now,windowMs:1200,imu:{vibration:true,dynamic_g:.085,tilt_deg:0.05*Math.sin(k)}});state=out.state;
  if(out.ready){peak=Math.max(peak,Math.abs(out.d_mm));if(out.verification_only)verify++;if(out.consensus_rejected)consReject++;}
}
assert(peak<12,`vibration-only displacement escaped guard: ${peak.toFixed(2)} mm`);
assert(verify>5,'verification-only state did not engage');
// Now create a real +20 mm permanent normal displacement while vibration continues.
// All anchors support the same geometry; after repeated consensus it must be accepted.
let accepted=false,final=0;
for(let k=0;k<35;k++){
  now+=100;
  out=updateRobustSlidingWindow({state,tag,measurements:meas(.020,[.001,-.001,.001,-.001]),now,windowMs:1200,imu:{vibration:k<12,dynamic_g:k<12?.070:.006,tilt_deg:0}});state=out.state;
  if(out.ready){final=out.d_mm;if(out.displacement_consensus_confirmed)accepted=true;}
}
assert(accepted,'coherent real displacement never reached multi-anchor consensus');
assert(final>10 && final<30,`real displacement not recovered: ${final.toFixed(2)} mm`);
console.log('PASS vibration consensus guard',JSON.stringify({vibration_peak_mm:peak,verification_updates:verify,consensus_rejections:consReject,final_real_step_mm:final}));
