'use strict';
const assert=require('assert');
const {updateRobustSlidingWindow}=require('../lib/robust-sliding-window');

const tag={x:0,y:0,z:0,nx:0,ny:1,nz:0};
const anchors=[
  {node_id:'ANCHOR-01',x:0,y:1,z:0},
  {node_id:'ANCHOR-02',x:.6,y:.8,z:0},
  {node_id:'ANCHOR-03',x:-.7,y:.7,z:.1},
  {node_id:'ANCHOR-04',x:.2,y:1.2,z:-.2}
];
function range(a,d){return Math.hypot(tag.x-(a.x),tag.y+d-a.y,tag.z-a.z);}
let state={samples:[],d_m:0,v_mps:0,t:0};
const noises=[.002,-.003,.001,.004,-.002,.003,-.001,.002];
const trueD=.020;
let out;
const t0=100000;
for(let k=0;k<8;k++){
  const now=t0+k*150;
  const measurements=anchors.slice(0,k<2?2:4).map((a,i)=>({anchorId:a.node_id,anchor:a,corrected_m:range(a,trueD)+noises[(k+i)%noises.length],sigma_m:.008,nlos_score:0}));
  if(k===5)measurements[0].corrected_m+=.09; // robust outlier: one bad UWB measurement
  out=updateRobustSlidingWindow({state,tag,measurements,now,windowMs:1200,imu:{dynamic_g:0.01,vibration:false}});
  state=out.state;
}
assert(out.ready,'window should become ready');
assert(Math.abs(out.d_mm-20)<8,`expected around 20mm, got ${out.d_mm}`);
assert(out.anchorCount>=2);
assert(Number.isFinite(out.posterior_sigma_mm));
assert(out.motion_score>=0&&out.motion_score<=1);
const quietWindow=out.effective_window_ms;
let moving=updateRobustSlidingWindow({state,tag,measurements:anchors.slice(0,2).map(a=>({anchorId:a.node_id,anchor:a,corrected_m:range(a,trueD+.01),sigma_m:.008,nlos_score:0})),now:t0+1350,windowMs:1200,imu:{dynamic_g:.10,vibration:true,tilt_deg:2}});
assert(moving.motion_score>out.motion_score,'motion event should raise continuous motion score');
assert(moving.effective_window_ms<quietWindow,'motion event should shorten effective temporal memory');
console.log('PASS robust sliding window',JSON.stringify({d_mm:out.d_mm,sigma_mm:out.posterior_sigma_mm,residual_mm:out.residual_rms_m*1000,samples:out.sampleCount}));
