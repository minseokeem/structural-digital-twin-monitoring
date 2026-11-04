'use strict';
const assert=require('assert');
const {updateRobustSlidingWindow}=require('../lib/robust-sliding-window');
const tag={x:0,y:0.30,z:0,nx:0,ny:1,nz:0};
const anchors=[
 {node_id:'ANCHOR-01',x:-0.55,y:0,z:-0.45},
 {node_id:'ANCHOR-02',x:0.55,y:0,z:-0.45},
 {node_id:'ANCHOR-03',x:-0.55,y:0,z:0.45},
 {node_id:'ANCHOR-04',x:0.55,y:0,z:0.45},
];
function range(a,d=0){return Math.hypot(tag.x-a.x,tag.y+d-a.y,tag.z-a.z)}
let state={samples:[],d_m:0,v_mps:0,t:0}, now=1000, out;
// warm up at zero
for(let k=0;k<15;k++){
 now+=100;
 const ms=anchors.slice(0,2).map((a,i)=>({anchorId:a.node_id,anchor:a,corrected_m:range(a,0)+(i?-.001:.001),sigma_m:.008,nlos_score:0}));
 out=updateRobustSlidingWindow({state,tag,measurements:ms,now,windowMs:1200,imu:{dynamic_g:.005,vibration:false,tilt_deg:0}}); state=out.state;
}
assert(Math.abs(out.d_mm)<15,'warmup drift too large');
let peak=Math.abs(out.d_mm||0), rejected=0;
// brutal asymmetric two-anchor corruption; old R13.1 could feed residual -> motion -> huge GN steps.
for(let k=0;k<8;k++){
 now+=100;
 const ms=[
  {anchorId:anchors[0].node_id,anchor:anchors[0],corrected_m:range(anchors[0],0)+.22,sigma_m:.008,nlos_score:0},
  {anchorId:anchors[1].node_id,anchor:anchors[1],corrected_m:range(anchors[1],0)-.18,sigma_m:.008,nlos_score:0},
 ];
 out=updateRobustSlidingWindow({state,tag,measurements:ms,now,windowMs:1200,imu:{dynamic_g:.005,vibration:false,tilt_deg:0}}); state=out.state;
 peak=Math.max(peak,Math.abs(out.d_mm||0));if(out.state_guard_rejected)rejected++;
}
assert(peak<160,`runaway guard failed: ${peak} mm`);
// recover with good data
for(let k=0;k<25;k++){
 now+=100;
 const ms=anchors.slice(0,3).map(a=>({anchorId:a.node_id,anchor:a,corrected_m:range(a,0),sigma_m:.008,nlos_score:0}));
 out=updateRobustSlidingWindow({state,tag,measurements:ms,now,windowMs:1200,imu:{dynamic_g:.005,vibration:false,tilt_deg:0}}); state=out.state;
}
assert(Math.abs(out.d_mm)<25,`did not recover: ${out.d_mm} mm`);
console.log('PASS runaway guard',JSON.stringify({peak_mm:peak,rejected,final_mm:out.d_mm}));
