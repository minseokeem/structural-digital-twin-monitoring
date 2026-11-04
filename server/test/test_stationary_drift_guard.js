'use strict';
const {updateRobustSlidingWindow}=require('../lib/robust-sliding-window');

function rng(seed=123456789){let x=seed>>>0;return()=>{x=(1664525*x+1013904223)>>>0;return x/4294967296;};}
function normalNoise(rand,s=1){const u=Math.max(1e-12,rand()),v=Math.max(1e-12,rand());return s*Math.sqrt(-2*Math.log(u))*Math.cos(2*Math.PI*v);}
function range(tag,a,d=0){const x=tag.x+tag.nx*d-a.x,y=tag.y+tag.ny*d-a.y,z=tag.z+tag.nz*d-a.z;return Math.hypot(x,y,z);}
const tag={x:0,y:0.72,z:0,nx:0,ny:-1,nz:0};
const anchors=[
  {node_id:'ANCHOR-01',x:-0.55,y:0,z:-0.35},
  {node_id:'ANCHOR-02',x:0.10,y:0,z:0.65},
  {node_id:'ANCHOR-03',x:0.62,y:0,z:-0.10},
  {node_id:'ANCHOR-04',x:-0.20,y:0,z:0.72}
];
let state={samples:[],d_m:0,v_mps:0,t:0};
const rand=rng(71);let now=0,maxAbs=0,maxV=0,zvCount=0;
for(let k=0;k<360;k++){
  now+=333;
  const drift=Math.min(0.020,k/359*0.020); // one bad link slowly drifts +20 mm over 2 min
  const measurements=anchors.slice(0,3).map((a,i)=>({anchorId:a.node_id,anchor:a,
    corrected_m:range(tag,a,0)+(i===1?drift:0)+normalNoise(rand,0.0022),sigma_m:i===1?0.010:0.006,nlos_score:i===1?0.25:0.05}));
  const out=updateRobustSlidingWindow({state,tag,measurements,now,windowMs:1200,imu:{vibration:false,dynamic_g:0.004,tilt_deg:0.02}});
  state=out.state;
  if(out.ready){maxAbs=Math.max(maxAbs,Math.abs(out.d_mm));maxV=Math.max(maxV,Math.abs(out.v_mm_s));if(out.zero_velocity_active)zvCount++;}
}
const finalMm=state.d_m*1000,finalV=state.v_mps*1000;
console.log(JSON.stringify({final_mm:finalMm,max_abs_mm:maxAbs,final_v_mm_s:finalV,max_v_mm_s:maxV,zero_velocity_updates:zvCount},null,2));
if(Math.abs(finalMm)>8)throw new Error(`stationary drift too large: ${finalMm.toFixed(2)} mm`);
if(Math.abs(finalV)>2.5)throw new Error(`stationary velocity not suppressed: ${finalV.toFixed(2)} mm/s`);
if(zvCount<100)throw new Error('zero-velocity constraint did not engage reliably');
console.log('PASS stationary drift guard');
