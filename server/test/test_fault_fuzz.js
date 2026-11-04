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
function meas(d=0,noise=[0,0,0,0],drop=new Set(),sig=.008){return anchors.flatMap((a,i)=>drop.has(i)?[]:[{anchorId:a.node_id,anchor:a,corrected_m:range(a,d)+(noise[i]||0),sigma_m:sig,nlos_score:Math.abs(noise[i]||0)>.08?.85:.03}]);}
function warm(state,now,d=0,n=35){let out;for(let k=0;k<n;k++){now+=100;out=updateRobustSlidingWindow({state,tag,measurements:meas(d,[.001,-.001,.0015,-.001]),now,windowMs:1200,imu:{vibration:false,dynamic_g:.004,tilt_deg:0}});state=out.state;}return {state,now,out};}
function runStep(state,now,d,noiseFn,imuFn,n=30){let out,peak=0,reacq=0;for(let k=0;k<n;k++){now+=100;out=updateRobustSlidingWindow({state,tag,measurements:meas(d,noiseFn(k)),now,windowMs:1200,imu:imuFn(k)});state=out.state;if(out.ready){peak=Math.max(peak,Math.abs(out.d_mm));if(out.reacquired)reacq++;}}return {state,now,out,peak,reacq};}

// A: real +60 mm step under vibration, while one Anchor lies by +0.60 m.
{
 let state={samples:[],d_m:0,v_mps:0,t:0},now=1000;({state,now}=warm(state,now));
 let r=runStep(state,now,.060,k=>[.002,.600,-.002,.003],k=>({vibration:k<15,dynamic_g:k<15?.09:.005,tilt_deg:0}),35);
 assert(r.out.d_mm>45&&r.out.d_mm<75,`60 mm reacquisition failed: ${r.out.d_mm}`);
 assert(r.reacq>=1,`large-step reacquisition never fired`);
 const q=Object.keys(r.out.quarantine_until||{});assert(q.includes('ANCHOR-02'),`bad A2 not quarantined: ${q}`);
 console.log('PASS real 60mm + huge bad anchor',{final:r.out.d_mm,reacq:r.reacq,q});
}

// B: structure stays still while A1 slowly drifts by -140 mm. Good links must win.
{
 let state={samples:[],d_m:0,v_mps:0,t:0},now=1000;({state,now}=warm(state,now));let out,max=0;
 for(let k=0;k<180;k++){now+=100;const drift=-.14*Math.min(1,k/80);out=updateRobustSlidingWindow({state,tag,measurements:meas(0,[drift,.001,-.001,.0015]),now,windowMs:1200,imu:{vibration:false,dynamic_g:.004,tilt_deg:0}});state=out.state;if(out.ready)max=Math.max(max,Math.abs(out.d_mm));}
 assert(max<12,`stationary state dragged by one bad anchor: ${max}`);
 assert((out.quarantined_anchors||[]).includes('ANCHOR-01'),`drifting A1 not quarantined`);
 console.log('PASS stationary + one drifting anchor',{max,final:out.d_mm,q:out.quarantined_anchors});
}

// C: 2-vs-2 contradictory large steps during vibration must NOT reacquire.
{
 let state={samples:[],d_m:0,v_mps:0,t:0},now=1000;({state,now}=warm(state,now));let out,reacq=0,max=0;
 for(let k=0;k<18;k++){now+=100; // manipulate ranges so two Anchors imply +60 mm and two imply -60 mm
   const n=anchors.map((a,i)=>{const h=(range(a,.001)-range(a,0))/.001;return h*(i<2?.060:-.060);});
   out=updateRobustSlidingWindow({state,tag,measurements:meas(0,n),now,windowMs:1200,imu:{vibration:true,dynamic_g:.09,tilt_deg:.1}});state=out.state;if(out.ready){if(out.reacquired)reacq++;max=Math.max(max,Math.abs(out.d_mm));}
 }
 assert(reacq===0,`ambiguous 2v2 caused reacquisition`);assert(max<15,`ambiguous 2v2 moved state ${max}`);
 console.log('PASS 2v2 ambiguity hold',{max,reacq});
}

// D: random one-packet spikes, dropouts and vibration bursts must never run away.
{
 let seed=0x13579bdf;const rnd=()=>{seed=(1664525*seed+1013904223)>>>0;return seed/4294967296};
 let state={samples:[],d_m:0,v_mps:0,t:0},now=1000;({state,now}=warm(state,now));let out,maxErr=0,maxAbs=0,truth=0;
 for(let k=0;k<1200;k++){
   now+=100;
   if(k===250)truth=.030;if(k===520)truth=.010;if(k===800)truth=.060;if(k===980)truth=.020;
   const noise=[0,0,0,0].map(()=>.003*(rnd()-.5));
   if(rnd()<.055){const i=Math.floor(rnd()*4);noise[i]+=(rnd()<.5?-1:1)*(.12+.35*rnd());}
   if(rnd()<.018){for(let i=0;i<4;i++)noise[i]+=(rnd()-.5)*.12;}
   const drop=new Set();if(rnd()<.04)drop.add(Math.floor(rnd()*4));if(rnd()<.012){drop.add(Math.floor(rnd()*4));drop.add(Math.floor(rnd()*4));}
   const vib=rnd()<.10;
   out=updateRobustSlidingWindow({state,tag,measurements:meas(truth,noise,drop),now,windowMs:1200,imu:{vibration:vib,dynamic_g:vib?.07+.06*rnd():.004+.004*rnd(),tilt_deg:vib?.3*(rnd()-.5):.01*(rnd()-.5)}});state=out.state;
   if(out.ready){maxAbs=Math.max(maxAbs,Math.abs(out.d_mm));maxErr=Math.max(maxErr,Math.abs(out.d_mm-truth*1000));assert(Math.abs(out.d_mm)<155,'runaway state escaped physical cap');}
 }
 assert(maxErr<55,`fuzz error too large: ${maxErr}`);
 console.log('PASS irregular fuzz',{maxErr,maxAbs,final:out.d_mm,truth:truth*1000});
}

// E: one Anchor disappears permanently; remaining 3 should still track +25 mm.
{
 let state={samples:[],d_m:0,v_mps:0,t:0},now=1000;({state,now}=warm(state,now));let out;
 for(let k=0;k<60;k++){now+=100;out=updateRobustSlidingWindow({state,tag,measurements:meas(.025,[.001,0,-.001,.001],new Set([0])),now,windowMs:1200,imu:{vibration:k<5,dynamic_g:k<5?.06:.004,tilt_deg:0}});state=out.state;}
 assert(out.d_mm>15&&out.d_mm<35,`3-anchor tracking failed: ${out.d_mm}`);
 console.log('PASS permanent anchor dropout',{final:out.d_mm});
}

console.log('ALL R13.5 FAULT FUZZ TESTS PASS');
