'use strict';
const assert=require('assert');
const {updateRobustSlidingWindow}=require('../lib/robust-sliding-window');
const tag={x:0,y:.42,z:0,nx:0,ny:1,nz:0};
const anchors=[
 {node_id:'ANCHOR-01',x:-.55,y:0,z:-.45},{node_id:'ANCHOR-02',x:.50,y:0,z:-.30},
 {node_id:'ANCHOR-03',x:-.30,y:0,z:.55},{node_id:'ANCHOR-04',x:.60,y:0,z:.42}
];
const range=(a,d)=>Math.hypot(tag.x-a.x,tag.y+d-a.y,tag.z-a.z);
function rng(seed){let s=seed>>>0;return()=>{s=(1664525*s+1013904223)>>>0;return s/4294967296}}
function measurements(truth,noise,drop){return anchors.flatMap((a,i)=>drop.has(i)?[]:[{anchorId:a.node_id,anchor:a,corrected_m:range(a,truth)+noise[i],sigma_m:.008,nlos_score:Math.abs(noise[i])>.07?.82:.04}]);}
let worstFinal=0,worstPeak=0,totalReacq=0,totalQuarantine=0;
for(let seed=1;seed<=24;seed++){
 const R=rng(seed*7919),state={samples:[],d_m:0,v_mps:0,t:0};let st=state,now=1000,out,truth=0;
 // clean warmup
 for(let k=0;k<35;k++){now+=100;out=updateRobustSlidingWindow({state:st,tag,measurements:measurements(0,[.001,-.001,.001,-.001],new Set()),now,windowMs:1200,imu:{vibration:false,dynamic_g:.004,tilt_deg:0}});st=out.state;}
 let persistentBad=-1,badUntil=0,peakErr=0;
 for(let k=0;k<700;k++){
   now+=100;
   if(k===100||k===260||k===430){ // irregular real structural steps, max 60 mm
     truth=(R()-.5)*.12;
   }
   const noise=[0,0,0,0].map(()=>.004*(R()-.5));
   if(k===150||k===345){persistentBad=Math.floor(R()*4);badUntil=k+45+Math.floor(R()*25);}
   if(k<badUntil&&persistentBad>=0)noise[persistentBad]+=(R()<.5?-1:1)*(.10+.20*R());
   if(R()<.045){const i=Math.floor(R()*4);noise[i]+=(R()<.5?-1:1)*(.12+.45*R());}
   if(R()<.012){const c=(R()<.5?-1:1)*(.06+.08*R());for(let i=0;i<4;i++)noise[i]+=c;}
   const drop=new Set();if(R()<.045)drop.add(Math.floor(R()*4));if(R()<.008){drop.add(Math.floor(R()*4));drop.add(Math.floor(R()*4));}
   const vibration=R()<.11 || [100,260,430].some(x=>k>=x&&k<x+8);
   out=updateRobustSlidingWindow({state:st,tag,measurements:measurements(truth,noise,drop),now,windowMs:1200,imu:{vibration,dynamic_g:vibration?.06+.07*R():.003+.006*R(),tilt_deg:vibration?.4*(R()-.5):.01*(R()-.5)}});st=out.state;
   if(out.ready){const e=Math.abs(out.d_mm-truth*1000);peakErr=Math.max(peakErr,e);assert(Math.abs(out.d_mm)<155,`seed ${seed} runaway ${out.d_mm}`);if(out.reacquired)totalReacq++;totalQuarantine+=out.quarantined_anchors?.length||0;}
 }
 // final 8 s clean recovery at final truth
 for(let k=0;k<80;k++){now+=100;out=updateRobustSlidingWindow({state:st,tag,measurements:measurements(truth,[.001,-.001,.001,-.001],new Set()),now,windowMs:1200,imu:{vibration:false,dynamic_g:.004,tilt_deg:0}});st=out.state;}
 const finalErr=Math.abs(out.d_mm-truth*1000);worstFinal=Math.max(worstFinal,finalErr);worstPeak=Math.max(worstPeak,peakErr);
 assert(finalErr<10,`seed ${seed} did not recover: truth=${truth*1000} out=${out.d_mm} err=${finalErr}`);
}
console.log('PASS R13.5 random 24-seed fault matrix',JSON.stringify({worstFinalErrMm:worstFinal,worstPeakFaultErrMm:worstPeak,totalReacquisitions:totalReacq,quarantineObservations:totalQuarantine}));
