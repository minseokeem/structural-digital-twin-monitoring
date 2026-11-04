'use strict';
const assert=require('assert');
const {updateRobustSlidingWindow}=require('../lib/robust-sliding-window');
const tag={x:0,y:.42,z:0,nx:0,ny:1,nz:0};
const anchors=[
 {node_id:'ANCHOR-01',x:-.55,y:0,z:-.45},{node_id:'ANCHOR-02',x:.50,y:0,z:-.30},
 {node_id:'ANCHOR-03',x:-.30,y:0,z:.55},{node_id:'ANCHOR-04',x:.60,y:0,z:.42}
];
const rr=(a,d=0)=>Math.hypot(tag.x-a.x,tag.y+d-a.y,tag.z-a.z);
function rng(seed){let s=seed>>>0;return()=>{s=(1664525*s+1013904223)>>>0;return s/4294967296}}
function make(truth,noise,drop=new Set()){return anchors.flatMap((a,i)=>drop.has(i)?[]:[{anchorId:a.node_id,anchor:a,corrected_m:rr(a,truth)+(noise[i]||0),sigma_m:.008,nlos_score:Math.abs(noise[i]||0)>.07?.88:.03}]);}
let worstSettled=0,worstHealthyTransient=0,totalReacq=0;
for(let seed=1;seed<=32;seed++){
 const R=rng(seed*65537),state={samples:[],d_m:0,v_mps:0,t:0};let st=state,now=1000,out,truth=0;
 for(let k=0;k<40;k++){now+=100;out=updateRobustSlidingWindow({state:st,tag,measurements:make(0,[.001,-.001,.001,-.001]),now,windowMs:1200,imu:{vibration:false,dynamic_g:.004,tilt_deg:0}});st=out.state;}
 const steps=[.060,.020,-.035,.045,0];
 for(let sidx=0;sidx<steps.length;sidx++){
   truth=steps[sidx];const bad=Math.floor(R()*4),badBias=(R()<.5?-1:1)*(.15+.45*R());
   for(let k=0;k<70;k++){
     now+=100;const noise=[0,0,0,0].map(()=>.003*(R()-.5));
     // For the first ~2 seconds after a true displacement, corrupt one independent link heavily.
     if(k<22)noise[bad]+=badBias;
     if(R()<.03){const i=Math.floor(R()*4);noise[i]+=(R()<.5?-1:1)*(.1+.3*R());}
     const drop=new Set();if(R()<.035)drop.add(Math.floor(R()*4));
     const vib=k<14 || R()<.04;
     out=updateRobustSlidingWindow({state:st,tag,measurements:make(truth,noise,drop),now,windowMs:1200,imu:{vibration:vib,dynamic_g:vib?.07+.07*R():.004,tilt_deg:vib?.35*(R()-.5):0}});st=out.state;
     if(out.ready){const e=Math.abs(out.d_mm-truth*1000);if(k>=35)worstSettled=Math.max(worstSettled,e);else if(k>=8&&k<22)worstHealthyTransient=Math.max(worstHealthyTransient,e);if(out.reacquired)totalReacq++;assert(Math.abs(out.d_mm)<155,'physical cap escaped');}
   }
   assert(Math.abs(out.d_mm-truth*1000)<12,`seed ${seed}, step ${truth*1000}: failed to settle (${out.d_mm})`);
 }
}
console.log('PASS R13.5 dynamic 32-seed reacquisition matrix',JSON.stringify({worstSettledErrorMm:worstSettled,worstTransientTrackingErrorMm:worstHealthyTransient,totalReacquisitions:totalReacq}));
