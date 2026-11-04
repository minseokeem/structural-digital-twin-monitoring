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
function make(noise,drop=new Set(),sigma=.008){return anchors.flatMap((a,i)=>drop.has(i)?[]:[{anchorId:a.node_id,anchor:a,corrected_m:rr(a)+(noise[i]||0),sigma_m:sigma,nlos_score:Math.abs(noise[i]||0)>.07?.88:.03}]);}
let worstPeak=0,worstFinal=0,totalQ=0,totalReject=0;
for(let seed=1;seed<=48;seed++){
 const R=rng(seed*104729),state={samples:[],d_m:0,v_mps:0,t:0};let st=state,now=1000,out;
 for(let k=0;k<40;k++){now+=100;out=updateRobustSlidingWindow({state:st,tag,measurements:make([.001,-.001,.001,-.001]),now,windowMs:1200,imu:{vibration:false,dynamic_g:.004,tilt_deg:0}});st=out.state;}
 let bad=-1,badUntil=0,driftStart=-1,driftSign=1,peak=0;
 for(let k=0;k<1800;k++){ // 3 minutes stationary with irregular faults
   now+=100;const noise=[0,0,0,0].map(()=>.005*(R()-.5));
   // Slowly drifting link, like the real A1 contamination observed in CSV.
   if(k%500===80){bad=Math.floor(R()*4);driftStart=k;badUntil=k+180+Math.floor(R()*100);driftSign=R()<.5?-1:1;}
   if(k<badUntil&&bad>=0){const f=Math.min(1,(k-driftStart)/90);noise[bad]+=driftSign*(.02+.14*f);}
   // Single catastrophic link spikes.
   if(R()<.035){const i=Math.floor(R()*4);noise[i]+=(R()<.5?-1:1)*(.10+.60*R());}
   // Short packet-wide RF/timestamp burst.
   if(R()<.008){const c=(R()<.5?-1:1)*(.05+.12*R());for(let i=0;i<4;i++)noise[i]+=c;}
   // One or two anchor packet losses.
   const drop=new Set();if(R()<.05)drop.add(Math.floor(R()*4));if(R()<.008){drop.add(Math.floor(R()*4));drop.add(Math.floor(R()*4));}
   // Shared floor vibration/footsteps without structural displacement.
   const vib=R()<.12;
   out=updateRobustSlidingWindow({state:st,tag,measurements:make(noise,drop),now,windowMs:1200,imu:{vibration:vib,dynamic_g:vib?.06+.08*R():.003+.006*R(),tilt_deg:vib?.45*(R()-.5):.015*(R()-.5)}});st=out.state;
   if(out.ready){peak=Math.max(peak,Math.abs(out.d_mm));totalQ+=(out.quarantined_anchors||[]).length;totalReject+=out.packet_rejected||out.state_guard_rejected?1:0;assert(Math.abs(out.d_mm)<25,`seed ${seed}: false stationary displacement ${out.d_mm}mm`);}
 }
 // clean recovery period
 for(let k=0;k<60;k++){now+=100;out=updateRobustSlidingWindow({state:st,tag,measurements:make([.001,-.001,.001,-.001]),now,windowMs:1200,imu:{vibration:false,dynamic_g:.004,tilt_deg:0}});st=out.state;}
 const fin=Math.abs(out.d_mm);worstPeak=Math.max(worstPeak,peak);worstFinal=Math.max(worstFinal,fin);
 assert(fin<6,`seed ${seed}: no stationary recovery ${out.d_mm}mm`);
}
console.log('PASS R13.5 stationary 48-seed fault matrix',JSON.stringify({worstFalseDisplacementMm:worstPeak,worstFinalRecoveryMm:worstFinal,totalQuarantineObservations:totalQ,totalRejectedUpdates:totalReject}));
