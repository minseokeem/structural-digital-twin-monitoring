'use strict';
const assert=require('assert');const {updateRobustSlidingWindow}=require('../lib/robust-sliding-window');
const anchors=[{node_id:'ANCHOR-01',x:-.55,y:0,z:-.45},{node_id:'ANCHOR-02',x:.50,y:0,z:-.30},{node_id:'ANCHOR-03',x:-.30,y:0,z:.55},{node_id:'ANCHOR-04',x:.60,y:0,z:.42}];
const tags=[{id:'T1',x:-.25,y:.42,z:0,nx:0,ny:1,nz:0},{id:'T2',x:0,y:.42,z:0,nx:0,ny:1,nz:0},{id:'T3',x:.25,y:.42,z:0,nx:0,ny:1,nz:0}];
const states=new Map(tags.map(t=>[t.id,{samples:[],d_m:0,v_mps:0,t:0}]));let now=1000;
const rr=(t,a,d)=>Math.hypot(t.x-a.x,t.y+d-a.y,t.z-a.z);
function run(t,d,noise,vib){const ms=anchors.map((a,i)=>({anchorId:a.node_id,anchor:a,corrected_m:rr(t,a,d)+(noise[i]||0),sigma_m:.008,nlos_score:Math.abs(noise[i]||0)>.08?.85:.03}));const o=updateRobustSlidingWindow({state:states.get(t.id),tag:t,measurements:ms,now,windowMs:1200,imu:{vibration:vib,dynamic_g:vib?.09:.004,tilt_deg:0}});states.set(t.id,o.state);return o;}
let outs={};for(let k=0;k<40;k++){now+=100;for(const t of tags)outs[t.id]=run(t,0,[.001,-.001,.001,-.001],false);}
let max2=0,max3=0,reacq1=0;
for(let k=0;k<40;k++){now+=100;
 outs.T1=run(tags[0],.060,[.003,.650,-.003,.002],k<18); // actual 60mm + catastrophic A2
 outs.T2=run(tags[1],0,[-.140,.002,-.002,.002],k<18); // stationary + bad A1
 outs.T3=run(tags[2],0,[.002,-.002,.002,-.002],k<18); // healthy but same vibration
 if(outs.T1.reacquired)reacq1++;max2=Math.max(max2,Math.abs(outs.T2.d_mm||0));max3=Math.max(max3,Math.abs(outs.T3.d_mm||0));
}
assert(outs.T1.d_mm>45&&outs.T1.d_mm<75,`T1 failed real 60mm ${outs.T1.d_mm}`);assert(reacq1>0,'T1 no reacquisition');
assert(max2<12,`T2 contaminated by A1 ${max2}`);assert(max3<10,`T3 moved from shared vibration ${max3}`);
console.log('PASS three-tag shared event',{T1:outs.T1.d_mm,T2:outs.T2.d_mm,T3:outs.T3.d_mm,T2peak:max2,T3peak:max3,T1reacq:reacq1,T1q:outs.T1.quarantined_anchors,T2q:outs.T2.quarantined_anchors});
