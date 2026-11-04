'use strict';
const assert=require('assert');
const {planInformationRanging}=require('../lib/information-ranging');

const tag={node_id:'TAG-01',x:0,y:0,z:0,nx:0,ny:1,nz:0};
const anchors=[
  {node_id:'ANCHOR-01',x:0,y:1,z:0},
  {node_id:'ANCHOR-02',x:1,y:.7,z:0},
  {node_id:'ANCHOR-03',x:1,y:.15,z:0},
  {node_id:'ANCHOR-04',x:-.4,y:.9,z:.1}
];
const calibration={links:{}};
for(const a of anchors)calibration.links[a.node_id]={calibrated:true,noise_m:.008,runtime_noise_m:.008};
const ranges=anchors.map(a=>({anchor_id:a.node_id,ok:true,m:1,success_rate:98,nlos_score:0,nlos_suspect:false,scheduled:true}));
const state={lastFullSweepMs:Date.now()};
let plan=planInformationRanging({tag,anchors,sensor:{ranges,uwb_window_sigma_mm:3},calibration,state,now:Date.now(),targetSigmaMm:4,minLinks:2});
assert(plan.targetLinks>=2&&plan.targetLinks<=4);
assert(plan.anchorIds.includes('ANCHOR-01')||plan.anchorIds.includes('ANCHOR-04'),'high-normal-sensitivity anchor should be selected');
assert(plan.scores[0].information>=plan.scores.at(-1).information,'scores should be information ordered');

const badRanges=ranges.map(r=>r.anchor_id==='ANCHOR-01'?{...r,nlos_score:.95,nlos_suspect:true,success_rate:55}:r);
plan=planInformationRanging({tag,anchors,sensor:{ranges:badRanges,uwb_window_sigma_mm:18,uwb_residual_rms_m:.03},calibration,state:{lastFullSweepMs:Date.now()},now:Date.now(),targetSigmaMm:4,minLinks:2});
const a1=plan.scores.find(x=>x.anchor_id==='ANCHOR-01');
const a4=plan.scores.find(x=>x.anchor_id==='ANCHOR-04');
assert(a1.sigma_mm>a4.sigma_mm,'NLOS must inflate sigma continuously');
assert(plan.targetLinks>=2,'minimum two-anchor safety floor');
console.log('PASS information ranging',JSON.stringify({targetLinks:plan.targetLinks,anchors:plan.anchorIds,projectedSigmaMm:plan.projectedSigmaMm}));
