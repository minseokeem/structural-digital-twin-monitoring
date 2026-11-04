'use strict';
const assert=require('assert/strict');
const {planAdaptiveRanging}=require('../lib/adaptive-anchor-selector');
const tag={node_id:'TAG-01',x:0,y:.45,z:0,nx:0,ny:1,nz:0};
const anchors=[
 {node_id:'ANCHOR-01',x:-.6,y:0,z:-.3},
 {node_id:'ANCHOR-02',x:.6,y:.30,z:-.3},
 {node_id:'ANCHOR-03',x:-.6,y:0,z:.3},
 {node_id:'ANCHOR-04',x:.6,y:.30,z:.3}
];
const calibration={links:Object.fromEntries(anchors.map(a=>[a.node_id,{noise_m:.012,runtime_noise_m:.010}]))};
const baseRanges=anchors.map(a=>({anchor_id:a.node_id,success_rate:98,nlos_score:0}));
let state={lastFullSweepMs:1000};
let p=planAdaptiveRanging({tag,anchors,sensor:{measurement_valid:true,last_measurement_ms:3000,ranges:baseRanges,accel_g:.005},calibration,state,now:3000,fullSweepMs:5000});
assert.equal(p.targetLinks,2);assert.equal(p.anchorIds.length,2);assert(p.anchorMask>0);
console.log('PASS stable -> best two',p.anchorIds,p.scores);

p=planAdaptiveRanging({tag,anchors,sensor:{measurement_valid:true,last_measurement_ms:3500,ranges:baseRanges,accel_g:.032},calibration,state,now:3500,fullSweepMs:5000});
assert.equal(p.targetLinks,3);console.log('PASS soft quality -> three',p.anchorIds);

p=planAdaptiveRanging({tag,anchors,sensor:{measurement_valid:true,last_measurement_ms:4000,ranges:baseRanges,vibration:true,accel_g:.08},calibration,state,now:4000,fullSweepMs:5000});
assert.equal(p.targetLinks,4);assert.equal(p.anchorMask,15);console.log('PASS IMU event -> four');

p=planAdaptiveRanging({tag,anchors,sensor:{measurement_valid:true,last_measurement_ms:7000,ranges:baseRanges,accel_g:.005},calibration,state,now:7000,fullSweepMs:5000});
assert([3,4].includes(p.targetLinks)); // hysteresis after the hard event
console.log('PASS hysteresis',p.profile,p.reason);

state={lastFullSweepMs:1000};
p=planAdaptiveRanging({tag,anchors,sensor:{measurement_valid:true,last_measurement_ms:7000,ranges:baseRanges,accel_g:.005},calibration,state,now:7000,fullSweepMs:5000});
assert.equal(p.targetLinks,4);assert.equal(p.reason,'periodic_health_sweep');console.log('PASS periodic full sweep');

state={lastFullSweepMs:7000};
const bad=baseRanges.map(x=>({...x}));bad[0].success_rate=30;bad[0].nlos_score=.9;
p=planAdaptiveRanging({tag,anchors,sensor:{measurement_valid:true,last_measurement_ms:7100,ranges:bad,accel_g:.005},calibration,state,now:7100,fullSweepMs:5000});
assert.equal(p.targetLinks,4);assert(p.scores.find(x=>x.anchor_id==='ANCHOR-01').score<p.scores.find(x=>x.anchor_id==='ANCHOR-02').score);
console.log('PASS bad/NLOS link penalized');

state={lastFullSweepMs:7100};
const sweepBad=baseRanges.map((x,i)=>({...x,m:.8,scheduled:true}));sweepBad[0].success_rate=25;sweepBad[0].nlos_score=.92;
p=planAdaptiveRanging({tag,anchors,sensor:{measurement_valid:true,last_measurement_ms:7200,ranges:sweepBad,accel_g:.005,requested_links:4,anchor_mask:15,adaptive_event:false},calibration,state,now:7200,fullSweepMs:5000});
assert.equal(p.targetLinks,2);assert(!p.anchorIds.includes('ANCHOR-01'),'known bad full-sweep Anchor should be excluded, not trap VERIFY4');
console.log('PASS full-sweep NLOS does not trap VERIFY4',p.anchorIds);

state={lastFullSweepMs:7200};
const rememberedBad=baseRanges.map((x,i)=>({...x,m:i===0?null:.8,scheduled:i!==0}));rememberedBad[0].nlos_score=.92;rememberedBad[0].success_rate=90;
p=planAdaptiveRanging({tag,anchors,sensor:{measurement_valid:true,last_measurement_ms:7250,ranges:rememberedBad,accel_g:.005,requested_links:2,anchor_mask:6,adaptive_event:false},calibration,state,now:7250,fullSweepMs:5000});
assert.equal(p.targetLinks,2);assert(!p.anchorIds.includes('ANCHOR-01'),'remembered NLOS Anchor should stay de-prioritized while unscheduled');
console.log('PASS remembered NLOS remains excluded between health sweeps',p.anchorIds);

state={lastFullSweepMs:7200};
const activeBad=baseRanges.map((x,i)=>({...x,m:.8,scheduled:i<2}));activeBad[0].nlos_score=.85;
p=planAdaptiveRanging({tag,anchors,sensor:{measurement_valid:true,last_measurement_ms:7300,ranges:activeBad,accel_g:.005,requested_links:2,anchor_mask:3,adaptive_event:false},calibration,state,now:7300,fullSweepMs:5000});
assert.equal(p.targetLinks,4);console.log('PASS NLOS on active pair escalates to VERIFY4');

