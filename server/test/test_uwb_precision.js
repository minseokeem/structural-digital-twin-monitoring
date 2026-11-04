const {engine:makeEngine}=require('./test_uwb_compensation');
function engine(file,height=.45,span=1,seed=0x12345678){return makeEngine(file,height,span,seed,'unified');}
const path=require('path'),assert=require('assert/strict');
const file=path.join(__dirname,'..','server.js');
const rms=values=>Math.sqrt(values.reduce((s,x)=>s+x*x,0)/values.length);
function stationary(file,seed,noise){
 const e=engine(file,.45,1,seed);
 for(let i=0;i<300;i++)e.step(0,{noise});
 const values=[];for(let i=0;i<600;i++){const r=e.step(0,{noise});if(r.dz_mm!=null)values.push(r.dz_mm);}
 assert(values.length>570,'too many invalid outputs');return rms(values);
}
const seeds=[1,12,123,1234,12345];const report={};
for(const [name,noise] of [['equal_noise',[.012,.012,.012,.012]],['one_noisy_anchor',[.006,.008,.035,.008]]]){
 const current=seeds.map(seed=>stationary(file,seed,noise));report[name]={r7_mean_rmse_mm:current.reduce((a,b)=>a+b,0)/current.length};
 if(process.env.UWB_ORIGINAL_SERVER){const before=seeds.map(seed=>stationary(process.env.UWB_ORIGINAL_SERVER,seed,noise));const old=before.reduce((a,b)=>a+b,0)/before.length;report[name].r6_mean_rmse_mm=old;assert(report[name].r7_mean_rmse_mm<old*.85,'precision improvement not demonstrated');}
}
const e=engine(file);for(let i=0;i<240;i++)e.step(0,{noise:0});
let r,settled=null;for(let i=1;i<=120;i++){r=e.step(-.03,{noise:0});if(settled===null&&Math.abs(r.dz_mm+30)<3)settled=i*100;}
assert(settled && settled<=7000,'30 mm response too slow');assert(Math.abs(r.dz_mm+30)<.2,'steady step biased');report.step_30mm_90pct_ms=settled;
// Three simultaneous range bursts plus IMU events cannot bypass the filter.
for(let i=0;i<3;i++)e.step(-.03,{noise:0,common:.1,imu:{vibration:true}});
let peak=0;for(let i=0;i<60;i++){r=e.step(-.03,{noise:0});peak=Math.max(peak,Math.abs(r.dz_mm+30));}assert(peak<3,'short common burst leaked excessively');report.common_burst_peak_mm=peak;
// Reconnect after a real displacement: don't fabricate a fresh zero-valued estimate.
e.step(-.03,{noise:0,missing:[0,1,2,3],dt:3000});
for(let i=0;i<30;i++){r=e.step(-.03,{noise:0});if(r.valid)assert(Math.abs(r.dz_mm+30)<1,'reconnect moved toward zero');}
// Explicitly test low packet rate with the same time-window implementation.
const low=engine(file);for(let i=0;i<230;i++)r=low.step(0,{noise:.001,dt:500});assert(r.valid,'2 Hz unified mode cannot initialize');
console.log('PASS: Unified multi-seed stationary noise, uneven noise, response time, common bursts, reconnect at nonzero displacement, 2 Hz input.');
console.log(JSON.stringify(report,null,2));
