const {engine}=require('./test_uwb_compensation');
const path=require('path'),assert=require('assert/strict');
const file=path.join(__dirname,'..','server.js');
function response(height=.45){
 const e=engine(file,height,1,123,'unified');let r;
 for(let i=0;i<240;i++)r=e.step(0,{noise:0});
 let reached=null;for(let i=1;i<=120;i++){r=e.step(-.03,{noise:0,imu:{dynamic_g:i<8?.07:.005,vibration:i<3}});if(reached===null&&r.valid&&Math.abs(r.dz_mm+30)<3)reached=i*100;}
 assert(Math.abs(r.dz_mm+30)<.5,'sustained displacement lost');return {e,r,reached};
}
const main=response();assert(main.reached<=3000,'Unified Adaptive response is too slow');
const weak=response(.1);assert(weak.r.valid);assert(weak.r.reference_only);assert.equal(weak.r.quality_warning,'weak_vertical_geometry');assert(weak.r.resolution_limited);assert(Number.isFinite(weak.r.dz_mm));
const flat=engine(file,0,1,123,'unified');let r;
for(let i=0;i<240;i++)r=flat.step(0,{noise:0});assert.equal(r.dz_mm,null);assert.equal(r.valid,false);
r=main.e.step(-.03,{noise:0,missing:[0,1,2,3]});assert.equal(r.dz_mm,null);
console.log('PASS: Unified Adaptive latency, weak-geometry reference value, zero-sensitivity and missing-data handling.');
console.log(JSON.stringify({unified_30mm_90pct_ms:main.reached,weak_geometry_displacement_mm:weak.r.dz_mm,weak_geometry_sigma_mm:weak.r.uncertainty_mm},null,2));
