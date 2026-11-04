const {engine}=require('./test_uwb_compensation');
const assert=require('assert/strict');
const file=require('path').join(__dirname,'..','server.js');
const e=engine(file,.45,1,123,'stable');
let r;e.calibrate();
const quiet=[];
for(let i=0;i<260;i++){r=e.step(0,{noise:.006});if(r.valid)quiet.push(r.dz_mm);}
const before=quiet.at(-1);
assert(Number.isFinite(before));
// No mode switch is performed. The same state must follow a persistent 30 mm step.
let reached=null;const path=[];
for(let i=1;i<=70;i++){
  r=e.step(-.03,{noise:.003,dynamic_g:i<10?.08:.01,vibration:i<5});
  if(r.valid)path.push(r.dz_mm);
  if(reached===null&&r.valid&&Math.abs(r.dz_mm+30)<5)reached=i*100;
}
assert(reached&&reached<=2500,`unified estimator did not follow persistent motion quickly enough: ${reached}`);
assert(Math.abs(r.dz_mm+30)<4,`unified final error too large: ${r.dz_mm}`);
// Return to quiet state without resetting estimator.
for(let i=0;i<120;i++)r=e.step(-.03,{noise:.005,dynamic_g:.005,vibration:false});
assert(Math.abs(r.dz_mm+30)<5,'quiet settling changed the real displacement');
console.log('PASS: Unified Adaptive continuity; no filter-mode state reset is required.');
console.log(JSON.stringify({before_mm:before,step_30mm_reached_ms:reached,final_mm:r.dz_mm},null,2));
