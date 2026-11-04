const assert=require('assert/strict'),path=require('path');
const {engine}=require('./test_uwb_compensation');
function run(file){
  const e=engine(file,.45,1,123,'stable',{UWB_CAL_SAMPLES_V10_1:'180',UWB_CAL_MIN_MS_V10_1:'15000'});
  e.c.moduleRegistry=new Map(['TAG-01','TAG-02','TAG-03'].map(node_id=>[node_id,{node_id,role:'tag',last_seen_ms:1e12}]));
  e.c.NODE_OFFLINE_MS=6000;
  let r;
  for(let i=0;i<220;i++)r=e.step(0,{noise:0,dt:100});
  return r;
}
if(process.env.ORIGINAL_SERVER){
  const r=run(process.env.ORIGINAL_SERVER);
  assert.equal(r.calibration.ready,false);
  console.log('REPRODUCED original continuous 10 Hz multi-Tag calibration never completes:',r.calibration.progress);
}
const r=run(path.join(__dirname,'..','server.js'));
assert.equal(r.calibration.ready,true);
assert(r.calibration.min_samples>=40);
console.log('PASS same 10 Hz multi-Tag case completes; calibration retains required time span.');
