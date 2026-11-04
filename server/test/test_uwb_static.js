const {engine}=require('./test_uwb_compensation');
const path=require('path'),assert=require('assert/strict');
const file=path.join(__dirname,'..','server.js');
const calibration={UWB_CAL_SAMPLES:'1200',UWB_CAL_MIN_MS:'120000'};
const seeds=[1,12,123,1234,12345];
const report=[];
for(const height of [.29,.8])for(const seed of seeds){
  const e=engine(file,height,1,seed,'stable',calibration);
  const noise=[.046,.027,.028,.033];let r;
  for(let i=0;i<1250;i++)r=e.step(0,{noise});
  assert(r.calibration.ready,'static calibration failed');
  const bias=Object.values(e.c.uwbCalibration.get('TAG-01').links).map(l=>l.bias_m);
  for(let i=0;i<1350;i++)r=e.step(0,{noise});
  assert(r.averaging_ready,'averaging never ready');
  const values=[];
  for(let i=0;i<600;i++){r=e.step(0,{noise});assert(r.valid);values.push(r.dz_mm);}
  const mean=values.reduce((a,b)=>a+b,0)/values.length;
  const rmse=Math.sqrt(values.reduce((a,b)=>a+b*b,0)/values.length);
  const max=Math.max(...values.map(Math.abs));
  report.push({height_m:height,seed,window_s:r.filter.window_ms/1000,mean_mm:mean,rmse_mm:rmse,max_abs_mm:max,inside_5mm_pct:100*values.filter(v=>Math.abs(v)<=5).length/values.length,goal_pass:max<=5});
  // A sustained deformation must not be clipped or zeroed, even in stable mode.
  for(let i=0;i<1500;i++)r=e.step(-.02,{noise:.001});
  assert(Math.abs(r.dz_mm+20)<10,'sustained 20 mm descent lost');
  report.at(-1).held_descent_mm=r.dz_mm;
  assert.deepEqual(Object.values(e.c.uwbCalibration.get('TAG-01').links).map(l=>l.bias_m),bias,'baseline adapted');
}
console.log('PASS: static averaging, fixed baseline, sustained descent. ±5 mm is evaluated, not assumed.');
console.log(JSON.stringify(report,null,2));
