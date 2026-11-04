const fs=require('fs'),vm=require('vm'),path=require('path'),assert=require('assert/strict');
const {measurementSigmaM}=require('../lib/information-ranging');
const {updateRobustSlidingWindow}=require('../lib/robust-sliding-window');
function engine(file, height=0.45, span=1,initialSeed=0x12345678,filterMode="responsive",calibration={}){
 const src=fs.readFileSync(file,'utf8');
 const slice=(a,b)=>src.slice(src.indexOf(a),src.indexOf(b,src.indexOf(a)));
 let time=100000;
 const c={console,process:{env:{UWB_FILTER_MODE:filterMode,UWB_FILTER_MODE_V9:filterMode,UWB_CAL_SAMPLES:'200',UWB_CAL_MIN_MS:'20000',...calibration}},Date:{now:()=>time},Map,Math,Number,Object,Array,String,Boolean,JSON,Set};
 c.physicalLayout={metric:true,tags:[{node_id:'TAG-01',x:0,y:height,z:0}],anchors:[[-span,0,-span],[span,0,-span],[-span,0,span],[span,0,span]].map((a,i)=>({node_id:`ANCHOR-0${i+1}`,x:a[0],y:a[1],z:a[2]}))};
 c.rangeBaselines=new Map();c.uwbCalibration=new Map();c.uwbMotionState=new Map();c.uwbFastMotionState=new Map();c.adaptiveRangingState=new Map();c.robustWindowState=new Map();c.UWB_Z_SIGN=1;c.UWB_Z_SCALE=1;c.UWB_CAL_MIN_READY_LINKS=3;c.UWB_SLIDING_WINDOW_MS=4000;c.UWB_INFO_TARGET_SIGMA_MM=4;c.measurementSigmaM=measurementSigmaM;c.updateRobustSlidingWindow=updateRobustSlidingWindow;
 vm.createContext(c);vm.runInContext(slice('function median(values) {','function moduleSnapshot() {')+slice('function currentRangeMap(ranges) {','function observeAnchorViaTag(anchorId, tagNodeId) {')+slice('const UWB_CAL_SAMPLES =','function normalizePacket(msg, rinfo) {'),c);
 let seed=initialSeed;
 const random=()=>{seed=(1664525*seed+1013904223)>>>0;return seed/4294967296;};
 const normal=()=>Math.sqrt(-2*Math.log(Math.max(1e-9,random())))*Math.cos(2*Math.PI*random());
 const biases=[0.12,-0.08,0.2,0.05];
 function step(dy=0,opt={}){
   time+=opt.dt||100;
   const ranges=c.physicalLayout.anchors.map((a,i)=>({anchor_id:a.node_id,ok:!(opt.missing||[]).includes(i),success_rate:100,
     m:Math.hypot(a.x,height+dy-a.y,a.z)+biases[i]+normal()*(Array.isArray(opt.noise)?opt.noise[i]:(opt.noise??0.008))+(opt.spike===i?0.55:0)+(opt.common||0)}));
   return c.estimateDzMm('TAG-01',ranges,opt.imu||{});
 }
 function calibrate(){let r;for(let i=0;i<240;i++)r=step(0,{noise:0.006});assert(r.calibration?.ready,'calibration never completed');return r;}
 return {c,step,calibrate,biases};
}
module.exports={engine};
if(require.main===module){
const file=path.join(__dirname,'..','server.js');
const e=engine(file);e.calibrate();
Object.values(e.c.uwbCalibration.get('TAG-01').links).forEach((l,i)=>assert(Math.abs(l.bias_m-e.biases[i])<0.005,'bias error'));
let stationary=[];for(let i=0;i<600;i++){const r=e.step();assert(r.valid);stationary.push(r.dz_mm);}
const rms=a=>Math.sqrt(a.reduce((s,x)=>s+x*x,0)/a.length);
assert(rms(stationary)<12,'stationary noise amplification');
let r;for(let i=0;i<100;i++)r=e.step(-0.01,{noise:0.001});assert(Math.abs(r.dz_mm+10)<3,'10 mm descent erased');
for(let i=0;i<100;i++)r=e.step(0.1,{noise:0.001});assert(Math.abs(r.dz_mm-100)<3,'100 mm step erased');
for(let i=0;i<3;i++)r=e.step(0.1,{noise:0.001,spike:0});assert(Math.abs(r.dz_mm-100)<15,'one-anchor spike');
for(let i=0;i<40;i++)r=e.step(0.1,{missing:[0],noise:0.001});assert(r.valid&&Math.abs(r.dz_mm-100)<5,'three-anchor operation');
for(let i=0;i<40;i++)r=e.step(0.1,{missing:[0,1],noise:0.001});assert(r.valid&&Math.abs(r.dz_mm-100)<8,'two-anchor adaptive operation');
r=e.step(0.1,{missing:[0,1,2,3]});assert.equal(r.dz_mm,null);assert.equal(r.method,'no_range');
for(let i=0;i<100;i++)r=e.step(0.1,{noise:0.001});assert(Math.abs(r.dz_mm-100)<4,'reconnection');
const frozen=Object.values(e.c.uwbCalibration.get('TAG-01').links).map(l=>l.bias_m);
for(let i=0;i<600;i++)r=e.step(0.1-i*0.0001,{noise:0.001});
assert(Math.abs(r.dz_mm-40)<5,'slow deformation erased');
assert.deepEqual(Object.values(e.c.uwbCalibration.get('TAG-01').links).map(l=>l.bias_m),frozen,'bias adapted during deformation');
const weak=engine(file,0.01,1);for(let i=0;i<240;i++)r=weak.step(0,{noise:0.001});assert.equal(r.valid,true);assert.equal(r.reference_only,true);assert(Number.isFinite(r.dz_mm));
const moving=engine(file);for(let i=0;i<300;i++)r=moving.step(i*0.002,{noise:0.001});assert(!r.calibration.ready,'moving calibration accepted');
for(let i=0;i<240;i++)r=moving.step(0,{noise:0.001});assert(r.calibration.ready,'stable window did not recover');
const imu=engine(file);for(let i=0;i<100;i++)r=imu.step(0,{imu:{vibration:true}});assert(!r.calibration.ready,'vibration calibration accepted');
e.c.physicalLayout.metric=false;r=e.step();assert.equal(r.dz_mm,null);assert.equal(r.method,'coordinates_required');
const comparison={new_stationary_rmse_mm:rms(stationary)};
if(process.env.UWB_ORIGINAL_SERVER){const old=engine(process.env.UWB_ORIGINAL_SERVER);old.calibrate();const values=[];for(let i=0;i<600;i++)values.push(old.step().dz_mm);comparison.original_stationary_rmse_mm=rms(values);}
console.log('PASS: fixed calibration, noise, steps, slow motion, spike, dropout, reconnect, poor geometry, moving calibration, IMU, unknown scale');
console.log(JSON.stringify(comparison,null,2));

}
