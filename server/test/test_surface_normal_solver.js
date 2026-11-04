const fs=require('fs'),vm=require('vm'),path=require('path'),assert=require('assert/strict');
const {measurementSigmaM}=require('../lib/information-ranging');
const {updateRobustSlidingWindow}=require('../lib/robust-sliding-window');
const file=path.join(__dirname,'..','server.js');
const src=fs.readFileSync(file,'utf8');
const slice=(a,b)=>src.slice(src.indexOf(a),src.indexOf(b,src.indexOf(a)));
function make(normal,base=[0,0.6,0]){
  let time=100000;
  const c={console,process:{env:{UWB_FILTER_MODE_V9:'responsive',UWB_CAL_SAMPLES:'120',UWB_CAL_MIN_MS:'12000'}},Date:{now:()=>time},Map,Math,Number,Object,Array,String,Boolean,JSON,Set};
  c.physicalLayout={metric:true,tags:[{node_id:'TAG-01',x:base[0],y:base[1],z:base[2],nx:normal[0],ny:normal[1],nz:normal[2]}],anchors:[
    {node_id:'ANCHOR-01',x:-1,y:0,z:-1},{node_id:'ANCHOR-02',x:1.2,y:0.2,z:-1},
    {node_id:'ANCHOR-03',x:-1.1,y:1.5,z:1},{node_id:'ANCHOR-04',x:1,y:1.8,z:1.2}
  ]};
  c.rangeBaselines=new Map();c.uwbCalibration=new Map();c.uwbMotionState=new Map();c.uwbFastMotionState=new Map();c.adaptiveRangingState=new Map();c.robustWindowState=new Map();c.UWB_Z_SIGN=1;c.UWB_Z_SCALE=1;c.UWB_CAL_MIN_READY_LINKS=3;c.UWB_SLIDING_WINDOW_MS=1200;c.UWB_INFO_TARGET_SIGMA_MM=4;c.measurementSigmaM=measurementSigmaM;c.updateRobustSlidingWindow=updateRobustSlidingWindow;
  vm.createContext(c);vm.runInContext(slice('function median(values) {','function moduleSnapshot() {')+slice('function currentRangeMap(ranges) {','function observeAnchorViaTag(anchorId, tagNodeId) {')+slice('const UWB_CAL_SAMPLES =','function normalizePacket(msg, rinfo) {'),c);
  const bias=[.08,-.05,.11,.03];
  function step(mm){time+=100;const d=mm/1000;const p={x:base[0]+normal[0]*d,y:base[1]+normal[1]*d,z:base[2]+normal[2]*d};const ranges=c.physicalLayout.anchors.map((a,i)=>({anchor_id:a.node_id,ok:true,success_rate:100,m:Math.hypot(p.x-a.x,p.y-a.y,p.z-a.z)+bias[i]}));return c.estimateDzMm('TAG-01',ranges,{});}
  let r;for(let i=0;i<150;i++)r=step(0);assert(r.calibration?.ready,'calibration');
  return {step};
}
for(const [name,n] of [['wall-x',[1,0,0]],['ceiling-down',[0,-1,0]],['inclined',[Math.SQRT1_2,Math.SQRT1_2,0]]]){
  const e=make(n);let r;for(let i=0;i<70;i++)r=e.step(-25);assert(r.valid,name+' valid');assert(Math.abs(r.dz_mm+25)<2.5,`${name} displacement ${r.dz_mm}`);console.log(name,r.dz_mm.toFixed(2),r.normal_sensitivity.toFixed(3));
}
console.log('PASS: wall / ceiling / inclined Surface normal displacement solver');
