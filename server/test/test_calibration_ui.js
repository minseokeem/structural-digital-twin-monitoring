const fs=require('fs'),vm=require('vm'),assert=require('assert/strict');
const s=fs.readFileSync(require('path').join(__dirname,'../public/app.js'),'utf8');
const code=s.slice(s.indexOf('  function updateCalibrationFromSnapshot('),s.indexOf('\n  $("sidebarCloseBtn")',s.indexOf('  function updateCalibrationFromSnapshot(')));
let displayed={};const state={tagCount:3,calibrationRequested:true,calibrationZeroPending:false,calibrationGeneration:7,monitoringActive:false};
const c={state,mappedModule:(type,i)=>`TAG-0${i}`,showCalibrationProgress:(pct,text)=>{displayed={pct,text}},
 setFlowStage:()=>{},$:()=>({classList:{add(){}},style:{}}),addEvent:()=>{},setTimeout:()=>{},hideCalibrationToast:()=>{},renderStatsDashboard:()=>{}};
vm.createContext(c);vm.runInContext(code,c);
const raw=(i,p)=>({node_id:`TAG-0${i}`,online:true,range_count:4,uwb_calibration:{ready:p===1,progress:p,target_samples:40,min_samples:p*40}});
c.updateCalibrationFromSnapshot({calibration_generation:6,sensors:[raw(1,1),raw(2,1),raw(3,1)]});assert(!state.monitoringActive);
state.calibrationZeroPending=true;
c.updateCalibrationFromSnapshot({calibration_generation:7,sensors:[raw(1,1),raw(2,1),raw(3,1)]});assert(!state.monitoringActive);
state.calibrationZeroPending=false;
c.updateCalibrationFromSnapshot({calibration_generation:7,sensors:[raw(1,1),raw(2,1)]});
assert(!state.monitoringActive);assert.equal(displayed.pct,0);assert(displayed.text.includes('TAG-03: 연결 대기'));
c.updateCalibrationFromSnapshot({calibration_generation:7,sensors:[{...raw(1,0),tdma:{status:'start_ack_missing'}},raw(2,.8),raw(3,.4)]});
assert(displayed.text.includes('시작 명령 응답 대기'));assert.equal(displayed.pct,0);assert.equal(displayed.text.split('\n').length,3);
c.updateCalibrationFromSnapshot({calibration_generation:7,sensors:[raw(1,1),raw(2,1),raw(3,1)]});assert(state.monitoringActive);assert(!state.calibrationRequested);
console.log('PASS UI: generation isolation, pending Zero, missing/offline Tags, honest 0%, per-Tag diagnosis, complete only after all Tags ready.');
