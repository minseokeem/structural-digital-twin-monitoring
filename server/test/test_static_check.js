const {StaticCheck}=require('../public/static-check');
const assert=require('assert/strict');
const packet=(seq,v=0,extra={})=>({node_id:'TAG-01',boot_id:'b',seq,last_seen_ms:seq*100,
  online:true,measurement_valid:true,dz_mm:v,averaging_ready:true,
  uwb_calibration:{ready:true,id:42},uwb_filter:{mode:'stable'},...extra});
function run(value,extra={}){const c=new StaticCheck();c.start(0);for(let i=1;i<=601;i++)c.push(packet(i,value(i),extra),i*100);return c;}
let c=run(i=>4*Math.sin(i));assert(c.result.pass);assert.equal(c.result.samples,601);
c=run(i=>i===300?5.01:0);assert(!c.result.pass);assert.equal(c.result.max_abs_mm,5.01);
c=run(()=>8);assert(!c.result.pass);assert.equal(c.result.mean_mm,8);
c=new StaticCheck();c.start(0);c.push(packet(1,0,{averaging_ready:false}),100);assert.equal(c.status,'waiting');
c.push(packet(2),200);c.push(packet(3,0,{uwb_calibration:{ready:true,id:43}}),300);assert.equal(c.status,'interrupted');
c=new StaticCheck();c.start(0);c.push(packet(1),100);c.tick(60100);assert(c.result&&!c.result.enough&&!c.result.pass);
c=run(()=>0,{vibration:true});assert(!c.result.pass&&c.result.movement);
c=run(()=>0,{measurement_valid:false});assert.equal(c.status,'waiting');
c=new StaticCheck();c.start(0);
for(let i=1;i<=601;i++)c.push(packet(i,0,{averaging_ready:i<300}),i*100);
assert(c.result&&!c.result.enough&&!c.result.pass,'re-warming data counted as valid');
console.log('PASS: exact ±5 mm bounds, constant offset, warmup exclusion, recalibration abort, missing packets, vibration.');
