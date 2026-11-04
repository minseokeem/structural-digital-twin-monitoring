'use strict';
const assert=require('assert/strict');
const {TDMAScheduler}=require('../lib/tdma-scheduler');

function simulate(mode){
 let now=100000,queue=[],serial=0,overlap=0;
 const count=mode==='single'?1:3;
 const modules=new Map();
 const devices=Array.from({length:count},(_,i)=>({id:`TAG-0${i+1}`,boot:(i+1).toString(16).padStart(8,'0'),offset:7000+i*12000,epoch:0,seq:0,seen:false,wasActive:false,until:0,packet:0,grants:[],enabled:true,mask:3,profile:2}));
 const later=(f,ms)=>queue.push({t:now+ms,f,k:serial++});
 let scheduler;
 function telemetry(d,type){
  if(!d.enabled)return;const active=now<d.until;const bad=mode==='rotating'&&d.id===devices[Math.floor((now-100000)/5000)%count].id;
  const msg={node_id:d.id,role:'tag',ip:d.id,online:true,slot_protocol:3,boot_id:d.boot,device_ms:(now-d.offset)>>>0,last_seen_ms:now,slot_seq:d.seq,server_epoch:d.epoch,ranging_mode:'adaptive_quality_v1',uwb_active:active,packet_seq:++d.packet,type};
  if(type==='tag_data')msg.ranges=[0,1,2,3].map(i=>({anchor_id:`ANCHOR-0${i+1}`,scheduled:!!(d.mask&(1<<i)),ok:!!(d.mask&(1<<i))&&!bad,m:!!(d.mask&(1<<i))&&!bad?.65:null}));
  const lose=mode==='stop_ack_loss'&&!active&&scheduler.current?.phase==='stopping';if(!lose){modules.set(d.id,msg);scheduler.observe(msg,msg);}
 }
 function deliver(d,c){
  if(!d.enabled||c.target_boot!==parseInt(d.boot,16)||c.server_epoch<d.epoch)return;
  if(c.server_epoch>d.epoch){if(c.active)return;d.epoch=c.server_epoch;d.seen=false;d.until=0;}
  const delta=(c.slot_seq-d.seq)>>>0;if(d.seen&&delta>=0x80000000)return;
  if(d.seen&&delta===0){if(c.active!==d.wasActive)return;telemetry(d,'hello');return;}
  if(c.active){const remain=(c.device_deadline_ms-((now-d.offset)>>>0))|0;if(remain<18||remain>2000)return;d.until=now+Math.min(remain,c.lease_ms);d.mask=c.anchor_mask;d.profile=c.range_profile;d.grants.push(now);if(devices.some(x=>x!==d&&x.until>now))overlap++;}
  else d.until=0;
  d.seq=c.slot_seq;d.seen=true;d.wasActive=c.active;telemetry(d,'hello');
 }
 scheduler=new TDMAScheduler({getTags:()=>[...modules.values()].map(m=>({...m,online:now-m.last_seen_ms<6000})),send:(m,c)=>{const d=devices.find(x=>x.id===m.node_id);if(mode==='stop_loss'&&!c.active&&scheduler.current?.phase==='stopping')return;const bad=mode==='rotating'&&d.id===devices[Math.floor((now-100000)/5000)%count].id;later(()=>deliver(d,c),c.active&&bad?220:5);},now:()=>now,later,serverEpoch:1234567890123,prepareMs:100,guardMs:25,liveMaxMs:260,calMaxMs:520,retryMs:20,getPolicy:()=>mode==='calibration'?({mode:'calibration',samples:3,minLinks:2,targetLinks:4,anchorMask:15,rangeProfile:'calibration'}):({mode:'live',samples:1,minLinks:2,targetLinks:2,anchorMask:5,rangeProfile:'normal'})});
 function heartbeat(d){telemetry(d,now<d.until?'tag_data':'hello');later(()=>heartbeat(d),70);}devices.forEach(heartbeat);scheduler.start();
 if(mode==='rejoin'){later(()=>devices[0].enabled=false,5000);later(()=>{devices[0].enabled=true;devices[0].boot='abcd1234';devices[0].seen=false;devices[0].until=0;},11000);}
 const end=now+30000;while(queue.length){queue.sort((a,b)=>a.t-b.t||a.k-b.k);const e=queue.shift();if(e.t>end)break;now=e.t;e.f();}scheduler.stop();
 assert.equal(overlap,0,`${mode}: overlap`);for(const d of devices)assert(d.grants.length>=10,`${mode}: starved ${d.id}`);
 const result=scheduler.snapshot();if(mode==='healthy'||mode==='calibration')assert(result.tags.every(t=>t.missed_slots===0),JSON.stringify(result.tags));
 if(mode==='rotating')assert(result.tags.some(t=>t.missed_slots>0));
 if(mode==='healthy')assert(result.recent_turns.every(t=>t.elapsed_ms<500),JSON.stringify(result.recent_turns));
 console.log('PASS',mode,{grants:devices.map(d=>d.grants.length),avg:Math.round(result.recent_turns.reduce((a,x)=>a+x.elapsed_ms,0)/Math.max(1,result.recent_turns.length))});
}
for(const mode of ['healthy','rotating','stop_loss','stop_ack_loss','calibration','single','rejoin'])simulate(mode);

let n=10000;const t={node_id:'TAG-01',boot_id:'00000001',online:true,role:'tag',ip:'a',slot_protocol:3,ranging_mode:'adaptive_quality_v1',device_ms:10,last_seen_ms:n};
const s=new TDMAScheduler({getTags:()=>[t],send:()=>{},now:()=>n});s.begin(t);s.current.phase='active';s.current.startSeq=2;s.current.policy={mode:'live',samples:2,minLinks:2,targetLinks:2,anchorMask:3,rangeProfile:'normal'};
function emit(seq,ids){s.observe(t,{type:'tag_data',server_epoch:s.serverEpoch,slot_seq:2,uwb_active:true,packet_seq:seq,ranges:ids.map(anchor_id=>({anchor_id,ok:true,m:1,scheduled:true}))});}
emit(1,['A','B']);emit(1,['A','B']);assert(!s.current.complete);emit(2,['A','B']);assert(s.current.complete);console.log('PASS duplicate packet cannot fake completion');
