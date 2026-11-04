'use strict';
const assert=require('assert/strict');
const {TDMAScheduler}=require('../lib/tdma-scheduler');
let now=100000,queue=[],serial=0,overlap=0;
const ids=['TAG-01','TAG-02','TAG-03'];
const devices=ids.map((id,i)=>({id,boot:(i+1).toString(16).padStart(8,'0'),offset:5000+i*7000,epoch:0,seq:0,seen:false,wasActive:false,until:0,packet:0,grants:[]}));
const modules=new Map();
const later=(f,ms)=>queue.push({t:now+ms,f,k:serial++});
let scheduler;
function telemetry(d,type){
  const active=now<d.until;
  const msg={node_id:d.id,role:'tag',ip:d.id,online:true,slot_protocol:3,boot_id:d.boot,device_ms:(now-d.offset)>>>0,last_seen_ms:now,slot_seq:d.seq,server_epoch:d.epoch,ranging_mode:'unified_information_v2',uwb_active:active,packet_seq:++d.packet,type};
  if(type==='tag_data') msg.ranges=[{anchor_id:'ANCHOR-01',scheduled:true,ok:true,m:.7},{anchor_id:'ANCHOR-02',scheduled:true,ok:true,m:.8}];
  modules.set(d.id,msg);scheduler.observe(msg,msg);
}
function deliver(d,c){
  if(c.target_boot!==parseInt(d.boot,16)||c.server_epoch<d.epoch)return;
  if(c.server_epoch>d.epoch){if(c.active)return;d.epoch=c.server_epoch;d.seen=false;d.until=0;}
  const delta=(c.slot_seq-d.seq)>>>0;if(d.seen&&delta>=0x80000000)return;
  if(d.seen&&delta===0){if(c.active!==d.wasActive)return;telemetry(d,'hello');return;}
  if(c.active){
    const remain=(c.device_deadline_ms-((now-d.offset)>>>0))|0;if(remain<18||remain>2000)return;
    d.until=now+Math.min(remain,c.lease_ms);d.grants.push(now);
    if(devices.some(x=>x!==d&&x.until>now))overlap++;
  } else d.until=0;
  d.seq=c.slot_seq;d.seen=true;d.wasActive=c.active;telemetry(d,'hello');
}
scheduler=new TDMAScheduler({
  getTags:()=>[...modules.values()].map(m=>({...m,online:now-m.last_seen_ms<6000})),
  send:(m,c)=>{const d=devices.find(x=>x.id===m.node_id);const delay=d.id==='TAG-03'?180:5;later(()=>deliver(d,c),delay);},
  now:()=>now,later,serverEpoch:999999,prepareMs:260,guardMs:35,liveMaxMs:320,calMaxMs:680,retryMs:40,
  getPolicy:()=>({mode:'live',samples:1,minLinks:2,targetLinks:2,anchorMask:3,rangeProfile:'normal'})
});
function heartbeat(d){telemetry(d,now<d.until?'tag_data':'hello');later(()=>heartbeat(d),70);}devices.forEach(heartbeat);scheduler.start();
const end=now+30000;while(queue.length){queue.sort((a,b)=>a.t-b.t||a.k-b.k);const e=queue.shift();if(e.t>end)break;now=e.t;e.f();}scheduler.stop();
assert.equal(overlap,0,'TDMA overlap');
for(const d of devices) assert(d.grants.length>=10,`starved ${d.id}: ${d.grants.length}`);
console.log('PASS weak-tag fairness',devices.map(d=>({id:d.id,grants:d.grants.length})),scheduler.snapshot().tags);
