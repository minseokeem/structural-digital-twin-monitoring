'use strict';

// Protocol 3 / R13.1 Unified: one Wi-Fi TAG owns UWB at a time. The server includes an
// information-driven Anchor mask in START. A turn normally uses the smallest
// useful set (minimum 2), while uncertainty or health sweeps may request 3 or 4.
class TDMAScheduler {
  constructor({getTags,send,getPolicy=()=>({mode:'live',samples:1,minLinks:2,targetLinks:2,anchorMask:3,rangeProfile:'normal'}),
    now=Date.now,later=setTimeout,serverEpoch=Date.now(),debug=false,
    liveMaxMs=320,calMaxMs=680,prepareMs=260,guardMs=35,retryMs=40}={}) {
    Object.assign(this,{getTags,send,getPolicy,now,later,serverEpoch,debug,liveMaxMs,calMaxMs,prepareMs,guardMs,retryMs});
    this.seq=0;this.round=[];this.stats=new Map();this.current=null;this.running=false;this.nextAt=0;this.history=[];
  }
  stat(id){if(!this.stats.has(id))this.stats.set(id,{grants:0,acks:0,missed_slots:0,completed_slots:0,early_finishes:0,prepare_failures:0,resyncs:0,last_grant_ms:0,last_ack_ms:0,last_range_ms:0,last_slot_ms:0,last_error:'waiting_slot'});return this.stats.get(id);}
  nextSequence(){this.seq=(this.seq+1)>>>0;return this.seq;}
  eligible(){return this.getTags().filter(t=>t.online&&t.role==='tag'&&t.ip&&Number(t.slot_protocol)===3&&['unified_information_v2','information_window_v1','adaptive_quality_v1'].includes(String(t.ranging_mode||''))&&/^[0-9a-f]{8}$/i.test(t.boot_id||'')&&Number.isFinite(t.device_ms));}
  choose(tags){const live=new Map(tags.map(t=>[t.node_id,t]));this.round=this.round.filter(id=>live.has(id));if(!this.round.length)this.round=[...live.keys()].sort();return live.get(this.round.shift());}
  command(t,c){return {...c,server_epoch:this.serverEpoch,target_boot:parseInt(t.boot_id,16)>>>0};}
  sendStop(t,seq){if(t)this.send(t,this.command(t,{cmd:'uwb_slot',active:false,slot_seq:seq}));}
  stopAll(c,seq){for(const t of this.eligible())this.sendStop(t,seq);c.lastSend=this.now();}
  observe(module,msg){
    const c=this.current;if(!c||c.node_id!==module.node_id||c.boot_id!==module.boot_id||Number(msg.server_epoch)!==this.serverEpoch)return;
    if(c.phase==='stopping'){if(Number(msg.slot_seq)===c.stopSeq&&msg.uwb_active===false)c.stopAck=true;return;}
    if(c.phase!=='active'||Number(msg.slot_seq)!==c.startSeq)return;
    const s=this.stat(c.node_id);if(msg.uwb_active===true&&!c.acked){c.acked=true;s.acks++;s.last_ack_ms=this.now();s.last_error='waiting_ranges';}
    if(msg.type!=='tag_data')return;
    const seq=Number.isInteger(msg.packet_seq)?msg.packet_seq:msg.seq;if(!Number.isInteger(seq))return;
    if(c.dataSeq!==null){const d=(seq-c.dataSeq)>>>0;if(d===0||d>=0x80000000)return;}c.dataSeq=seq>>>0;
    const links=new Set((Array.isArray(msg.ranges)?msg.ranges:[]).filter(r=>r&&r.scheduled!==false&&r.ok!==false&&Number.isFinite(r.m)&&r.m>.03&&r.m<100).map(r=>r.anchor_id).filter(Boolean));
    if(links.size>=c.policy.minLinks){c.goodPackets++;s.last_range_ms=this.now();c.hasRanges=true;}
    if(msg.imu_zeroing!==true)for(const id of links)c.linkSamples.set(id,(c.linkSamples.get(id)||0)+1);
    c.complete=c.goodPackets>=c.policy.samples;
  }
  begin(tag){
    const policy={mode:'live',samples:1,minLinks:2,targetLinks:2,anchorMask:3,rangeProfile:'normal',reason:'default',...this.getPolicy(tag.node_id)};
    const c=this.current={node_id:tag.node_id,boot_id:tag.boot_id,policy,phase:'preparing',started:this.now(),prepareDeadline:this.now()+this.prepareMs,prepareSeq:this.nextSequence(),startSeq:0,stopSeq:0,lastSend:-Infinity,prepareAttempts:0,acked:false,stopAck:false,complete:false,hasRanges:false,goodPackets:0,linkSamples:new Map(),dataSeq:null,deadline:0};
    const s=this.stat(tag.node_id);s.grants++;s.last_grant_ms=this.now();s.last_error='waiting_slot';this.stopAll(c,c.prepareSeq);
    if(this.debug)console.log(`[TDMA] ${tag.node_id} ${policy.rangeProfile} mask=0x${Number(policy.anchorMask).toString(16)} turn`);
  }
  activate(c,t){c.phase='active';c.startSeq=this.nextSequence();c.activeStarted=this.now();c.deadline=this.now()+(c.policy.mode==='calibration'?this.calMaxMs:this.liveMaxMs);c.deviceDeadline=(t.device_ms+Math.max(0,c.deadline-t.last_seen_ms))>>>0;c.lastSend=-Infinity;}
  stopCurrent(c,reason){c.phase='stopping';c.reason=reason;c.stopSeq=this.nextSequence();c.stopStarted=this.now();c.stopAck=false;c.safeAfter=Math.max(this.now(),c.deadline)+this.guardMs;this.stopAll(c,c.stopSeq);}
  finish(c){
    const s=this.stat(c.node_id);s.last_slot_ms=this.now()-c.started;s.last_reason=c.reason;s.last_samples=c.goodPackets;s.last_profile=c.policy.rangeProfile;s.last_anchor_mask=c.policy.anchorMask;
    if(c.complete){s.completed_slots++;s.early_finishes+=c.reason==='samples_ready'?1:0;s.last_error='';}
    else{
      s.missed_slots++;
      if(c.reason==='prepare_timeout'){s.resyncs++;s.last_error='prepare_timeout';}
      else s.last_error=!c.acked?'start_ack_missing':!c.hasRanges?'no_range':'insufficient_samples';
    }
    this.history.push({node_id:c.node_id,mode:c.policy.mode,profile:c.policy.rangeProfile,anchor_mask:c.policy.anchorMask,elapsed_ms:s.last_slot_ms,reason:c.reason});if(this.history.length>30)this.history.shift();this.current=null;this.nextAt=this.now();
  }
  tick(){
    const now=this.now();if(now<this.nextAt)return;const c=this.current;
    if(!c){const tag=this.choose(this.eligible());if(tag)this.begin(tag);return;}
    const tag=this.eligible().find(t=>t.node_id===c.node_id&&t.boot_id===c.boot_id);
    if(c.phase==='preparing'){
      if(!tag||now>=c.prepareDeadline){
        const s=this.stat(c.node_id);s.prepare_failures++;s.last_error='prepare_timeout';
        this.stopCurrent(c,'prepare_timeout');return;
      }
      if(Number(tag.server_epoch)===this.serverEpoch&&Number(tag.slot_seq)===c.prepareSeq&&tag.uwb_active===false&&now-c.started>=this.guardMs){
        this.activate(c,tag);
      } else {
        // the initial STOP is broadcast once to guarantee mutual exclusion.
        // During the prepare handshake, retry only the TAG whose turn is being
        // prepared. Re-broadcasting STOP to every TAG every 25 ms created a burst
        // of ACK traffic and could starve the weakest Wi-Fi TAG.
        if(now-c.lastSend>=this.retryMs){this.sendStop(tag,c.prepareSeq);c.prepareAttempts++;c.lastSend=now;}
        return;
      }
    }
    if(c.phase==='active'){
      if(c.complete){this.stopCurrent(c,'samples_ready');return;}
      if(!tag||now>=c.deadline){this.stopCurrent(c,tag?'deadline':'offline');return;}
      if(!c.acked&&now-c.lastSend>=this.retryMs){
        const lease=Math.floor(c.deadline-now);if(lease>=18)this.send(tag,this.command(tag,{cmd:'uwb_slot',active:true,slot_seq:c.startSeq,lease_ms:lease,device_deadline_ms:c.deviceDeadline,anchor_mask:Number(c.policy.anchorMask)&15,target_links:Number(c.policy.targetLinks)||2,range_profile:Number(c.policy.targetLinks)||2}));c.lastSend=now;
      }
      return;
    }
    if(c.phase==='stopping'){
      if((c.stopAck&&now-c.stopStarted>=this.guardMs)||now>=c.safeAfter){this.finish(c);return;}
      // Initial STOP was broadcast in stopCurrent(). Retry only the current TAG
      // to avoid an ACK storm from all idle TAGs.
      if(now-c.lastSend>=this.retryMs){this.sendStop(tag,c.stopSeq);c.lastSend=now;}
    }
  }
  start(){this.running=true;this.nextAt=this.now()+1800;const run=()=>{if(!this.running)return;this.tick();this.later(run,10);};run();}
  stop(){this.running=false;}
  maxTurnMs(){return this.prepareMs+Math.max(this.liveMaxMs,this.calMaxMs)+this.guardMs+45;}
  snapshot(){
    const tags=this.getTags().filter(t=>t.role==='tag');return {enabled:true,protocol:3,adaptive:true,information_driven:true,server_epoch:this.serverEpoch,active_tag:this.current?.node_id||null,phase:this.current?.phase||'idle',mode:this.current?.policy.mode||null,range_profile:this.current?.policy.rangeProfile||null,anchor_mask:this.current?.policy.anchorMask||0,anchor_ids:this.current?.policy.anchorIds||[],adaptive_reason:this.current?.policy.reason||'',slot_seq:this.current?.startSeq||0,good_packets:this.current?.goodPackets||0,target_samples:this.current?.policy.samples||0,live_max_ms:this.liveMaxMs,calibration_max_ms:this.calMaxMs,worst_round_ms:tags.filter(t=>t.online).length*this.maxTurnMs(),recent_turns:this.history,tags:tags.map(t=>({node_id:t.node_id,...this.stat(t.node_id),status:!t.online?'offline':(Number(t.slot_protocol)!==3||!['unified_information_v2','information_window_v1','adaptive_quality_v1'].includes(String(t.ranging_mode||'')))?'firmware_update_required':this.stat(t.node_id).last_error||'receiving'}))};
  }
}
module.exports={TDMAScheduler};
