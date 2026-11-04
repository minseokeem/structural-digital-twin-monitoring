"use strict";

// Regression test for the R14.4 compact per-anchor transport contract.
// This mirrors the server expansion contract and verifies a worst-case four-link
// datagram remains well below the 1200-byte threshold that previously caused
// tag_data to disappear after counters grew.
function expand(r,index){
  const compactIndex=Number(r.i);
  const anchorIndex=Number.isInteger(compactIndex)&&compactIndex>=1&&compactIndex<=4?compactIndex:index+1;
  const boolAlias=(a,b,f)=>a!==undefined?Boolean(a):b!==undefined?Number(b)!==0:f;
  const numAlias=(a,b,f=null)=>{
  if(a!==undefined&&a!==null&&a!==''&&Number.isFinite(Number(a)))return Number(a);
  if(b!==undefined&&b!==null&&b!==''&&Number.isFinite(Number(b)))return Number(b);
  return f;
};
  return {...r,
    anchor_id:String(r.anchor_id||`ANCHOR-${String(anchorIndex).padStart(2,'0')}`),
    scheduled:boolAlias(r.scheduled,r.s,true), ok:boolAlias(r.ok,r.o,false),
    success_rate:numAlias(r.success_rate,r.q), nlos_score:numAlias(r.nlos_score,r.n,0),
    nlos_suspect:boolAlias(r.nlos_suspect,r.x,false), clk_ppm:numAlias(r.clk_ppm,r.c)};
}
const ranges=[1,2,3,4].map(i=>({i,s:1,o:1,m:19.999999,q:100.0,n:1.000,x:1,c:-999.999}));
const packet={type:'tag_data',node_id:'TAG-03',role:'tag',seq:4294967295,boot_id:'ffffffff',device_ms:4294967295,
 tilt_deg:-123.456,vibration:true,dynamic_g:1.2345,ax_g:-1.2345,ay_g:-1.2345,az_g:-1.2345,imu_failures:4294967295,
 uwb_active:true,slot_seq:4294967295,slot_cmd_rx:4294967295,slot_protocol:3,ranging_mode:'unified_information_v2',
 server_epoch:1844674407370955,packet_seq:4294967295,imu_zeroing:false,range_profile:4,requested_links:4,anchor_mask:15,
 adaptive_event:true,adaptive_reason:'imu_independent_verify',ranges,range_count:4,wifi_rssi:-120,cmd_port:4211,
 fw:'soc-web-dw3000-tag-r14.4-compact-telemetry',battery_v:4.2,imu_ok:true};
const bytes=Buffer.byteLength(JSON.stringify(packet));
if(bytes>=1080)throw new Error(`compact packet too large: ${bytes}`);
const expanded=ranges.map(expand);
if(expanded[3].anchor_id!=='ANCHOR-04'||expanded[3].scheduled!==true||expanded[3].ok!==true||expanded[3].success_rate!==100||expanded[3].nlos_score!==1||expanded[3].nlos_suspect!==true||expanded[3].clk_ppm!==-999.999)throw new Error('compact expansion mismatch');
if(expand({i:1,s:1,o:0,m:null,q:0,n:0,x:0,c:null},0).clk_ppm!==null)throw new Error('null clock offset must remain null');
console.log(`PASS R14.4 compact telemetry: worst-case JSON ${bytes} bytes, aliases expand correctly`);
