const express = require("express");
const path = require("path");
const OpenAI = require("openai");
const dgram = require("dgram");
const ModelAnchorLimits = require("./public/model-anchor-limits");
const {TDMAScheduler} = require("./lib/tdma-scheduler");
const {planInformationRanging,measurementSigmaM} = require("./lib/information-ranging");
const {updateRobustSlidingWindow} = require("./lib/robust-sliding-window");

// Node 20.12+ loads the optional local .env; shell variables retain precedence.
try { if (process.loadEnvFile) process.loadEnvFile(path.join(__dirname, '.env')); }
catch (err) { if (err.code !== 'ENOENT') throw err; }
const app = express();
const PORT = Number(process.env.PORT || 8080);

app.use(express.json({ limit: "8mb" }));
app.use(express.static(path.join(__dirname, "public")));

app.get("/api/status", (req, res) => {
  res.json({
    ok: true,
    aiEnabled: Boolean(process.env.OPENAI_API_KEY),
    model: process.env.OPENAI_MODEL || "gpt-5.6",
    sensorBridge: true,
    sensorArchitecture: "three_tag_wifi_tdma_unified_information_adaptive_robust_window_four_uwb_only_anchors",
    sensorUdpPort: Number(process.env.SENSOR_UDP_PORT || 4210)
  });
});

// ---------------------------------------------------------------------------
// REAL SENSOR BRIDGE: ESP32-C3 UDP -> Node.js -> Browser SSE
// ---------------------------------------------------------------------------
const SENSOR_UDP_PORT = Number(process.env.SENSOR_UDP_PORT || 4210);
const SENSOR_COMMAND_PORT = Number(process.env.SENSOR_COMMAND_PORT || 4211);
const NODE_OFFLINE_MS = Number(process.env.NODE_OFFLINE_MS || 6000);
// surface-normal sign convention:
// Positive displacement means motion along the stored installation-surface normal.
// Ignore the legacy UWB_Z_SIGN variable so an old bridge-era setting such as -1
// cannot silently invert the generic Surface+Normal solver. Override only with
// UWB_NORMAL_SIGN_V11 when a deliberate installation-wide inversion is required.
const UWB_Z_SIGN = Number(process.env.UWB_NORMAL_SIGN_V11 || 1);
const UWB_Z_SCALE = Number(process.env.UWB_Z_SCALE || 1.0);
// R13 Information-Driven scheduling. Protocol 3 still carries an Anchor mask per TDMA turn, but the server chooses the minimum useful set from expected posterior displacement uncertainty.
const UWB_TDMA = String(process.env.UWB_TDMA_V13 ?? process.env.UWB_TDMA_V12 ?? process.env.UWB_TDMA_V11_5 ?? "1") !== "0";
const UWB_INFO_TARGET_SIGMA_MM=Math.max(2.0,Math.min(12.0,Number(process.env.UWB_INFO_TARGET_SIGMA_MM||4.0)));
const UWB_SLIDING_WINDOW_MS=Math.max(1200,Math.min(6000,Number(process.env.UWB_SLIDING_WINDOW_MS||4000))); // Unified: long quiet memory, automatically shortened during motion
const UWB_INFO_FULL_SWEEP_MS=Math.max(1800,Math.min(8000,Number(process.env.UWB_INFO_FULL_SWEEP_MS||2800)));
const UWB_CAL_MIN_READY_LINKS=Math.max(2,Math.min(4,Number(process.env.UWB_CAL_MIN_READY_LINKS||3)));
function timingSetting(name, fallback, lo, hi) {
  const n=Number(process.env[name]);
  return Number.isFinite(n)&&n>0?Math.max(lo,Math.min(hi,n)):fallback;
}
const UWB_LIVE_MAX_MS=timingSetting('UWB_LIVE_MAX_MS_V12',320,180,900);
const UWB_CAL_SLOT_MAX_MS=timingSetting('UWB_CAL_SLOT_MAX_MS_V12',680,350,1200);
const UWB_PREPARE_MS=timingSetting('UWB_PREPARE_MS_V12',260,120,500);
const UWB_SWITCH_GUARD_MS=timingSetting('UWB_SWITCH_GUARD_MS_V12',35,20,120);
const TAG_MEASUREMENT_STALE_MS = Math.max(2500, Number(process.env.TAG_MEASUREMENT_STALE_MS || (UWB_TDMA?4500:1500)));
const TAG_RANGE_HOLD_MS = Math.max(1200, Number(process.env.TAG_RANGE_HOLD_MS_V11_5 || 2800));
const SENSOR_DEBUG = String(process.env.SENSOR_DEBUG || "0") === "1";
// Browser rendering is intentionally decoupled from raw ESP packet rate.  The
// registry still processes every UDP packet, while SSE snapshots are coalesced
// to at most ~10 Hz so the 3D viewer is not asked to rebuild on every ACK/hello.
const BROWSER_SNAPSHOT_MIN_MS = Math.max(60, Math.min(500, Number(process.env.BROWSER_SNAPSHOT_MIN_MS || 100)));

const udpBridge = dgram.createSocket("udp4");
const moduleRegistry = new Map();   // node_id -> latest module metadata
const sensorRegistry = new Map();   // node_id -> latest normalized sensor state
const rangeBaselines = new Map();   // non-metric fallback: tag node_id -> { anchor_id: range_m }
const uwbCalibration = new Map();   // metric layout: fixed per Tag↔Anchor calibration learned from known model coordinates
const uwbMotionState = new Map();   // adaptive output filter state; never used to adapt calibration bias
const uwbFastMotionState = new Map(); // R11.2: sustained short-window motion evidence while stable mode keeps its long static average
const adaptiveRangingState = new Map(); // R13: per-Tag information-plan / periodic health-sweep state
const robustWindowState = new Map(); // R13: [Delta n, Delta n_dot] robust sliding-window estimator
const sseClients = new Set();
let physicalLayout = { metric:false, tags:[], anchors:[] };
let tdmaScheduler = null;
let calibrationGeneration = 0;
const retiredBoots = new Map();
const pendingTiltZeros = new Map();

function median(values) {
  const a = values.filter(Number.isFinite).sort((x,y)=>x-y);
  if (!a.length) return null;
  const m = Math.floor(a.length/2);
  return a.length % 2 ? a[m] : (a[m-1]+a[m])/2;
}

function moduleSnapshot() {
  const now = Date.now();
  return [...moduleRegistry.values()]
    .map(m => ({ ...m, online: now - m.last_seen_ms <= NODE_OFFLINE_MS }))
    .sort((a,b) => String(a.node_id).localeCompare(String(b.node_id)));
}

function sensorSnapshot() {
  const now = Date.now();
  return [...sensorRegistry.values()]
    .map(v => {
      const online=now-v.last_seen_ms<=NODE_OFFLINE_MS;
      const stale=v.role==='tag' && (!online || now-(v.last_measurement_ms||0)>TAG_MEASUREMENT_STALE_MS);
      const tdma=tdmaScheduler?.snapshot().tags.find(t=>t.node_id===v.node_id);
      return {...v,online,tdma,...(stale?{measurement_valid:false,dz_mm:null,raw_dz_mm:null,dz_method:'stale_data'}:{})};
    })
    .sort((a,b) => String(a.node_id).localeCompare(String(b.node_id)));
}

function snapshotPayload() {
  return {
    type: "sensor_snapshot",
    server_time_ms: Date.now(),
    modules: moduleSnapshot(),
    sensors: sensorSnapshot(),
    layout_metric: Boolean(physicalLayout.metric),
    calibration_generation: calibrationGeneration,
    tdma: tdmaScheduler?.snapshot() || {enabled:false}
  };
}

let lastSnapshotBroadcastMs = 0;
let snapshotBroadcastTimer = null;
function broadcastSnapshot() {
  const wire = `data: ${JSON.stringify(snapshotPayload())}\n\n`;
  lastSnapshotBroadcastMs = Date.now();
  for (const res of [...sseClients]) {
    try { res.write(wire); } catch { sseClients.delete(res); }
  }
}
function scheduleSnapshotBroadcast() {
  if (snapshotBroadcastTimer) return;
  const wait = Math.max(0, BROWSER_SNAPSHOT_MIN_MS - (Date.now() - lastSnapshotBroadcastMs));
  snapshotBroadcastTimer = setTimeout(() => {
    snapshotBroadcastTimer = null;
    broadcastSnapshot();
  }, wait);
}

function sendDeviceCommand(module, command) {
  if (!module?.ip) return;
  const buf = Buffer.from(JSON.stringify(command));
  udpBridge.send(buf, 0, buf.length, module.cmd_port || SENSOR_COMMAND_PORT, module.ip, err => {
    if (err) console.error("ESP command send error:", err.message);
  });
}

function currentRangeMap(ranges) {
  const out = {};
  for (const r of Array.isArray(ranges) ? ranges : []) {
    const id = String(r.anchor_id || "").trim();
    const m = Number(r.m);
    if (r.ok !== false && id && Number.isFinite(m) && m > 0.03 && m < 100) out[id] = m;
  }
  return out;
}

function getLayoutTag(nodeId) {
  return physicalLayout.tags.find(x => x.node_id === nodeId) || null;
}
function getLayoutAnchor(nodeId) {
  return physicalLayout.anchors.find(x => x.node_id === nodeId) || null;
}

function observeAnchorViaTag(anchorId, tagNodeId) {
  const now = Date.now();
  const suffix = String(anchorId).match(/(\d+)$/)?.[1] || "";
  moduleRegistry.set(anchorId, {
    ...(moduleRegistry.get(anchorId) || {}),
    node_id: anchorId,
    role: "anchor",
    ip: "",
    cmd_port: 0,
    fw: "soc-web-dw3000-anchor-v5-uwb-only",
    wifi_rssi: null,
    short_addr: suffix ? `0xA${String(Number(suffix)).padStart(3,"0")}` : "",
    transport: "uwb_via_tag",
    source_tag: tagNodeId,
    imu_ok: true,
    last_seen_ms: now,
    online: true
  });
  sensorRegistry.set(anchorId, {
    ...(sensorRegistry.get(anchorId) || {}),
    node_id: anchorId,
    role: "anchor",
    online: true,
    transport: "uwb_via_tag",
    source_tag: tagNodeId,
    last_seen_ms: now
  });
}

// uses new calibration env names so an old R9 .env containing
// UWB_CAL_SAMPLES=1200 / UWB_CAL_MIN_MS=120000 cannot silently restore the
// two-minute calibration. Small legacy values are still accepted for tests.
const UWB_CAL_SAMPLES = Math.max(80, Number(process.env.UWB_CAL_SAMPLES_V10_1 ||
  (Number(process.env.UWB_CAL_SAMPLES)<=400?process.env.UWB_CAL_SAMPLES:180)));
const UWB_CAL_MIN_MS = Math.max(8000,Number(process.env.UWB_CAL_MIN_MS_V10_1 ||
  (Number(process.env.UWB_CAL_MIN_MS)<=30000?process.env.UWB_CAL_MIN_MS:15000)));
const UWB_CAL_MOTION_PAUSE_MS = Math.max(500,Number(process.env.UWB_CAL_MOTION_PAUSE_MS || 1200));
const UWB_CAL_RECENT_MS = Math.max(500,Number(process.env.UWB_CAL_RECENT_MS || 1200));
const UWB_MAX_SIGMA_MM = Number(process.env.UWB_MAX_SIGMA_MM || 80);
// do not let an old 250 ms setting silently make deformation mode noisy again.
const UWB_UNIFIED_TAU_QUIET_MS = Math.max(700, Number(process.env.UWB_UNIFIED_TAU_QUIET_MS || 1400));
const UWB_UNIFIED_TAU_MOTION_MS = Math.max(180, Number(process.env.UWB_UNIFIED_TAU_MOTION_MS || 420));
const UWB_STATIC_DYNAMIC_G = Math.max(0.005, Number(process.env.UWB_STATIC_DYNAMIC_G || 0.035));
const UWB_HOLD_SOFT_G = Math.max(0.04, Number(process.env.UWB_HOLD_SOFT_G || 0.09));
const UWB_HOLD_HARD_G = Math.max(UWB_HOLD_SOFT_G + 0.02, Number(process.env.UWB_HOLD_HARD_G || 0.16));
const UWB_STATIC_MAX_SIGMA_MM = Math.max(2, Number(process.env.UWB_STATIC_MAX_SIGMA_MM || 45));
const UWB_STATIC_MAX_RESIDUAL_MM = Math.max(4, Number(process.env.UWB_STATIC_MAX_RESIDUAL_MM || 20));
const UWB_COORD_GUARD_MIN_MM = Math.max(4, Number(process.env.UWB_COORD_GUARD_MIN_MM || 6));
// a single continuous estimator. There is no Stable/Precision/Responsive switch.
// Motion score changes process uncertainty and time weighting continuously, so the
// estimator state is never reset just because the user selected another mode.
const uwbFilterMode='unified';
function filterSettings(tagNodeId) {
  return {
    mode:'unified', automatic:true,
    window_ms:UWB_SLIDING_WINDOW_MS,
    target_sigma_mm:UWB_INFO_TARGET_SIGMA_MM,
    quiet_tau_ms:UWB_UNIFIED_TAU_QUIET_MS,
    motion_tau_ms:UWB_UNIFIED_TAU_MOTION_MS,
    static_hold:false,
    description:'Unified Adaptive · information gain + robust sliding window'
  };
}
function estimatorSettings(){
  return {window_ms:UWB_SLIDING_WINDOW_MS,target_sigma_mm:UWB_INFO_TARGET_SIGMA_MM};
}
function trimmedCenter(values) {
  const a=values.filter(Number.isFinite).sort((a,b)=>a-b);
  if (!a.length)return null;
  const trim=a.length>=5?Math.floor(a.length*0.2):0;
  const v=trim?a.slice(trim,-trim):a;
  return v.reduce((sum,x)=>sum+x,0)/v.length;
}
const packetOrder = new Map();
const UWB_MIN_LINKS = Math.max(2, Math.min(4, Number(process.env.UWB_MIN_LINKS || 2)));
const UWB_MULTI_TAG_CAL_SAMPLES = Math.max(30, Number(process.env.UWB_MULTI_TAG_CAL_SAMPLES || 40));
const UWB_MULTI_TAG_CAL_RECENT_MS = Math.max(1800, Number(process.env.UWB_MULTI_TAG_CAL_RECENT_MS || 2600));

function onlineTagCount(){
  if(typeof moduleRegistry==='undefined'||!moduleRegistry?.values)return 1;
  const now=Date.now();
  return [...moduleRegistry.values()].filter(m=>m.role==='tag' && now-m.last_seen_ms<=NODE_OFFLINE_MS).length;
}
function tdmaRecentMs(){
  return (typeof UWB_TDMA!=='undefined' && UWB_TDMA)
    ? Math.max(1,onlineTagCount())*(UWB_PREPARE_MS+Math.max(UWB_LIVE_MAX_MS,UWB_CAL_SLOT_MAX_MS)+UWB_SWITCH_GUARD_MS+60)+500 : 1000;
}
function calibrationRequirements(){
  const multi=Math.max(onlineTagCount(),physicalLayout.tags.length)>1;
  return {
    samples:multi?Math.min(UWB_CAL_SAMPLES,UWB_MULTI_TAG_CAL_SAMPLES):UWB_CAL_SAMPLES,
    min_ms:UWB_CAL_MIN_MS,
    recent_ms:multi?Math.max(UWB_CAL_RECENT_MS,UWB_MULTI_TAG_CAL_RECENT_MS,tdmaRecentMs()):UWB_CAL_RECENT_MS
  };
}

function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }
function mad(values, center = median(values)) {
  if (!Number.isFinite(center)) return null;
  return median(values.filter(Number.isFinite).map(v => Math.abs(v-center)));
}
function dist3(a,b) {
  return Math.hypot(Number(a.x)-Number(b.x), Number(a.y)-Number(b.y), Number(a.z)-Number(b.z));
}
function tagNormal(tag) {
  let nx=Number(tag?.nx),ny=Number(tag?.ny),nz=Number(tag?.nz);
  const mag=Math.hypot(nx,ny,nz);
  if(!(Number.isFinite(mag)&&mag>1e-6))return {x:0,y:1,z:0};
  nx/=mag;ny/=mag;nz/=mag;

  // Canonical sign guard.  GLB triangle winding is not always consistent.  For
  // installation classes whose sign has a clear physical meaning, lock it here as
  // well as in the browser candidate generator so an old/saved layout cannot invert
  // displacement direction unexpectedly.
  const cls=String(tag?.surface_class||tag?.surfaceClass||'').toLowerCase();
  if(cls==='upper' && ny<0){nx=-nx;ny=-ny;nz=-nz;}
  if(cls==='ceiling' && ny>0){nx=-nx;ny=-ny;nz=-nz;}
  return {x:nx,y:ny,z:nz};
}
function tagPointAt(tag, displacementM=0) {
  const n=tagNormal(tag),d=Number(displacementM)||0;
  return {x:Number(tag.x)+n.x*d,y:Number(tag.y)+n.y*d,z:Number(tag.z)+n.z*d};
}
function normalGeometryJacobian(tag, anchor, displacementM=0) {
  const p=tagPointAt(tag,displacementM),n=tagNormal(tag);
  const dx=p.x-Number(anchor.x),dy=p.y-Number(anchor.y),dz=p.z-Number(anchor.z);
  const r=Math.hypot(dx,dy,dz);
  return r>1e-9?(dx*n.x+dy*n.y+dz*n.z)/r:0;
}
function calibrationState(tagNodeId) {
  let state = uwbCalibration.get(tagNodeId);
  if (!state) {
    state = { links:{}, started_ms:Date.now(), requirements:calibrationRequirements() };
    uwbCalibration.set(tagNodeId,state);
  }
  return state;
}
function resetTagTemporalState(tagNodeId) {
  // reconnect recovery must not throw away a valid static pair calibration.
  // Clear only temporal estimators/schedulers so stale velocity/window data cannot
  // cause a jump when a Tag returns after an outage.
  uwbMotionState.delete(tagNodeId);
  uwbFastMotionState.delete(tagNodeId);
  adaptiveRangingState.delete(tagNodeId);
  robustWindowState.delete(tagNodeId);
}
function resetTagUwbState(tagNodeId) {
  uwbCalibration.delete(tagNodeId);
  resetTagTemporalState(tagNodeId);
}
function resetAllUwbState() {
  calibrationGeneration++;
  pendingTiltZeros.clear();
  uwbCalibration.clear();
  uwbMotionState.clear();
  uwbFastMotionState.clear();
  adaptiveRangingState.clear();
  robustWindowState.clear();
  for (const state of sensorRegistry.values()) if (state.role==='tag') {
    state.dz_mm=null;state.raw_dz_mm=null;state.measurement_valid=false;
    state.last_valid_measurement_ms=0;state.measurement_held=false;state.hold_age_ms=0;
    state.dz_method='calibrating_metric_pairs';state.uwb_calibration={ready:false,progress:0};
  }
}

function updatePairCalibration(tagNodeId, ranges, msg={}) {
  const tag=getLayoutTag(tagNodeId), cal=calibrationState(tagNodeId);
  const ids=physicalLayout.anchors.map(a=>a.node_id).filter(Boolean);
  const req=cal.requirements || calibrationRequirements();
  const targetSamples=req.samples, targetMinMs=req.min_ms;
  const now=Date.now(), values=currentRangeMap(ranges);
  const minReady=Math.min(ids.length,Math.max(UWB_MIN_LINKS,UWB_CAL_MIN_READY_LINKS));

  if (!cal.frozen) {
    const moving=Boolean(msg.vibration) || Number(msg.dynamic_g)>0.08 || msg.imu_zeroing===true;
    if (moving) {
      cal.pause_until_ms=Math.max(Number(cal.pause_until_ms||0),now+UWB_CAL_MOTION_PAUSE_MS);
      cal.reason='움직임 감지 · 보정 일시정지 (진행률 유지)';
    }

    if(now>=Number(cal.pause_until_ms||0)){
      for (const id of ids) {
        const anchor=getLayoutAnchor(id);
        const l=cal.links[id] ||= {expected_m:dist3(tag,anchor),samples:[],calibrated:false};
        if(l.calibrated)continue;
        if (!Number.isFinite(values[id])) continue;
        if(!Array.isArray(l.samples))l.samples=[];
        if(l.samples.length && now-l.samples.at(-1).t>tdmaRecentMs()) l.dropouts=(l.dropouts||0)+1;
        if(l.samples.length && now-l.samples.at(-1).t<Math.max(20,Math.ceil(targetMinMs/2000)))continue;
        l.samples.push({v:values[id],t:now});
        while(l.samples.length>targetSamples+8 && l.samples.length>1 && now-l.samples[1].t>=targetMinMs)l.samples.shift();
        const v=l.samples.map(x=>x.v), med=median(v);
        l.noise_m=Math.max(0.003,1.4826*(mad(v,med)||0));
        const half=Math.floor(v.length/2);
        l.drift_m=half ? Math.abs(median(v.slice(0,half))-median(v.slice(half))) : Infinity;
        l.median_m=med;l.center_m=trimmedCenter(v);
        const spanMs=l.samples.length>1?now-l.samples[0].t:0;
        l.progress=Math.min(0.99,Math.min(v.length/targetSamples,spanMs/targetMinMs));
        l.stable=v.length>=targetSamples && spanMs>=targetMinMs && l.noise_m<=0.05 && l.drift_m<=0.015;
        if(l.stable && Number.isFinite(l.center_m)){
          l.bias_m=l.center_m-l.expected_m;l.calibrated=true;l.baseline_samples=l.samples.length;
          l.calibrated_ms=now;l.last_calibration_sample_ms=now;delete l.samples;
        }
      }
    }

    const calibratedIds=ids.filter(id=>cal.links[id]?.calibrated);
    if(calibratedIds.length>=minReady && !cal.ready){
      // R13: a Tag becomes usable once enough independent Tag↔Anchor links are
      // individually calibrated.  A weak fourth Anchor may finish later during
      // periodic health sweeps and no longer blocks the whole system at 0%.
      let info=0;
      for(const id of calibratedIds){
        const l=cal.links[id],a=getLayoutAnchor(id),j=normalGeometryJacobian(tag,a,0);
        info+=j*j/Math.max(0.008,l.noise_m||0.02)**2;
      }
      const sigmaMm=info>1e-12?1000/Math.sqrt(info):Infinity;
      const count=Math.min(...calibratedIds.map(id=>cal.links[id].baseline_samples||targetSamples));
      const baselineVariance=Number.isFinite(sigmaMm)?sigmaMm*sigmaMm/Math.max(1,count):Infinity;
      const remainingVariance=Math.max(0.25,1.5**2-(Number.isFinite(baselineVariance)?baselineVariance:0));
      const wantedMs=Number.isFinite(sigmaMm)?100*(sigmaMm*sigmaMm/remainingVariance):12000;
      cal.stable_window_ms=Math.round(clamp(wantedMs,4000,30000)/1000)*1000;
      cal.baseline_sigma_mm=Number.isFinite(baselineVariance)?Math.sqrt(baselineVariance):null;
      cal.ready=true;cal.ready_ms=now;
    }
    const calibratedCount=ids.filter(id=>cal.links[id]?.calibrated).length;
    if(calibratedCount===ids.length && ids.length){cal.frozen=true;cal.ready=true;cal.reason='전체 링크 보정 완료';}
    else if(cal.ready)cal.reason=`사용 가능 ${calibratedCount}/${ids.length} · 나머지 링크 백그라운드 보정`;
    else if(!moving && now>=Number(cal.pause_until_ms||0))cal.reason=`정지 상태 수집 · ${calibratedCount}/${minReady} 링크 필요`;
    if(ids.some(id=>(cal.links[id]?.noise_m||0)>0.05 && (cal.links[id]?.samples?.length||0)>=Math.min(targetSamples,80)))
      cal.reason='거리 잡음 큼 · 좋은 링크부터 독립 보정 중';
    if(ids.some(id=>(cal.links[id]?.drift_m||0)>0.015 && (cal.links[id]?.samples?.length||0)>=Math.min(targetSamples,80)))
      cal.reason='거리 드리프트 · 좋은 링크부터 독립 보정 중';
  }

  const count=ids.filter(id=>cal.links[id]?.calibrated).length;
  const perLinkProgress=ids.map(id=>{
    const l=cal.links[id];if(l?.calibrated)return 1;
    if(Number.isFinite(l?.progress))return l.progress;
    const n=l?.samples?.length||0;if(!n)return 0;
    const span=n>1?Date.now()-l.samples[0].t:0;return Math.min(0.99,Math.min(n/targetSamples,span/targetMinMs));
  });
  const sorted=[...perLinkProgress].sort((a,b)=>b-a);
  const progress=cal.ready?1:(sorted.length>=minReady?Math.min(...sorted.slice(0,minReady)):0);
  const sampleCounts=ids.map(id=>cal.links[id]?.baseline_samples||cal.links[id]?.samples?.length||0);
  return {id:cal.started_ms,ready:!!cal.ready,full_ready:count===ids.length,calibrated_links:count,
    required_links:minReady,total_links:ids.length,progress,reason:cal.reason,target_samples:targetSamples,
    min_samples:sampleCounts.length?Math.min(...sampleCounts):0,
    link_samples:ids.map((id,i)=>({anchor_id:id,samples:sampleCounts[i],calibrated:!!cal.links[id]?.calibrated,progress:perLinkProgress[i]})),links:cal.links};
}

function linkQualityWeight(rangePacket, link) {
  const rate=rangePacket?.success_rate;
  const rateWeight=rate!=null&&Number.isFinite(Number(rate))?clamp(Number(rate)/100,0.1,1):0.8;
  const sigma=Math.max(0.008,Number(link?.noise_m||0.02),Number(link?.runtime_noise_m||0));
  // DW3000 diagnostics are a confidence input only: never "correct" the measured range
  // from CIR heuristics. Suspected NLOS measurements remain available to the robust
  // solver, but are strongly de-weighted so another Anchor can dominate the estimate.
  const nlos=clamp(Number(rangePacket?.nlos_score||0),0,1);
  const diagWeight=clamp(1-0.80*nlos,0.18,1)*(rangePacket?.nlos_suspect===true?0.55:1);
  // Relative inverse variance. The common factor does not change the position solution.
  return rateWeight*Math.min(1,(0.012/sigma)**2)*diagWeight;
}

function correctedMetricLinks(tagNodeId, ranges, calStatus) {
  const packets = new Map((Array.isArray(ranges)?ranges:[]).map(r=>[String(r?.anchor_id||''),r]));
  const out=[];
  for (const [anchorId,link] of Object.entries(calStatus.links||{})) {
    if (!link?.calibrated) continue;
    const r=packets.get(anchorId);
    const raw=Number(r?.m);
    const anchor=getLayoutAnchor(anchorId);
    if (!anchor || r?.ok===false || !Number.isFinite(raw) || raw<=0.03 || raw>=100) continue;
    const instant=raw-Number(link.bias_m||0);
    if (!Number.isFinite(instant) || instant<=0.03) continue;
    const now=Date.now(),settings=filterSettings(tagNodeId);
    if (!Array.isArray(link.runtime_history) || now-(link.last_sample_ms||0)>tdmaRecentMs()) {
      link.runtime_history=[];link.diff_history=[];link.prev_instant=null;
    }
    if (Number.isFinite(link.prev_instant)) {
      link.diff_history.push(instant-link.prev_instant);
      if(link.diff_history.length>100)link.diff_history.shift();
    }
    link.prev_instant=instant;link.last_sample_ms=now;
    if(link.diff_history.length>=20)link.runtime_noise_m=1.4826*(mad(link.diff_history)||0)/Math.sqrt(2);
    link.runtime_history.push({v:instant,t:now});
    while(link.runtime_history.length>4 && now-link.runtime_history[0].t>settings.window_ms)link.runtime_history.shift();
    link.runtime_history=link.runtime_history.slice(-2400);
    const runtimeValues=link.runtime_history.map(x=>x.v);
    const corrected=median(runtimeValues.slice(-7));
    // keeps an independent ~1.2 s recent estimate even in the long stable mode.
    // It is NOT used for ordinary static output. It is only admitted after several
    // mutually consistent packets prove that a large real displacement is being hidden
    // by the long static averaging window.
    const recentEntries=link.runtime_history.filter(x=>now-x.t<=1200);
    const recentValues=recentEntries.map(x=>x.v);
    const recent_m=recentValues.length>=4?median(recentValues):null;
    const recent_ready=recentValues.length>=6 && now-recentEntries[0].t>=450;
    const warmed=link.runtime_history.length>=4 && now-link.runtime_history[0].t>=300;
    out.push({
      anchorId, anchor, warmed, recent_ready, recent_m,
      window_ready:now-link.runtime_history[0].t>=settings.window_ms-100,
      window_progress:clamp((now-link.runtime_history[0].t)/settings.window_ms,0,1),
      raw_m:raw, instant_m:instant, corrected_m:corrected, expected_m:Number(link.expected_m),
      bias_m:Number(link.bias_m||0), noise_m:Math.max(Number(link.noise_m||0.03),Number(link.runtime_noise_m||0)),
      success_rate:Number.isFinite(Number(r?.success_rate))?Number(r.success_rate):null,
      nlos_score:clamp(Number(r?.nlos_score||0),0,1), nlos_suspect:r?.nlos_suspect===true,
      fp_ratio:Number.isFinite(Number(r?.fp_ratio))?Number(r.fp_ratio):null,
      fp_peak_gap:Number.isFinite(Number(r?.fp_peak_gap))?Number(r.fp_peak_gap):null,
      baseWeight:linkQualityWeight(r,link)
    });
  }
  return out;
}

function solveAlongNormalRobust(tag, links, previousD=0) {
  if (!Array.isArray(links) || links.length < UWB_MIN_LINKS) return null;
  const n=tagNormal(tag);
  let d=Number.isFinite(previousD)?Number(previousD):0;

  // Use the known surface-normal line P=P0+d*n. Each Anchor provides two
  // quadratic roots; choose the branch closest to the previous displacement.
  const seeds=[];
  for(const l of links){
    const rx=Number(tag.x)-Number(l.anchor.x);
    const ry=Number(tag.y)-Number(l.anchor.y);
    const rz=Number(tag.z)-Number(l.anchor.z);
    const b=rx*n.x+ry*n.y+rz*n.z;
    const c=rx*rx+ry*ry+rz*rz-Number(l.corrected_m)*Number(l.corrected_m);
    const disc=b*b-c;
    if(disc<0)continue;
    const q=Math.sqrt(Math.max(0,disc));
    const r1=-b+q,r2=-b-q;
    seeds.push(Math.abs(r1-d)<=Math.abs(r2-d)?r1:r2);
  }
  const seed=median(seeds);
  const seedSpreadM=Number.isFinite(seed)&&seeds.length>=3?1.4826*(mad(seeds,seed)||0):Infinity;
  if(Number.isFinite(seed))d=seed;

  const residualAt=(dd,l)=>{
    const p=tagPointAt(tag,dd);
    const dx=p.x-Number(l.anchor.x),dy=p.y-Number(l.anchor.y),dz=p.z-Number(l.anchor.z);
    const pred=Math.hypot(dx,dy,dz);
    const j=pred>1e-9?(dx*n.x+dy*n.y+dz*n.z)/pred:0;
    return {l,pred,res:Number(l.corrected_m)-pred,j};
  };

  let classified=[];
  for(let iter=0;iter<14;iter++){
    const residuals=links.map(l=>residualAt(d,l));
    const center=median(residuals.map(x=>x.res))||0;
    const md=mad(residuals.map(x=>x.res),center);
    const robustSigma=Math.max(.012,Number.isFinite(md)?1.4826*md:.012);
    const huber=Math.max(.035,2.5*robustSigma);
    let num=0,den=0;
    classified=residuals.map(x=>{
      const ar=Math.abs(x.res-center);
      const hw=ar<=huber?1:huber/Math.max(ar,1e-9);
      const w=x.l.baseWeight*hw;
      num+=w*x.j*x.res;den+=w*x.j*x.j;
      return {...x,huberW:hw,w};
    });
    const prior=Number.isFinite(previousD)?previousD:0;
    const priorWeight=.0025;
    num+=priorWeight*(prior-d);den+=priorWeight;
    if(den<1e-6)return null;
    const step=clamp(num/den,-.50,.50);
    d+=step;
    if(Math.abs(step)<1e-6)break;
  }

  let residuals=links.map(l=>residualAt(d,l));
  let center=median(residuals.map(x=>x.res))||0;
  let md=mad(residuals.map(x=>x.res),center);
  let robustSigma=Math.max(.012,Number.isFinite(md)?1.4826*md:.012);
  let huber=Math.max(.035,2.5*robustSigma);
  classified=residuals.map(x=>{
    const ar=Math.abs(x.res-center);
    const hw=ar<=huber?1:huber/Math.max(ar,1e-9);
    return {...x,huberW:hw,w:x.l.baseWeight*hw};
  });
  let inlierIds=new Set(classified.filter(x=>x.huberW>=.35).map(x=>x.l.anchorId));

  if(inlierIds.size>=UWB_MIN_LINKS && inlierIds.size<links.length){
    const active=links.filter(l=>inlierIds.has(l.anchorId));
    for(let iter=0;iter<10;iter++){
      let num=0,den=0;
      for(const l of active){
        const x=residualAt(d,l),w=l.baseWeight;
        num+=w*x.j*x.res;den+=w*x.j*x.j;
      }
      const prior=Number.isFinite(previousD)?previousD:0,priorWeight=.0025;
      num+=priorWeight*(prior-d);den+=priorWeight;
      if(den<1e-6)break;
      const step=clamp(num/den,-.50,.50);d+=step;
      if(Math.abs(step)<1e-6)break;
    }
  }

  const final=links.map(l=>residualAt(d,l));
  const activeFinal=final.filter(x=>inlierIds.has(x.l.anchorId));
  if(activeFinal.length<UWB_MIN_LINKS)return null;
  let sq=0,ws=0,jInfo=0;
  for(const x of activeFinal){const w=x.l.baseWeight;sq+=w*x.res*x.res;ws+=w;jInfo+=w*x.j*x.j;}
  const rms=ws>0?Math.sqrt(sq/ws):Infinity;
  const rangeSigma=Math.max(rms,.012);
  const uncertainty=jInfo>1e-6?rangeSigma/Math.sqrt(jInfo):Infinity;
  const js=activeFinal.map(x=>x.j).filter(Number.isFinite);
  const jMean=js.length?js.reduce((a,v)=>a+v,0)/js.length:0;
  const jSpread=js.length?Math.sqrt(js.reduce((a,v)=>a+(v-jMean)*(v-jMean),0)/js.length):0;
  const sides=new Set(js.map(v=>Math.sign(v)).filter(v=>v!==0));
  return {
    displacement_m:d,
    raw_mm:d*1000,
    residual_rms_m:rms,
    inlier_residual_rms_m:rms,
    uncertainty_mm:Number.isFinite(uncertainty)?uncertainty*1000:null,
    used_links:activeFinal.length,
    normal_sensitivity:Math.sqrt(activeFinal.reduce((sum,x)=>sum+x.j*x.j,0)/activeFinal.length),
    vertical_sensitivity:Math.sqrt(activeFinal.reduce((sum,x)=>sum+x.j*x.j,0)/activeFinal.length),
    normal_side_mix:sides.size>=2,
    vertical_side_mix:sides.size>=2,
    normal_j_spread:jSpread,
    vertical_j_spread:jSpread,
    anchor_solution_spread_mm:Number.isFinite(seedSpreadM)?seedSpreadM*1000:null,
    anchor_y_spread_mm:Number.isFinite(seedSpreadM)?seedSpreadM*1000:null,
    link_details:final.map(x=>({
      anchor_id:x.l.anchorId,raw_m:x.l.raw_m,corrected_m:x.l.corrected_m,
      bias_m:x.l.bias_m,instant_corrected_m:x.l.instant_m,noise_m:x.l.noise_m,residual_m:x.res,weight:x.l.baseWeight,
      nlos_score:x.l.nlos_score,nlos_suspect:x.l.nlos_suspect,fp_ratio:x.l.fp_ratio,fp_peak_gap:x.l.fp_peak_gap,
      outlier:!inlierIds.has(x.l.anchorId)
    }))
  };
}

function solveAlongNormalDifferential(tag, links, previousD=0) {
  if(!Array.isArray(links)||links.length<UWB_MIN_LINKS)return null;
  const n=tagNormal(tag);
  let d=Number.isFinite(previousD)?Number(previousD):0;
  const rowsAt=dd=>links.map(l=>{
    const p=tagPointAt(tag,dd);
    const dx=p.x-Number(l.anchor.x),dy=p.y-Number(l.anchor.y),dz=p.z-Number(l.anchor.z);
    const pred=Math.hypot(dx,dy,dz);
    return {l,pred,res:Number(l.corrected_m)-pred,j:pred>1e-9?(dx*n.x+dy*n.y+dz*n.z)/pred:0,w:Number(l.baseWeight||1)};
  }).filter(x=>Number.isFinite(x.res)&&Number.isFinite(x.j)&&Number.isFinite(x.w)&&x.w>0);

  for(let iter=0;iter<14;iter++){
    const rows=rowsAt(d);if(rows.length<UWB_MIN_LINKS)return null;
    const wsum=rows.reduce((a,x)=>a+x.w,0);if(wsum<=1e-9)return null;
    const common=rows.reduce((a,x)=>a+x.w*x.res,0)/wsum;
    const jbar=rows.reduce((a,x)=>a+x.w*x.j,0)/wsum;
    const centered=rows.map(x=>x.res-common),md=mad(centered,median(centered));
    const sigma=Math.max(.006,Number.isFinite(md)?1.4826*md:.006),huber=Math.max(.018,2.5*sigma);
    let num=0,den=0;
    for(const x of rows){
      const rr=x.res-common,jj=x.j-jbar,hw=Math.abs(rr)<=huber?1:huber/Math.max(Math.abs(rr),1e-9),w=x.w*hw;
      num+=w*jj*rr;den+=w*jj*jj;
    }
    if(den<2e-5)return null;
    const prior=Number.isFinite(previousD)?previousD:0,priorWeight=.00035;
    num+=priorWeight*(prior-d);den+=priorWeight;
    const step=clamp(num/den,-.30,.30);d+=step;if(Math.abs(step)<1e-6)break;
  }
  const rows=rowsAt(d);if(rows.length<UWB_MIN_LINKS)return null;
  const wsum=rows.reduce((a,x)=>a+x.w,0),common=rows.reduce((a,x)=>a+x.w*x.res,0)/Math.max(wsum,1e-9),jbar=rows.reduce((a,x)=>a+x.w*x.j,0)/Math.max(wsum,1e-9);
  let sq=0,info=0;for(const x of rows){const rr=x.res-common,jj=x.j-jbar;sq+=x.w*rr*rr;info+=x.w*jj*jj;}
  if(info<2e-5)return null;
  const rms=Math.sqrt(sq/Math.max(wsum,1e-9)),sensitivity=Math.sqrt(info/Math.max(wsum,1e-9)),uncertainty=Math.max(rms,.010)/Math.sqrt(info);
  return {displacement_m:d,raw_mm:d*1000,common_offset_m:common,residual_rms_m:rms,differential_sensitivity:sensitivity,uncertainty_mm:Number.isFinite(uncertainty)?uncertainty*1000:null};
}

function fuseDifferentialCommonMode(tag, absoluteSolved, differentialSolved) {
  if(!absoluteSolved||!differentialSolved)return absoluteSolved;
  const commonMm=Math.abs(Number(differentialSolved.common_offset_m||0))*1000;
  const diffSens=Number(differentialSolved.differential_sensitivity||0),diffSigma=Number(differentialSolved.uncertainty_mm||Infinity),diffRms=Number(differentialSolved.residual_rms_m||Infinity)*1000;
  const reliable=diffSens>=.025&&Number.isFinite(diffSigma)&&diffSigma<=55&&Number.isFinite(diffRms)&&diffRms<=28;
  if(!reliable||commonMm<2.5)return {...absoluteSolved,common_mode_offset_mm:Number.isFinite(commonMm)?commonMm:null,differential_sensitivity:diffSens,differential_uncertainty_mm:Number.isFinite(diffSigma)?diffSigma:null,differential_used:false};
  const disagreement=Math.abs(Number(absoluteSolved.raw_mm)-Number(differentialSolved.raw_mm));
  if(!Number.isFinite(disagreement)||disagreement<3)return {...absoluteSolved,common_mode_offset_mm:commonMm,differential_sensitivity:diffSens,differential_uncertainty_mm:diffSigma,differential_used:false};
  const mix=clamp(.25+.65*clamp((commonMm-2.5)/9,0,1)*clamp((diffSens-.025)/.10,0,1),.25,.88);
  const displacement_m=Number(absoluteSolved.displacement_m)*(1-mix)+Number(differentialSolved.displacement_m)*mix;
  return {...absoluteSolved,displacement_m,raw_mm:displacement_m*1000,common_mode_offset_mm:commonMm,differential_sensitivity:diffSens,differential_uncertainty_mm:diffSigma,differential_used:true,differential_mix:mix,differential_raw_mm:differentialSolved.raw_mm};
}

function normalDirectionConsensus(tag,links,previousD=0){
  const estimates=[];
  for(const l of Array.isArray(links)?links:[]){
    const j=normalGeometryJacobian(tag,l.anchor,previousD);
    if(!Number.isFinite(j)||Math.abs(j)<0.08)continue;
    const p=tagPointAt(tag,previousD);
    const pred=Math.hypot(p.x-Number(l.anchor.x),p.y-Number(l.anchor.y),p.z-Number(l.anchor.z));
    const measured=Number(l.corrected_m);
    if(!Number.isFinite(pred)||!Number.isFinite(measured))continue;
    const step=(measured-pred)/j;
    const noise=Math.max(0.008,Number(l.noise_m||0.02));
    const threshold=Math.max(0.003,1.35*noise/Math.max(Math.abs(j),0.08));
    if(!Number.isFinite(step))continue;
    estimates.push({step,j,threshold,weight:Number(l.baseWeight||1),anchorId:l.anchorId});
  }
  const significant=estimates.filter(x=>Math.abs(x.step)>=x.threshold);
  if(significant.length<2)return {reliable:false,vote_count:significant.length,total:estimates.length,step_m:0,ratio:0};
  let pos=0,neg=0;
  for(const x of significant){if(x.step>0)pos+=x.weight;else neg+=x.weight;}
  const sign=pos>=neg?1:-1,totalW=pos+neg,major=Math.max(pos,neg);
  const same=significant.filter(x=>Math.sign(x.step)===sign).map(x=>x.step);
  const step=median(same);
  const ratio=totalW>0?major/totalW:0;
  return {reliable:same.length>=2&&ratio>=0.67,vote_count:same.length,total:significant.length,step_m:Number(step||0),ratio,sign};
}

function applyDirectionConsistencyGuard(tag,solved,links,previousD=0){
  if(!solved||!Number.isFinite(solved.displacement_m))return solved;
  const c=normalDirectionConsensus(tag,links,previousD);
  const solverStep=Number(solved.displacement_m)-Number(previousD||0);
  let guardedD=Number(solved.displacement_m),guarded=false,unstable=false;
  if(c.reliable&&Math.abs(c.step_m)>=0.003){
    if(Math.abs(solverStep)>=0.004&&Math.sign(solverStep)!==Math.sign(c.step_m)){
      guardedD=Number(previousD||0)+clamp(c.step_m,-0.08,0.08);
      guarded=true;
    }else if(Math.abs(solverStep)>Math.max(0.015,Math.abs(c.step_m)*2.8)){
      guardedD=Number(previousD||0)+clamp(0.65*c.step_m+0.35*solverStep,-0.08,0.08);
      guarded=true;
    }
  }
  // If the per-Anchor direction vote is inconclusive, do not freeze a valid robust
  // geometry solution. Low-sensitivity links can have thresholds larger than the
  // real displacement; treating that lack of evidence as a contradiction caused
  // genuine 20~40 mm motion to stick at an intermediate value.
  if(!guarded)return {...solved,direction_consensus_ratio:c.ratio,direction_votes:c.vote_count,direction_unstable:false};
  const residualMm=weightedGeometryResidualMm(tag,links,guardedD);
  return {...solved,displacement_m:guardedD,raw_mm:guardedD*1000,
    residual_rms_m:Number.isFinite(residualMm)?residualMm/1000:solved.residual_rms_m,
    direction_guarded:true,direction_unstable:unstable,direction_consensus_ratio:c.ratio,direction_votes:c.vote_count,
    direction_step_mm:c.step_m*1000};
}

function weightedGeometryResidualMm(tag, links, displacementM) {
  if(!Array.isArray(links)||links.length<UWB_MIN_LINKS)return null;
  const p=tagPointAt(tag,displacementM);let sq=0,weight=0;
  for(const l of links){
    const pred=Math.hypot(p.x-Number(l.anchor.x),p.y-Number(l.anchor.y),p.z-Number(l.anchor.z));
    const res=Number(l.corrected_m)-pred,w=Number(l.baseWeight||1);
    if(!Number.isFinite(pred)||!Number.isFinite(res)||!Number.isFinite(w)||w<=0)continue;
    sq+=w*res*res;weight+=w;
  }
  return weight>0?Math.sqrt(sq/weight)*1000:null;
}

function applyCoordinateConsistencyGuard(tagNodeId, tag, solved, links) {
  const motion=uwbMotionState.get(tagNodeId);
  if(!motion||!Number.isFinite(motion.filtered_mm)||!solved||!Number.isFinite(solved.raw_mm))return solved;
  const prevMm=Number(motion.filtered_mm),jumpMm=Number(solved.raw_mm)-prevMm,absJumpMm=Math.abs(jumpMm);
  const rawGateMm=Math.max(UWB_COORD_GUARD_MIN_MM,1.15*Number(solved.uncertainty_mm||0),450*Number(solved.residual_rms_m||.012));
  const gateMm=clamp(rawGateMm,UWB_COORD_GUARD_MIN_MM,12);
  if(!Number.isFinite(absJumpMm)||absJumpMm<=gateMm)return solved;
  const prevResidualMm=weightedGeometryResidualMm(tag,links,prevMm/1000),newResidualMm=Number.isFinite(Number(solved.residual_rms_m))?Number(solved.residual_rms_m)*1000:null;
  if(!Number.isFinite(prevResidualMm)||!Number.isFinite(newResidualMm))return solved;
  const improvementMm=prevResidualMm-newResidualMm,requiredImprovementMm=Math.max(2.5,prevResidualMm*.18);
  if(improvementMm>=requiredImprovementMm)return solved;
  const blend=clamp(gateMm/Math.max(absJumpMm,1e-6),.18,.72),guardedRawMm=prevMm+jumpMm*blend,guardedD=guardedRawMm/1000,guardedResidualMm=weightedGeometryResidualMm(tag,links,guardedD);
  return {...solved,displacement_m:guardedD,raw_mm:guardedRawMm,residual_rms_m:Number.isFinite(guardedResidualMm)?guardedResidualMm/1000:solved.residual_rms_m,coordinate_guarded:true,coordinate_guard_gate_mm:gateMm,coordinate_guard_prev_residual_mm:prevResidualMm,coordinate_guard_new_residual_mm:newResidualMm,coordinate_guard_guarded_residual_mm:guardedResidualMm};
}

function selectStableLargeMotionSolution(tagNodeId, stableSolved, fastSolved, msg={}) {
  if(!stableSolved || !fastSolved) return stableSolved;
  // Unified mode never switches to a separate stable/fast estimator branch.
  return stableSolved;
  const fast=Number(fastSolved.raw_mm), slow=Number(stableSolved.raw_mm);
  if(!Number.isFinite(fast) || !Number.isFinite(slow)) return stableSolved;

  const fastMotionMap=(typeof uwbFastMotionState!=='undefined')?uwbFastMotionState:(globalThis.__uwbFastMotionState||(globalThis.__uwbFastMotionState=new Map()));
  let st=fastMotionMap.get(tagNodeId);
  if(!st)st={values:[],direction:0,last_ms:0,active_until_ms:0};
  const now=Date.now();
  if(now-st.last_ms>1800)st.values=[];
  st.last_ms=now;

  const sensitivity=Number(fastSolved.normal_sensitivity||0);
  const sigma=Number(fastSolved.uncertainty_mm||Infinity);
  const residual=Number(fastSolved.residual_rms_m||Infinity)*1000;
  const spread=Number(fastSolved.anchor_y_spread_mm??fastSolved.anchor_solution_spread_mm??Infinity);
  const geometryGood=sensitivity>=.12 && Number.isFinite(sigma) && sigma<=38 &&
    Number.isFinite(residual) && residual<=24 && (!Number.isFinite(spread) || spread<=18);
  const gap=fast-slow;
  const gapDirection=Math.sign(gap);

  if(geometryGood && Math.abs(gap)>=10){
    if(st.direction && gapDirection && gapDirection!==st.direction)st.values=[];
    st.direction=gapDirection||st.direction;
    st.values.push(fast);
    if(st.values.length>11)st.values.shift();
  }else if(Math.abs(gap)<7){
    st.values=[];st.direction=0;
  }

  const center=median(st.values);
  const spreadFast=Number.isFinite(center)?1.4826*(mad(st.values,center)||0):Infinity;
  const motionHint=Boolean(msg.vibration)||Number(msg.dynamic_g||0)>=.055||Math.abs(Number(msg.tilt_deg||0))>=.8;
  const needed=motionHint?5:7;
  const proven=st.values.length>=needed && Number.isFinite(center) && spreadFast<=4.5 &&
    Math.abs(center-slow)>=10 && Math.sign(center-slow)===st.direction;
  if(proven)st.active_until_ms=now+2200;

  fastMotionMap.set(tagNodeId,st);
  if(now>Number(st.active_until_ms||0))return stableSolved;

  // Keep all robust diagnostics from the fast geometry solution. The downstream
  // deformation filter still limits output speed and stationary hold behaviour.
  return {...fastSolved,adaptive_stable_motion:true,stable_window_raw_mm:slow,fast_window_raw_mm:fast};
}

function preserveDeformationFilter(tagNodeId, rawMm, quality, dynamicG, vibration, tiltDeg=0) {
  // used only while the robust sliding window is warming up. There is no
  // static hold/dead-band and no mode-dependent reset. A continuous motion score
  // changes smoothing and per-update slew limits without changing estimator state.
  const now=Date.now();
  let s=uwbMotionState.get(tagNodeId);
  if(!s || !Number.isFinite(Number(s.filtered_mm))){
    s={filtered_mm:Number(rawMm)||0,time_ms:now-100,position_history:[]};
  }
  const dt=clamp(now-Number(s.time_ms||now-100),1,300);
  const dg=Math.abs(Number(dynamicG||0));
  const dgScore=clamp((dg-0.012)/0.11,0,1);
  // vibration alone is verification evidence, not displacement evidence.
  const vibScore=vibration?0.12:0;
  const sigma=Number(quality?.uncertainty_mm||0);
  const sigmaScore=Number.isFinite(sigma)?clamp((sigma-4)/30,0,1):0.4;
  const motionScore=clamp(Math.max(vibScore,dgScore,0.35*sigmaScore),0,1);
  s.position_history.push(Number(rawMm));
  while(s.position_history.length>5)s.position_history.shift();
  const target=median(s.position_history);
  const tau=UWB_UNIFIED_TAU_QUIET_MS+(UWB_UNIFIED_TAU_MOTION_MS-UWB_UNIFIED_TAU_QUIET_MS)*motionScore;
  const alpha=1-Math.exp(-dt/Math.max(1,tau));
  const maxStep=vibration?3:(4+9*motionScore);
  const desired=s.filtered_mm+clamp(Number(target)-s.filtered_mm,-maxStep,maxStep);
  s.filtered_mm+=alpha*(desired-s.filtered_mm);
  s.raw_mm=rawMm;s.time_ms=now;s.motion_score=motionScore;s.static_mode='';
  uwbMotionState.set(tagNodeId,s);
  return s.filtered_mm;
}

function estimateDzMm(tagNodeId, ranges, msg={}) {
  const invalid=(method,extra={})=>({dz_mm:null,raw_dz_mm:null,valid:false,method,range_count:0,...extra});
  const nowMap=currentRangeMap(ranges);
  if (!Object.keys(nowMap).length) return invalid('no_range');
  const tag=getLayoutTag(tagNodeId);
  if (!physicalLayout.metric || !tag) return invalid('coordinates_required');
  const cal=updatePairCalibration(tagNodeId,ranges,msg);
  const compact={id:cal.id,ready:cal.ready,full_ready:cal.full_ready,calibrated_links:cal.calibrated_links,
    required_links:cal.required_links,total_links:cal.total_links,progress:cal.progress,reason:cal.reason,
    target_samples:cal.target_samples,min_samples:cal.min_samples,link_samples:cal.link_samples};
  if (!cal.ready) return invalid('calibrating_metric_pairs',{calibration:compact});

  const links=correctedMetricLinks(tagNodeId,ranges,cal);
  if(links.length<UWB_MIN_LINKS)return invalid('insufficient_calibrated_links',{calibration:compact,
    link_details:[],averaging_ready:false,averaging_progress:0});

  let window=robustWindowState.get(tagNodeId);
  if(!window){window={samples:[],d_m:(uwbMotionState.get(tagNodeId)?.filtered_mm||0)/1000,v_mps:0,t:Date.now()};robustWindowState.set(tagNodeId,window);}
  const measurements=links.map(l=>({
    anchorId:l.anchorId,anchor:l.anchor,corrected_m:l.instant_m,
    sigma_m:measurementSigmaM({range:{success_rate:l.success_rate,nlos_score:l.nlos_score,nlos_suspect:l.nlos_suspect,ok:true},link:cal.links?.[l.anchorId]}),
    nlos_score:l.nlos_score
  }));
  const estCfg=estimatorSettings();
  const sw=updateRobustSlidingWindow({state:window,tag,measurements,now:Date.now(),windowMs:estCfg.window_ms,
    imu:{vibration:Boolean(msg.vibration),dynamic_g:Number(msg.dynamic_g||0),tilt_deg:Number(msg.tilt_deg||0)}});
  robustWindowState.set(tagNodeId,sw.state||window);

  // Keep the instantaneous robust geometry solve only as a diagnostic/fallback.
  const prevD=Number.isFinite(Number(sw.state?.d_m))?Number(sw.state.d_m):0;
  const instant=solveAlongNormalRobust(tag,links.map(l=>({...l,corrected_m:l.instant_m})),prevD);
  const factor=UWB_Z_SIGN*UWB_Z_SCALE;
  if(!sw.ready || !Number.isFinite(Number(sw.d_mm))){
    if(!instant)return invalid('window_warmup',{calibration:compact,averaging_ready:false,
      averaging_progress:Math.min(0.99,Math.max(Number(sw.windowSpanMs||0),Number(sw.windowAgeMs||0))/Math.min(estCfg.window_ms,2000)),range_count:links.length});
    const fallback=preserveDeformationFilter(tagNodeId,instant.raw_mm,instant,Number(msg.dynamic_g||0),Boolean(msg.vibration),Number(msg.tilt_deg||0));
    return {dz_mm:fallback*factor,raw_dz_mm:instant.raw_mm*factor,valid:true,method:'legacy_fallback_window_warmup',
      range_count:instant.used_links||links.length,calibration:compact,residual_rms_m:instant.residual_rms_m,
      uncertainty_mm:instant.uncertainty_mm!=null?instant.uncertainty_mm*Math.abs(factor):null,
      window_sigma_mm:null,window_velocity_mm_s:null,window_samples:sw.sampleCount||0,window_span_ms:sw.windowSpanMs||0,
      information_target_sigma_mm:estCfg.target_sigma_mm,link_details:instant.link_details||[],temporal_mode:'unified_window_warmup',
      normal_sensitivity:instant.normal_sensitivity??null,averaging_ready:false,averaging_progress:0,reference_only:true,
      quality_warning:'window_warmup',resolution_limited:false,filter:filterSettings(tagNodeId)};
  }

  const detailMap=new Map((sw.link_details||[]).map(x=>[x.anchor_id,x]));
  const linkDetails=links.map(l=>{
    const x=detailMap.get(l.anchorId)||{};
    return {anchor_id:l.anchorId,raw_m:l.raw_m,corrected_m:l.instant_m,bias_m:l.bias_m,instant_corrected_m:l.instant_m,
      noise_m:l.noise_m,sigma_m:Number(x.sigma_m||measurementSigmaM({range:{success_rate:l.success_rate,nlos_score:l.nlos_score,nlos_suspect:l.nlos_suspect},link:cal.links?.[l.anchorId]})),
      residual_m:Number.isFinite(Number(x.residual_m))?Number(x.residual_m):null,weight:Number.isFinite(Number(x.robust_weight))?Number(x.robust_weight):1,
      robust_weight:Number.isFinite(Number(x.robust_weight))?Number(x.robust_weight):1,age_weight:Number.isFinite(Number(x.age_weight))?Number(x.age_weight):1,
      repetition_weight:Number.isFinite(Number(x.repetition_weight))?Number(x.repetition_weight):1,
      total_weight:Number.isFinite(Number(x.total_weight))?Number(x.total_weight):1,
      information_h:Number.isFinite(Number(x.h))?Number(x.h):normalGeometryJacobian(tag,l.anchor,sw.d_m),
      nlos_score:l.nlos_score,nlos_suspect:l.nlos_suspect,fp_ratio:l.fp_ratio,fp_peak_gap:l.fp_peak_gap,
      hard_rejected:Boolean(x.hard_rejected),quarantined:Boolean(x.quarantined),quarantine_until_ms:Number(x.quarantine_until_ms||0),
      candidate_step_m:Number.isFinite(Number(x.candidate_step_m))?Number(x.candidate_step_m):null,
      innovation_m:Number.isFinite(Number(x.innovation_m))?Number(x.innovation_m):null,
      outlier:Boolean(x.hard_rejected)||Boolean(x.quarantined)||Number(x.robust_weight??1)<0.35};
  });
  const hs=linkDetails.map(x=>Number(x.information_h)).filter(Number.isFinite);
  const sensitivity=hs.length?Math.sqrt(hs.reduce((a,h)=>a+h*h,0)/hs.length):0;
  if(!Number.isFinite(sensitivity)||sensitivity<0.001)return invalid('insufficient_geometry',{calibration:compact,
    range_count:sw.anchorCount||links.length,link_details:linkDetails,normal_sensitivity:sensitivity,
    window_sigma_mm:Number(sw.posterior_sigma_mm)*Math.abs(factor),window_velocity_mm_s:Number(sw.v_mm_s||0)*factor,
    window_samples:Number(sw.sampleCount||0),window_span_ms:Number(sw.windowSpanMs||0),information_target_sigma_mm:estCfg.target_sigma_mm});
  const sigmaMm=Number(sw.posterior_sigma_mm)*Math.abs(factor);
  const residual=Number(sw.residual_rms_m);
  const medianSigma=median(linkDetails.map(x=>Number(x.sigma_m)).filter(Number.isFinite))||0.02;
  const residualOk=Number.isFinite(residual)&&residual<=Math.max(0.040,3.2*medianSigma);
  const valid=Number.isFinite(sigmaMm)&&sigmaMm<=UWB_MAX_SIGMA_MM&&residualOk;
  const weakGeometry=sensitivity<0.08;
  const referenceOnly=sigmaMm>Math.max(2*estCfg.target_sigma_mm,12)||weakGeometry;
  const warning=sw.consensus_rejected?'vibration_verification_hold':sw.state_guard_rejected?'window_state_guard':!residualOk?'window_residual_high':weakGeometry?'weak_vertical_geometry':sigmaMm>estCfg.target_sigma_mm?'uncertainty_above_target':'';
  const averagingProgress=clamp(Math.max(Number(sw.windowSpanMs||0),Number(sw.windowAgeMs||0))/Math.min(estCfg.window_ms,2000),0,1);
  const rawMm=instant?.raw_mm!=null?Number(instant.raw_mm):Number(sw.d_mm);
  return {dz_mm:Number(sw.d_mm)*factor,raw_dz_mm:rawMm*factor,valid,method:'robust_sliding_window_surface_normal',
    range_count:sw.anchorCount||links.length,calibration:compact,residual_rms_m:residual,
    uncertainty_mm:sigmaMm,window_sigma_mm:sigmaMm,window_velocity_mm_s:Number(sw.v_mm_s||0)*factor,
    window_samples:Number(sw.sampleCount||0),window_span_ms:Number(sw.windowSpanMs||0),information_target_sigma_mm:estCfg.target_sigma_mm,
    link_details:linkDetails,temporal_mode:'unified_robust_sliding_window',normal_sensitivity:sensitivity,vertical_sensitivity:sensitivity,
    averaging_ready:averagingProgress>=0.5,averaging_progress:averagingProgress,reference_only:referenceOnly,
    quality_warning:warning,resolution_limited:weakGeometry||(!valid&&Number.isFinite(sigmaMm)&&sigmaMm>UWB_MAX_SIGMA_MM),
    sliding_window_active:true,state_guard_rejected:Boolean(sw.state_guard_rejected),consensus_rejected:Boolean(sw.consensus_rejected),filter:filterSettings(tagNodeId),motion_score:Number(sw.motion_score||0),effective_window_ms:Number(sw.effective_window_ms||estCfg.window_ms),process_sigma_d_mm:Number(sw.process_sigma_d_mm||0),process_sigma_v_mm_s:Number(sw.process_sigma_v_mm_s||0),
    zero_velocity_active:Boolean(sw.zero_velocity_active),quiet_duration_ms:Number(sw.quiet_duration_ms||0),velocity_leak:Number.isFinite(Number(sw.velocity_leak))?Number(sw.velocity_leak):null,
    verification_only:Boolean(sw.verification_only),displacement_consensus_confirmed:Boolean(sw.displacement_consensus_confirmed),displacement_consensus_count:Number(sw.displacement_consensus_count||0),
    displacement_consensus_mm:Number.isFinite(Number(sw.displacement_consensus_mm))?Number(sw.displacement_consensus_mm):null,displacement_consensus_spread_mm:Number.isFinite(Number(sw.displacement_consensus_spread_mm))?Number(sw.displacement_consensus_spread_mm):null,
    residual_baseline_mm:Number.isFinite(Number(sw.residual_baseline_mm))?Number(sw.residual_baseline_mm):null,quiet_residual_limit_mm:Number.isFinite(Number(sw.quiet_residual_limit_mm))?Number(sw.quiet_residual_limit_mm):null,
    inlier_count:Number(sw.inlier_count||0),outlier_count:Number(sw.outlier_count||0),inlier_anchors:Array.isArray(sw.inlier_anchors)?sw.inlier_anchors:[],outlier_anchors:Array.isArray(sw.outlier_anchors)?sw.outlier_anchors:[],
    quarantined_anchors:Array.isArray(sw.quarantined_anchors)?sw.quarantined_anchors:[],reacquired:Boolean(sw.reacquired),reacquire_target_mm:Number.isFinite(Number(sw.reacquire_target_mm))?Number(sw.reacquire_target_mm):null,residual_baseline_frozen:Boolean(sw.residual_baseline_frozen)};
}

function expandCompactTagRanges(msg) {
  if (msg?.type !== 'tag_data' || !Array.isArray(msg.ranges)) return;
  msg.ranges = msg.ranges.map((r, index) => {
    if (!r || typeof r !== 'object') return r;
    const compactIndex = Number(r.i);
    const anchorIndex = Number.isInteger(compactIndex) && compactIndex >= 1 && compactIndex <= 4
      ? compactIndex : index + 1;
    const boolAlias = (longValue, shortValue, fallback) =>
      longValue !== undefined ? Boolean(longValue) :
      shortValue !== undefined ? Number(shortValue) !== 0 : fallback;
    const numericAlias = (longValue, shortValue, fallback = null) => {
      if (longValue !== undefined && longValue !== null && longValue !== '') {
        const a = Number(longValue);
        if (Number.isFinite(a)) return a;
      }
      if (shortValue !== undefined && shortValue !== null && shortValue !== '') {
        const b = Number(shortValue);
        if (Number.isFinite(b)) return b;
      }
      return fallback;
    };
    return {
      ...r,
      anchor_id: String(r.anchor_id || `ANCHOR-${String(anchorIndex).padStart(2,'0')}`),
      scheduled: boolAlias(r.scheduled, r.s, true),
      ok: boolAlias(r.ok, r.o, false),
      success_rate: numericAlias(r.success_rate, r.q),
      nlos_score: numericAlias(r.nlos_score, r.n, 0),
      nlos_suspect: boolAlias(r.nlos_suspect, r.x, false),
      clk_ppm: numericAlias(r.clk_ppm, r.c)
    };
  });
}

function normalizePacket(msg, rinfo) {
  expandCompactTagRanges(msg);
  const nodeId = String(msg.node_id || "").trim();
  const role = msg.role === "anchor" ? "anchor" : msg.role === "tag" ? "tag" : null;
  if (!nodeId || !role) return null;

  if (!['hello','tag_data','anchor_status'].includes(msg.type)) return null;
  const previousModule=moduleRegistry.get(nodeId);
  const wasOffline=Boolean(previousModule && Date.now()-Number(previousModule.last_seen_ms||0)>NODE_OFFLINE_MS);
  const boot=String(msg.boot_id || previousModule?.boot_id || 'legacy');
  const retired=retiredBoots.get(nodeId) || new Set();
  if (retired.has(boot)) return null;
  if (previousModule?.boot_id && previousModule.boot_id!==boot) {
    retired.add(previousModule.boot_id);
    if(retired.size>16)retired.delete(retired.values().next().value);
    retiredBoots.set(nodeId,retired);
    resetTagUwbState(nodeId);
    pendingTiltZeros.delete(nodeId);
    sensorRegistry.delete(nodeId);
    packetOrder.delete(nodeId);
  } else if (wasOffline && role==='tag') {
    // same boot returning after an outage. Preserve pair calibration, but
    // discard stale [Delta n, Delta n_dot] history and information-scheduler state.
    resetTagTemporalState(nodeId);
    const stale=sensorRegistry.get(nodeId);
    if(stale){
      stale.measurement_valid=false;stale.measurement_held=false;stale.dz_mm=null;
      stale.raw_dz_mm=null;stale.uwb_velocity_mm_s=0;stale.uwb_window_samples=0;
      stale.uwb_window_span_ms=0;stale.dz_method='reacquiring_after_reconnect';
    }
  }
  const orderedSeq=Number.isInteger(msg.packet_seq)?msg.packet_seq:
    (msg.type==='tag_data' && Number.isInteger(msg.seq)?msg.seq:null);
  if (orderedSeq!==null) {
    const now=Date.now(),old=packetOrder.get(nodeId),seq=orderedSeq>>>0;
    if(old && old.boot===boot){
      const delta=(seq-old.seq)>>>0;
      if(delta===0 || delta>=0x80000000)return null;
    }
    packetOrder.set(nodeId,{seq,boot,t:now});
  }
  const module = {
    ...(moduleRegistry.get(nodeId) || {}),
    node_id:nodeId,
    role,
    boot_id:boot,
    device_ms:Number.isFinite(msg.device_ms)?msg.device_ms:previousModule?.device_ms,
    server_epoch:Number(msg.server_epoch||0),
    slot_protocol:Number(msg.slot_protocol||previousModule?.slot_protocol||0),
    imu_zeroing:msg.imu_zeroing===true,
    zero_id:Number(msg.zero_id??previousModule?.zero_id??0),
    ranging_mode:String(msg.ranging_mode||previousModule?.ranging_mode||""),
    range_profile:String(msg.range_profile||previousModule?.range_profile||""),
    requested_links:Number.isFinite(Number(msg.requested_links))?Number(msg.requested_links):previousModule?.requested_links,
    anchor_mask:Number.isFinite(Number(msg.anchor_mask))?Number(msg.anchor_mask):previousModule?.anchor_mask,
    ip:rinfo.address,
    cmd_port:Number(msg.cmd_port || SENSOR_COMMAND_PORT),
    fw:String(msg.fw || ""),
    wifi_rssi:Number(msg.wifi_rssi ?? 0),
    short_addr:String(msg.short_addr || ""),
    imu_ok:msg.imu_ok !== false,
    ax_g:Number.isFinite(Number(msg.ax_g)) ? Number(msg.ax_g) : Number(previousModule?.ax_g || 0),
    ay_g:Number.isFinite(Number(msg.ay_g)) ? Number(msg.ay_g) : Number(previousModule?.ay_g || 0),
    az_g:Number.isFinite(Number(msg.az_g)) ? Number(msg.az_g) : Number(previousModule?.az_g || 0),
    imu_failures:Number.isFinite(Number(msg.imu_failures)) ? Number(msg.imu_failures) : Number(previousModule?.imu_failures || 0),
    battery_v:Number.isFinite(Number(msg.battery_v)) ? Number(msg.battery_v) : Number(moduleRegistry.get(nodeId)?.battery_v || 0),
    uwb_active:msg.uwb_active!=null?Boolean(msg.uwb_active):Boolean(moduleRegistry.get(nodeId)?.uwb_active),
    slot_seq:Number.isFinite(Number(msg.slot_seq))?Number(msg.slot_seq):Number(moduleRegistry.get(nodeId)?.slot_seq||0),
    slot_cmd_rx:Number.isFinite(Number(msg.slot_cmd_rx))?Number(msg.slot_cmd_rx):Number(moduleRegistry.get(nodeId)?.slot_cmd_rx||0),
    last_seen_ms:Date.now(),
    online:true
  };
  moduleRegistry.set(nodeId,module);
  if(pendingTiltZeros.get(nodeId)===module.zero_id &&
     module.server_epoch===tdmaScheduler?.serverEpoch)pendingTiltZeros.delete(nodeId);

  if (msg.type === "tag_data") {
    const est = estimateDzMm(nodeId,msg.ranges,{...msg,imu_zeroing:msg.imu_zeroing===true||pendingTiltZeros.has(nodeId)});
    if(est.calibration && pendingTiltZeros.has(nodeId))est.calibration.reason='기울기 영점 명령 응답 대기';
    const packetRanges = Array.isArray(msg.ranges) ? msg.ranges : [];
    const actualRangeCount = packetRanges.filter(r => {
      const m = Number(r?.m);
      return r?.ok !== false && String(r?.anchor_id || "").trim() && Number.isFinite(m) && m > 0.03 && m < 100;
    }).length;
    for (const r of packetRanges) {
      const anchorId = String(r?.anchor_id || "").trim();
      const m = Number(r?.m);
      if (r?.ok !== false && anchorId && Number.isFinite(m) && m > 0.03 && m < 100) {
        observeAnchorViaTag(anchorId,nodeId);
      }
    }
    tdmaScheduler?.observe(module,msg,Object.keys(currentRangeMap(packetRanges)).length);
    const previous=sensorRegistry.get(nodeId)||{};
    const nowMs=Date.now();
    const estValid=est.valid===true && Number.isFinite(Number(est.dz_mm));
    const previousValidMs=Number(previous.last_valid_measurement_ms||0);
    const canHold=!estValid && previous.dz_mm!=null && Number.isFinite(Number(previous.dz_mm)) && previousValidMs>0 &&
      nowMs-previousValidMs<=TAG_RANGE_HOLD_MS &&
      !['calibrating_metric_pairs','coordinates_required'].includes(String(est.method||''));
    const held=Boolean(canHold);
    sensorRegistry.set(nodeId,{
      node_id:nodeId, role:"tag", online:true,
      dz_mm:held?Number(previous.dz_mm):est.dz_mm,
      measurement_valid:estValid||held,
      measurement_held:held,
      hold_age_ms:held?Math.max(0,nowMs-previousValidMs):0,
      reference_only:held?Boolean(previous.reference_only):!!est.reference_only,
      quality_warning:held?'temporary_range_hold':(est.quality_warning||""),
      resolution_limited:held?Boolean(previous.resolution_limited):!!est.resolution_limited,
      normal_sensitivity:est.normal_sensitivity??est.vertical_sensitivity??previous.normal_sensitivity??null,
      vertical_sensitivity:est.normal_sensitivity??est.vertical_sensitivity??previous.vertical_sensitivity??null,
      displacement_axis:'surface_normal',
      surface_normal:(()=>{const t=getLayoutTag(nodeId),n=tagNormal(t);return [n.x,n.y,n.z]})(),
      raw_dz_mm:held?(previous.raw_dz_mm??null):(est.raw_dz_mm??null),
      dz_method:held?'temporary_range_hold':est.method,
      uwb_calibration:est.calibration || previous.uwb_calibration || {ready:false,progress:0},
      uwb_residual_rms_m:est.residual_rms_m!=null && Number.isFinite(Number(est.residual_rms_m)) ? Number(est.residual_rms_m) : (held?previous.uwb_residual_rms_m:null),
      uwb_uncertainty_mm:est.uncertainty_mm!=null && Number.isFinite(Number(est.uncertainty_mm)) ? Number(est.uncertainty_mm) : (held?previous.uwb_uncertainty_mm:null),
      uwb_window_sigma_mm:est.window_sigma_mm!=null && Number.isFinite(Number(est.window_sigma_mm)) ? Number(est.window_sigma_mm) : (held?previous.uwb_window_sigma_mm:null),
      uwb_velocity_mm_s:est.window_velocity_mm_s!=null && Number.isFinite(Number(est.window_velocity_mm_s)) ? Number(est.window_velocity_mm_s) : (held?previous.uwb_velocity_mm_s:null),
      uwb_window_samples:Number.isFinite(Number(est.window_samples))?Number(est.window_samples):Number(previous.uwb_window_samples||0),
      uwb_window_span_ms:Number.isFinite(Number(est.window_span_ms))?Number(est.window_span_ms):Number(previous.uwb_window_span_ms||0),
      uwb_target_sigma_mm:Number.isFinite(Number(est.information_target_sigma_mm))?Number(est.information_target_sigma_mm):UWB_INFO_TARGET_SIGMA_MM,
      uwb_motion_score:Number.isFinite(Number(est.motion_score))?Number(est.motion_score):Number(previous.uwb_motion_score||0),
      uwb_effective_window_ms:Number.isFinite(Number(est.effective_window_ms))?Number(est.effective_window_ms):Number(previous.uwb_effective_window_ms||UWB_SLIDING_WINDOW_MS),
      uwb_process_sigma_d_mm:Number.isFinite(Number(est.process_sigma_d_mm))?Number(est.process_sigma_d_mm):Number(previous.uwb_process_sigma_d_mm||0),
      uwb_process_sigma_v_mm_s:Number.isFinite(Number(est.process_sigma_v_mm_s))?Number(est.process_sigma_v_mm_s):Number(previous.uwb_process_sigma_v_mm_s||0),
      uwb_zero_velocity_active:est.zero_velocity_active!=null?Boolean(est.zero_velocity_active):Boolean(previous.uwb_zero_velocity_active),
      uwb_quiet_duration_ms:Number.isFinite(Number(est.quiet_duration_ms))?Number(est.quiet_duration_ms):Number(previous.uwb_quiet_duration_ms||0),
      uwb_velocity_leak:Number.isFinite(Number(est.velocity_leak))?Number(est.velocity_leak):(previous.uwb_velocity_leak??null),
      uwb_verification_only:est.verification_only!=null?Boolean(est.verification_only):Boolean(previous.uwb_verification_only),
      uwb_displacement_consensus_confirmed:est.displacement_consensus_confirmed!=null?Boolean(est.displacement_consensus_confirmed):Boolean(previous.uwb_displacement_consensus_confirmed),
      uwb_displacement_consensus_count:Number.isFinite(Number(est.displacement_consensus_count))?Number(est.displacement_consensus_count):Number(previous.uwb_displacement_consensus_count||0),
      uwb_displacement_consensus_mm:Number.isFinite(Number(est.displacement_consensus_mm))?Number(est.displacement_consensus_mm):null,
      uwb_displacement_consensus_spread_mm:Number.isFinite(Number(est.displacement_consensus_spread_mm))?Number(est.displacement_consensus_spread_mm):null,
      uwb_residual_baseline_mm:Number.isFinite(Number(est.residual_baseline_mm))?Number(est.residual_baseline_mm):null,
      uwb_quiet_residual_limit_mm:Number.isFinite(Number(est.quiet_residual_limit_mm))?Number(est.quiet_residual_limit_mm):null,
      uwb_consensus_rejected:est.consensus_rejected!=null?Boolean(est.consensus_rejected):Boolean(previous.uwb_consensus_rejected),
      uwb_inlier_count:Number.isFinite(Number(est.inlier_count))?Number(est.inlier_count):Number(previous.uwb_inlier_count||0),
      uwb_outlier_count:Number.isFinite(Number(est.outlier_count))?Number(est.outlier_count):Number(previous.uwb_outlier_count||0),
      uwb_inlier_anchors:Array.isArray(est.inlier_anchors)?est.inlier_anchors:[],
      uwb_outlier_anchors:Array.isArray(est.outlier_anchors)?est.outlier_anchors:[],
      uwb_quarantined_anchors:Array.isArray(est.quarantined_anchors)?est.quarantined_anchors:[],
      uwb_reacquired:est.reacquired!=null?Boolean(est.reacquired):false,
      uwb_reacquire_target_mm:Number.isFinite(Number(est.reacquire_target_mm))?Number(est.reacquire_target_mm):null,
      uwb_residual_baseline_frozen:est.residual_baseline_frozen!=null?Boolean(est.residual_baseline_frozen):Boolean(previous.uwb_residual_baseline_frozen),
      uwb_link_details:Array.isArray(est.link_details)?est.link_details:[],
      uwb_temporal_mode:String(est.temporal_mode||previous.uwb_temporal_mode||""),
      uwb_filter:filterSettings(nodeId),
      averaging_ready:!!est.averaging_ready,
      averaging_progress:est.averaging_progress||0,
      tilt_deg:Number.isFinite(Number(msg.tilt_deg))?Number(msg.tilt_deg):Number(previous.tilt_deg||0),
      vibration:msg.vibration!=null?Boolean(msg.vibration):Boolean(previous.vibration),
      accel_g:Number.isFinite(Number(msg.dynamic_g))?Number(msg.dynamic_g):Number(previous.accel_g||0),
      ax_g:Number.isFinite(Number(msg.ax_g))?Number(msg.ax_g):Number(previous.ax_g||0),
      ay_g:Number.isFinite(Number(msg.ay_g))?Number(msg.ay_g):Number(previous.ay_g||0),
      az_g:Number.isFinite(Number(msg.az_g))?Number(msg.az_g):Number(previous.az_g||0),
      imu_failures:Number.isFinite(Number(msg.imu_failures))?Number(msg.imu_failures):Number(previous.imu_failures||0),
      range_count:actualRangeCount,
      geometry_range_count:est.range_count,
      ranging_mode:String(msg.ranging_mode||module.ranging_mode||''),
      range_profile:Number(msg.range_profile||0),
      requested_links:Number(msg.requested_links||msg.range_profile||0),
      anchor_mask:Number(msg.anchor_mask||0),
      adaptive_reason:String(msg.adaptive_reason||''),
      adaptive_event:Boolean(msg.adaptive_event),
      ranges:packetRanges.map(r=>{
        const anchorId=String(r.anchor_id||"");
        const detail=(est.link_details||[]).find(x=>x.anchor_id===anchorId);
        const calLink=uwbCalibration.get(nodeId)?.links?.[anchorId];
        const scheduled=r.scheduled !== false;
        const prevRange=Array.isArray(previous.ranges)?previous.ranges.find(x=>x.anchor_id===anchorId):null;
        // An unscheduled Anchor has no new CIR observation. Preserve its last real
        // diagnostic result until the next periodic/full verification instead of
        // replacing a known NLOS link with a fake zero-risk value.
        const diagSource=scheduled?r:(prevRange||r);
        return {
          anchor_id:anchorId,
          scheduled,
          ok:r.ok !== false,
          m:r.ok === false ? null : Number(r.m),
          corrected_m:detail ? Number(detail.corrected_m) : (calLink?.calibrated && r.ok!==false ? Number(r.m)-Number(calLink.bias_m||0) : null),
          instant_corrected_m:calLink?.calibrated && r.ok!==false && r.m!=null ? Number(r.m)-Number(calLink.bias_m||0):null,
          noise_m:detail?.noise_m??calLink?.noise_m??null,
          sigma_m:detail?.sigma_m??null,
          residual_m:detail?.residual_m??null,
          information_h:detail?.information_h??null,
          information_gain:(detail?.sigma_m!=null && Number(detail.sigma_m)>0 && detail?.information_h!=null)?(Number(detail.information_h)*Number(detail.information_h))/(Number(detail.sigma_m)*Number(detail.sigma_m)):null,
          robust_weight:detail?.robust_weight??null,
          age_weight:detail?.age_weight??null,
          repetition_weight:detail?.repetition_weight??null,
          total_weight:detail?.total_weight??null,
          weight:detail?.weight??null,
          bias_m:calLink?.calibrated ? Number(calLink.bias_m||0) : null,
          hard_rejected:Boolean(detail?.hard_rejected),
          quarantined:Boolean(detail?.quarantined),
          quarantine_until_ms:Number(detail?.quarantine_until_ms||0),
          candidate_step_m:Number.isFinite(Number(detail?.candidate_step_m))?Number(detail.candidate_step_m):null,
          innovation_m:Number.isFinite(Number(detail?.innovation_m))?Number(detail.innovation_m):null,
          outlier:Boolean(detail?.outlier),
          calibrated:Boolean(calLink?.calibrated),
          success_rate:Number(r.success_rate),
          recent_successes:Number(r.recent_successes),
          recent_samples:Number(r.recent_samples),
          attempt_success_rate:Number(r.attempt_success_rate),
          attempts:Number(r.attempts),
          successes:Number(r.successes),
          fp_amp:Number.isFinite(Number(diagSource?.fp_amp))?Number(diagSource.fp_amp):null,
          peak_amp:Number.isFinite(Number(diagSource?.peak_amp))?Number(diagSource.peak_amp):null,
          fp_peak_gap:Number.isFinite(Number(diagSource?.fp_peak_gap))?Number(diagSource.fp_peak_gap):null,
          fp_ratio:Number.isFinite(Number(diagSource?.fp_ratio))?Number(diagSource.fp_ratio):null,
          nlos_score:Number.isFinite(Number(diagSource?.nlos_score))?Number(diagSource.nlos_score):0,
          nlos_suspect:Boolean(diagSource?.nlos_suspect),
          clock_offset_ppm:r.clk_ppm!=null && Number.isFinite(Number(r.clk_ppm))?Number(r.clk_ppm):null,
          diag_fresh:scheduled
        };
      }),
      uwb_active:msg.uwb_active !== false,
      imu_zeroing:msg.imu_zeroing===true,
      slot_seq:Number(msg.slot_seq||previous.slot_seq||0),
      slot_cmd_rx:Number(msg.slot_cmd_rx||previous.slot_cmd_rx||0),
      wifi_rssi:Number(msg.wifi_rssi ?? 0),
      battery_v:Number(msg.battery_v ?? 0),
      imu_ok:msg.imu_ok !== false,
      fw:String(msg.fw || ""),
      last_valid_measurement_ms:estValid?nowMs:previousValidMs,
      last_measurement_ms:nowMs,
      seq:Number(msg.seq || 0),
      boot_id:String(msg.boot_id||"legacy"),
      device_ms:msg.device_ms??null,
      last_seen_ms:nowMs
    });
  } else if (msg.type === "anchor_status" || msg.type === "hello") {
    tdmaScheduler?.observe(module,msg);
    const old=sensorRegistry.get(nodeId)||{};
    sensorRegistry.set(nodeId,{
      ...old,
      node_id:nodeId, role, online:true,
      boot_id:boot,device_ms:module.device_ms,slot_seq:module.slot_seq,
      slot_cmd_rx:module.slot_cmd_rx,imu_zeroing:module.imu_zeroing,
      wifi_rssi:Number(msg.wifi_rssi ?? old.wifi_rssi ?? 0),
      battery_v:Number.isFinite(Number(msg.battery_v))?Number(msg.battery_v):Number(old.battery_v||0),
      tilt_deg:Number.isFinite(Number(msg.tilt_deg))?Number(msg.tilt_deg):Number(old.tilt_deg||0),
      vibration:msg.vibration!=null?Boolean(msg.vibration):Boolean(old.vibration),
      accel_g:Number.isFinite(Number(msg.dynamic_g))?Number(msg.dynamic_g):Number(old.accel_g||0),
      ax_g:Number.isFinite(Number(msg.ax_g))?Number(msg.ax_g):Number(old.ax_g||0),
      ay_g:Number.isFinite(Number(msg.ay_g))?Number(msg.ay_g):Number(old.ay_g||0),
      az_g:Number.isFinite(Number(msg.az_g))?Number(msg.az_g):Number(old.az_g||0),
      imu_failures:Number.isFinite(Number(msg.imu_failures))?Number(msg.imu_failures):Number(old.imu_failures||0),
      uwb_active:msg.uwb_active!=null?Boolean(msg.uwb_active):(old.uwb_active!==false),
      imu_ok:msg.imu_ok !== false,
      fw:String(msg.fw || old.fw || ""),
      last_seen_ms:Date.now()
    });
  }
  return module;
}

udpBridge.on("message", (buffer, rinfo) => {
  try {
    const msg = JSON.parse(buffer.toString("utf8"));
    if (SENSOR_DEBUG) {
      console.log("[UDP RX]", rinfo.address, msg.type, msg.node_id || "", msg.seq ?? "");
    }
    const module = normalizePacket(msg,rinfo);
    if (!module) return;
    scheduleSnapshotBroadcast();
  } catch (err) {
    console.error("Bad ESP UDP packet:", err.message);
  }
});

udpBridge.on("error", err => console.error("Sensor UDP bridge error:",err));
udpBridge.bind(SENSOR_UDP_PORT,"0.0.0.0",() => {
  console.log(`Sensor UDP bridge listening on 0.0.0.0:${SENSOR_UDP_PORT}`);
});

app.get("/api/modules", (req,res) => res.json({modules:moduleSnapshot()}));
app.get("/api/sensors/state", (req,res) => res.json(snapshotPayload()));
app.get("/api/sensors/tdma", (req,res) => res.json(tdmaScheduler?.snapshot() || {enabled:false}));

app.get("/api/sensors/stream", (req,res) => {
  res.setHeader("Content-Type","text/event-stream");
  res.setHeader("Cache-Control","no-cache");
  res.setHeader("Connection","keep-alive");
  res.flushHeaders?.();
  sseClients.add(res);
  res.write(`data: ${JSON.stringify(snapshotPayload())}\n\n`);
  const keep = setInterval(()=>{ try{res.write(": keepalive\n\n");}catch{} },15000);
  req.on("close",()=>{ clearInterval(keep); sseClients.delete(res); });
});

app.get('/api/sensors/filter',(req,res)=>res.json({ok:true,...filterSettings()}));
app.post('/api/sensors/filter',(req,res)=>{
  // Backward-compatible endpoint: R13.1 has one mode only. Old clients cannot
  // reset estimator state by posting Stable/Precision/Responsive anymore.
  res.json({ok:true,...filterSettings(),message:'R13.1 uses Unified Adaptive mode only; manual mode switching is disabled.'});
});

app.get('/api/sensors/layout',(req,res)=>res.json(physicalLayout));
app.post("/api/sensors/layout", (req,res) => {
  const b=req.body||{};
  const all=[...(Array.isArray(b.tags)?b.tags:[]),...(Array.isArray(b.anchors)?b.anchors:[])];
  if (!Array.isArray(b.tags)||!Array.isArray(b.anchors)||all.length>16||all.some(p=>
    typeof p.node_id!=='string'||!p.node_id||!['x','y','z'].every(k=>typeof p[k]==='number'&&Number.isFinite(p[k])&&Math.abs(p[k])<=1000))||
    new Set(all.map(p=>p.node_id)).size!==all.length)
    return res.status(400).json({ok:false,message:'ID 중복 또는 좌표 오류: x/y/z는 m 단위 숫자입니다.'});
  const tags=b.tags.map(t=>{
    const n=tagNormal(t);
    return {...t,nx:n.x,ny:n.y,nz:n.z,normal_sign_locked:true};
  });
  const next={metric:b.metric===true,tags,anchors:b.anchors};
  const changed=JSON.stringify(next)!==JSON.stringify(physicalLayout);
  if (changed) { physicalLayout=next; rangeBaselines.clear();resetAllUwbState(); }
  res.json({ok:true,changed,...physicalLayout});
});

function requestTiltZero(module, generation) {
  const start=Date.now();
  if(module.slot_protocol===3)pendingTiltZeros.set(module.node_id,generation);
  const send=()=>{
    if(generation!==calibrationGeneration)return;
    const fresh=moduleRegistry.get(module.node_id);
    if(!fresh || fresh.boot_id!==module.boot_id)return;
    if(fresh.zero_id===generation && fresh.server_epoch===tdmaScheduler?.serverEpoch)return;
    sendDeviceCommand(fresh,{cmd:"zero_imu",zero_id:generation,
      server_epoch:tdmaScheduler?.serverEpoch||0,target_boot:parseInt(fresh.boot_id,16)>>>0});
    if(Date.now()-start<6000)setTimeout(send,250);
  };
  send();
}

app.post("/api/sensors/zero", (req,res) => {
  rangeBaselines.clear();
  pendingTiltZeros.clear();
  resetAllUwbState();
  // For metric GLB layouts the next UWB_CAL_SAMPLES valid packets are used only to learn
  // fixed Tag↔Anchor pair biases from the known model coordinates. Keep the structure still
  // during this short calibration window; after that the bias is frozen until the next Zero.
  for (const [nodeId,state] of sensorRegistry.entries()) {
    if (state.role !== "tag") continue;
    const m=currentRangeMap(state.ranges);
    if (!physicalLayout.metric && Object.keys(m).length) rangeBaselines.set(nodeId,m);
  }
  for (const module of moduleRegistry.values()) {
    if (module.role === "tag" && Date.now()-module.last_seen_ms<=NODE_OFFLINE_MS) {
      requestTiltZero(module,calibrationGeneration);
    }
  }
  broadcastSnapshot();
  res.json({
    ok:true,
    calibration_generation:calibrationGeneration,
    calibration_samples:calibrationRequirements().samples,
    message:physicalLayout.metric
      ? `UWB pair calibration restarted (${calibrationRequirements().samples} valid samples/link) and Tag tilt zero command applied.`
      : "UWB range baseline and Tag tilt zero command applied."
  });
});

setInterval(()=>{
  let changed=false;
  const now=Date.now();
  for (const m of moduleRegistry.values()) {
    const online=now-m.last_seen_ms<=NODE_OFFLINE_MS;
    if (m.online!==online) {m.online=online;changed=true;}
  }
  broadcastSnapshot(); // Age/stale/calibration diagnostics must advance even without UDP.
},1000);

if (UWB_TDMA) {
  tdmaScheduler = new TDMAScheduler({getTags:moduleSnapshot,send:sendDeviceCommand,
    getPolicy:nodeId=>{
      const tag=getLayoutTag(nodeId);
      const cal=uwbCalibration.get(nodeId)||{};
      const calibrating=Boolean(physicalLayout.metric && tag && !cal.ready);
      let adaptive=adaptiveRangingState.get(nodeId);
      if(!adaptive){adaptive={lastFullSweepMs:0};adaptiveRangingState.set(nodeId,adaptive);}
      const sensor=sensorRegistry.get(nodeId)||{};
      const infoCfg=estimatorSettings();
      const plan=planInformationRanging({
        tag:tag||{x:0,y:0,z:0,nx:0,ny:1,nz:0},
        anchors:physicalLayout.anchors,
        sensor,
        calibration:cal,
        state:adaptive,
        now:Date.now(),
        calibrating,
        fullSweepMs:UWB_INFO_FULL_SWEEP_MS,
        targetSigmaMm:infoCfg.target_sigma_mm,
        minLinks:UWB_MIN_LINKS,
        maxLinks:4
      });
      if(sensor && sensor.role==='tag'){
        sensor.uwb_projected_sigma_mm=plan.projectedSigmaMm??null;
        sensor.uwb_target_sigma_mm=plan.targetSigmaMm??infoCfg.target_sigma_mm;
        sensor.uwb_information_anchor_ids=plan.anchorIds||[];
        sensor.uwb_information_scores=plan.scores||[];
        sensor.uwb_information_reason=plan.reason||'';
      }
      return {
        mode:calibrating?'calibration':'live',
        samples:calibrating?4:1,
        minLinks:UWB_MIN_LINKS,
        targetLinks:plan.targetLinks,
        anchorMask:plan.anchorMask,
        anchorIds:plan.anchorIds,
        rangeProfile:plan.profile,
        reason:plan.reason,
        scores:plan.scores,
        projectedSigmaMm:plan.projectedSigmaMm,
        targetSigmaMm:plan.targetSigmaMm
      };
    },liveMaxMs:UWB_LIVE_MAX_MS,calMaxMs:UWB_CAL_SLOT_MAX_MS,
    prepareMs:UWB_PREPARE_MS,guardMs:UWB_SWITCH_GUARD_MS,retryMs:40,debug:SENSOR_DEBUG});
  tdmaScheduler.start();
}

function nearest(candidates, targetU, preferredZone) {
  const pool = preferredZone
    ? candidates.filter(c => c.zone === preferredZone)
    : candidates;
  const src = pool.length ? pool : candidates;
  return [...src].sort((a, b) => {
    const da = Math.abs((a.u ?? 0.5) - targetU);
    const db = Math.abs((b.u ?? 0.5) - targetU);
    return da - db;
  })[0];
}

function uniquePick(candidates, plan) {
  const used = new Set();
  const out = [];
  for (const p of plan) {
    const ordered = [...candidates].sort((a, b) => {
      const za = p.zone && a.zone === p.zone ? -0.3 : 0;
      const zb = p.zone && b.zone === p.zone ? -0.3 : 0;
      return (Math.abs((a.u ?? .5) - p.u) + za) - (Math.abs((b.u ?? .5) - p.u) + zb);
    });
    const found = ordered.find(c => !used.has(c.id));
    if (found) {
      used.add(found.id);
      out.push(found.id);
    }
  }
  return out;
}

function isBridgeProfile(profile={}) {
  const category=String(profile?.geometryHypothesis?.category||"").toLowerCase();
  const type=String(profile?.geometryHypothesis?.type||"").toLowerCase();
  return category.includes("bridge") || /bridge|viaduct|girder|교량|고가|거더/.test(`${category} ${type}`);
}

function tagBridgePriority(t={}) {
  const role=String(t.structuralRole||t.structural_role||"").toLowerCase();
  if(role==="primary_superstructure")return Number.isFinite(Number(t.bridgePriorityScore))?Number(t.bridgePriorityScore):3.0;
  if(role==="secondary_superstructure")return Number.isFinite(Number(t.bridgePriorityScore))?Number(t.bridgePriorityScore):1.0;
  if(role.includes("support"))return Number.isFinite(Number(t.bridgePriorityScore))?Number(t.bridgePriorityScore):-2.0;
  const name=String(t.sourceMeshName||t.source_mesh_name||"").toLowerCase();
  const cls=String(t.surfaceClass||t.surface||"").toLowerCase();
  if(/pier|column|pedestal|abutment|support|bearing|foot|foundation|cap/.test(name))return -3;
  if(/deck|slab|girder|beam|span|roadway|superstructure/.test(name)&&cls==="upper")return 3;
  if(cls==="upper")return .6;
  return 0;
}

function selectBridgeTagIds(tags,tagCount) {
  if(!Array.isArray(tags)||!tags.length)return [];
  const enriched=tags.map(t=>({...t,__bridgeScore:tagBridgePriority(t)}));
  const primary=enriched.filter(t=>t.__bridgeScore>=1.8);
  const nonSupport=enriched.filter(t=>t.__bridgeScore>-.8);
  const pool=primary.length>=tagCount?primary:(nonSupport.length>=tagCount?nonSupport:enriched);
  const used=new Set(),selected=[];
  const targets=Array.from({length:tagCount},(_,i)=>(i+1)/(tagCount+1));
  const span=Math.max(.001,Math.max(...pool.map(t=>Number(t.u??0)))-Math.min(...pool.map(t=>Number(t.u??0))));
  for(const target of targets){
    let best=null,bestScore=-Infinity;
    for(const t of pool){
      if(used.has(t.id))continue;
      const u=Number(t.u??.5);
      const access=Math.min(1,Math.max(0,Number(t.accessibility_m||0))/Math.max(span,.001));
      const targetFit=1-Math.min(1,Math.abs(u-target)/.34);
      const patchBonus=selected.some(q=>q.surfacePatch===t.surfacePatch)?0:.16;
      const score=1.55*t.__bridgeScore+1.35*targetFit+.18*access+patchBonus;
      if(score>bestScore){bestScore=score;best=t;}
    }
    if(!best)continue;
    used.add(best.id);selected.push(best);
  }
  while(selected.length<tagCount){
    let best=null,bestScore=-Infinity;
    for(const t of pool){
      if(used.has(t.id))continue;
      const div=selected.length?Math.min(...selected.map(q=>Math.abs(Number(t.u??.5)-Number(q.u??.5)))):1;
      const score=1.7*t.__bridgeScore+.9*div;
      if(score>bestScore){bestScore=score;best=t;}
    }
    if(!best)break;used.add(best.id);selected.push(best);
  }
  return selected.slice(0,tagCount).map(t=>t.id);
}


function anchorPlanarKey(a) {
  // physical installation rule. Anchor candidates that differ only in Y
  // represent the same real-world stand/column and cannot be selected together.
  return `${Number(a?.x||0).toFixed(4)}:${Number(a?.z||0).toFixed(4)}`;
}

function uniqueAnchorPlanarCount(anchors) {
  return new Set((Array.isArray(anchors)?anchors:[]).map(anchorPlanarKey)).size;
}

function selectAnchorIdsUniquePlanar(anchors, tagRef, anchorCount) {
  if (!Array.isArray(anchors) || !anchors.length || anchorCount < 1) return [];
  const minX=Math.min(...anchors.map(a=>Number(a.x)||0)),maxX=Math.max(...anchors.map(a=>Number(a.x)||0));
  const minZ=Math.min(...anchors.map(a=>Number(a.z)||0)),maxZ=Math.max(...anchors.map(a=>Number(a.z)||0));
  const minY=Math.min(...anchors.map(a=>Number(a.y)||0)),maxY=Math.max(...anchors.map(a=>Number(a.y)||0));
  const spanX=Math.max(.001,maxX-minX),spanZ=Math.max(.001,maxZ-minZ),spanY=Math.max(.001,maxY-minY);
  const safeTags=Array.isArray(tagRef)?tagRef:[];
  const tagJ=(t,a)=>{
    const nx=Number(t.nx??0),ny=Number(t.ny??1),nz=Number(t.nz??0),nm=Math.hypot(nx,ny,nz)||1;
    const dx=Number(t.x)-Number(a.x),dy=Number(t.y)-Number(a.y),dz=Number(t.z)-Number(a.z),d=Math.hypot(dx,dy,dz);
    return d>1e-9?(dx*nx+dy*ny+dz*nz)/(d*nm):0;
  };
  const geomScore=a=>{
    if(!safeTags.length)return 0;
    const ss=safeTags.map(t=>Math.abs(tagJ(t,a)));
    return .72*(ss.reduce((q,v)=>q+v,0)/ss.length)+.28*Math.min(...ss);
  };
  const sideScore=a=>safeTags.length?Math.sign(safeTags.reduce((sum,t)=>sum+tagJ(t,a),0)/safeTags.length):0;
  const selected=[],usedIds=new Set(),usedColumns=new Set(),ids=[];
  while(ids.length<anchorCount && usedIds.size<anchors.length){
    let best=null,bestScore=-Infinity;
    for(const a of anchors){
      if(usedIds.has(a.id))continue;
      const column=anchorPlanarKey(a);
      if(usedColumns.has(column))continue; // hard physical constraint
      const g=geomScore(a),side=sideScore(a);
      let div=1,quadBonus=0,heightBonus=0,sideBonus=0;
      if(selected.length){
        // Planar diversity is primary. Y diversity is only a secondary tie-breaker.
        div=Math.min(...selected.map(b=>Math.sqrt(((Number(a.x)-Number(b.x))/spanX)**2+((Number(a.z)-Number(b.z))/spanZ)**2)));
        quadBonus=selected.some(b=>b.quadrant===a.quadrant)?0:.20;
        heightBonus=selected.some(b=>Math.abs(Number(b.y)-Number(a.y))>spanY*.18)?.08:0;
        const sides=new Set(selected.map(sideScore));if(side!==0&&!sides.has(side))sideBonus=.18;
      }
      const score=1.65*g+.82*Math.min(div,1.4)+quadBonus+heightBonus+sideBonus;
      if(score>bestScore){bestScore=score;best=a;}
    }
    if(!best)break;
    usedIds.add(best.id);selected.push(best);usedColumns.add(anchorPlanarKey(best));ids.push(best.id);
  }
  return ids;
}

function demoResult(body) {
  const { profile, candidates, stage } = body;
  const tags = candidates.tags || [];
  const anchors = candidates.anchors || [];
  const tagCount = Math.max(1, Math.min(12, Number(body.tagCount || 4)));
  const anchorCount = Math.max(1, Math.min(12, Number(body.anchorCount || 4)));

  // structure-aware Tag selection. Surface+Normal candidates remain generic,
  // but bridge/viaduct monitoring prioritizes deformable superstructure points instead
  // of rewarding pier/support surfaces merely for spatial diversity. Tunnel/general
  // structures keep the original generic Surface selection behaviour.
  let tagIds=[];
  const bridgeMode=isBridgeProfile(profile);
  if(bridgeMode){
    tagIds=selectBridgeTagIds(tags,tagCount);
  }else if(tags.length){
    const minX=Math.min(...tags.map(t=>Number(t.x)||0)),maxX=Math.max(...tags.map(t=>Number(t.x)||0));
    const minY=Math.min(...tags.map(t=>Number(t.y)||0)),maxY=Math.max(...tags.map(t=>Number(t.y)||0));
    const minZ=Math.min(...tags.map(t=>Number(t.z)||0)),maxZ=Math.max(...tags.map(t=>Number(t.z)||0));
    const sx=Math.max(.001,maxX-minX),sy=Math.max(.001,maxY-minY),sz=Math.max(.001,maxZ-minZ);
    const used=new Set(),selected=[];
    while(tagIds.length<tagCount&&used.size<tags.length){
      let best=null,bestScore=-Infinity;
      for(const t of tags){
        if(used.has(t.id))continue;
        const access=Math.min(1,Math.max(0,Number(t.accessibility_m||0))/Math.max(sx,sy,sz,.001));
        let div=1,patchBonus=0,classBonus=0;
        if(selected.length){
          div=Math.min(...selected.map(q=>Math.sqrt(((Number(t.x)-Number(q.x))/sx)**2+((Number(t.y)-Number(q.y))/sy)**2+((Number(t.z)-Number(q.z))/sz)**2)));
          patchBonus=selected.some(q=>q.surfacePatch===t.surfacePatch)?0:.22;
          classBonus=selected.some(q=>(q.surfaceClass||q.surface)===(t.surfaceClass||t.surface))?0:.12;
        }
        const score=.9*Math.min(div,1.5)+.22*access+patchBonus+classBonus;
        if(score>bestScore){bestScore=score;best=t;}
      }
      if(!best)break;used.add(best.id);selected.push(best);tagIds.push(best.id);
    }
  }

  // Anchor geometry is evaluated along each Tag's local surface normal. A bridge
  // top surface naturally reduces to the old vertical-sensitivity case; a tunnel
  // wall/crown automatically uses its radial/local normal instead.
  const anchorIds=[];
  if(anchors.length){
    const chosenTags=tags.filter(t=>tagIds.includes(t.id));
    const tagRef=chosenTags.length?chosenTags:tags.slice(0,Math.max(1,tagCount));
    anchorIds.push(...selectAnchorIdsUniquePlanar(anchors,tagRef,anchorCount));
  }

  const rawType = profile?.geometryHypothesis?.type || "girder_bridge";
  const typeMap = {
    multi_span_girder_viaduct: "다경간 거더식 고가도로",
    girder_bridge: "거더교",
    elongated_bridge_or_viaduct: "장방형 교량 또는 고가도로",
    tunnel_or_lining_structure: "터널 또는 라이닝 구조물",
    unclassified_structure: "미분류 구조물"
  };

  return {
    mode: "demo",
    structure_assessment: {
      category: profile?.geometryHypothesis?.category==="tunnel"?"터널":profile?.geometryHypothesis?.category==="bridge"?"교량/고가도로":"일반 구조물",
      type: typeMap[rawType] || rawType,
      confidence: profile?.geometryHypothesis?.confidence ?? 0.82,
      summary: "데모 모드에서는 기하 분석 결과를 바탕으로 구조 형식을 추정합니다. 실제 API 모드에서는 AI가 웹 검색을 통해 구조 형식과 관련 기술자료를 독립적으로 확인합니다.",
      uncertainties: profile?.uncertainties || ["형상 정보만으로는 재료 특성과 내부 접합 상세를 확정할 수 없습니다."]
    },
    research_summary: [
      {
        title: "데모 모드 — 외부 웹 검색 미실행",
        source: "로컬 프로토타입",
        url: "",
        insight: "API 키를 연결하면 AI가 구조 형식, 해체 위험요소, 구조 계측 및 센서 배치 관련 자료를 직접 검색한 뒤 최종 배치를 결정합니다."
      }
    ],
    monitoring_strategy: bridgeMode ? [
      `교량/고가도로로 판단되어 Tag ${tagCount}개를 교각·지지부보다 상판·거더·경간 등 주 변형부에 우선 배치합니다.`,
      "Tag는 길이 방향으로 좌·중·우에 분산하여 연속 처짐, 비대칭 침하와 국부 변형을 비교합니다.",
      `Anchor ${anchorCount}개는 독립 고정 기준점으로 배치하고, 각 Tag의 표면 법선 방향 감도가 커지도록 위치·높이를 분산합니다.`
    ] : [
      `요청된 Tag ${tagCount}개를 서로 다른 설치 가능 표면/영역에 공간적으로 분산하여 법선 변위·기울기를 비교합니다.`,
      "동일 영역에 계측점이 과도하게 중복되지 않도록 후보 간 분산도를 고려합니다.",
      `Anchor ${anchorCount}개는 독립 고정 기준점으로 배치하고, 각 Tag의 표면 법선 방향 감도가 커지도록 위치·높이를 분산합니다.`
    ],
    final_placement: {
      tag_candidate_ids: tagIds,
      anchor_candidate_ids: anchorIds
    },
    placement_rationale: bridgeMode ? [
      "교량에서는 공간 분산 자체보다 상판·거더·경간 같은 주 변형부의 계측 가치를 우선했습니다. 교각·기둥·받침부는 주 변형부 후보가 부족하거나 지지부 거동 계측이 명시적으로 필요한 경우에만 후순위로 사용합니다.",
      "여러 Tag를 길이 방향으로 분산하여 한 점의 이동이 아니라 구조물 전체의 연속적인 처짐 형상과 비대칭 변형을 비교할 수 있도록 했습니다.",
      "Anchor는 계측 대상 변형부와 독립된 고정 기준 후보 중에서 법선 감도와 위치·높이 분산을 고려해 선정했습니다."
    ] : [
      "Tag는 상부면에 한정하지 않고 벽·천장·경사면을 포함한 실제 설치 가능 Surface 후보 중에서 선정했습니다.",
      "Anchor는 계측 대상 변형부와 독립된 고정 기준 후보 중에서 선정했으며, 외곽 스탠드와 터널 내부 같은 free-space 고정 스탠드 후보를 함께 비교했습니다.",
      "사용자가 설정한 센서 개수를 유지하면서 구조물의 여러 구간을 함께 감시하도록 배치했습니다."
    ],
    warnings: [
      "현재 결과는 프로토타입의 AI 추천안이며 실제 시공 승인을 의미하지 않습니다.",
      "실제 해체공사 계측 위치는 구조·안전 담당자의 검토가 필요합니다."
    ]
  };
}

function extractJson(text) {
  if (!text) throw new Error("Empty AI response");
  const fenced = text.match(/```(?:json)?\s*([\s\S]*?)```/i);
  const raw = fenced ? fenced[1] : text;
  const first = raw.indexOf("{");
  const last = raw.lastIndexOf("}");
  if (first < 0 || last <= first) throw new Error("AI response did not contain JSON");
  return JSON.parse(raw.slice(first, last + 1));
}


async function callOpenAIWithRetry(client, request, maxRetries = 4) {
  let lastError = null;

  for (let attempt = 0; attempt <= maxRetries; attempt++) {
    try {
      return await client.responses.create(request);
    } catch (err) {
      lastError = err;
      const status = err?.status;
      const retryable = [429, 500, 502, 503, 504].includes(status);

      if (!retryable || attempt === maxRetries) {
        throw err;
      }

      const delay = Math.pow(2, attempt) * 1500 + Math.random() * 700;
      console.log(
        `OpenAI 일시 오류 (${status}) — ${attempt + 1}/${maxRetries} 재시도, ${Math.round(delay)}ms 대기`
      );
      await new Promise(resolve => setTimeout(resolve, delay));
    }
  }

  throw lastError;
}

app.post("/api/ai/recommend", async (req, res) => {
  const input = req.body || {};
  const limits=ModelAnchorLimits.forFiles(input.sourceFiles);
  // Hard-filter both candidate classes before demo or OpenAI selection so the
  // final recommendation can never escape a model-specific coordinate envelope.
  const tags=(input?.candidates?.tags || []).filter(c=>ModelAnchorLimits.contains(c,limits));
  const anchors=(input?.candidates?.anchors || []).filter(c=>ModelAnchorLimits.contains(c,limits));
  const body={...input,candidates:{...input.candidates,tags,anchors}};
  const tagCount = Math.max(1, Math.min(12, Number(body.tagCount || 4)));
  const anchorCount = Math.max(1, Math.min(12, Number(body.anchorCount || 4)));

  const uniqueAnchorColumns=uniqueAnchorPlanarCount(anchors);
  if (tags.length < tagCount || anchors.length < anchorCount || uniqueAnchorColumns < anchorCount) {
    return res.status(400).json({
      error: `모델에서 필요한 후보 수를 확보하지 못했습니다. 필요: Tag ${tagCount}, Anchor ${anchorCount} / 생성: Tag ${tags.length}, Anchor ${anchors.length} / 서로 다른 Anchor XZ 위치: ${uniqueAnchorColumns}`
    });
  }

  if (!process.env.OPENAI_API_KEY) {
    return res.json(demoResult(body));
  }

  try {
    const client = new OpenAI({ apiKey: process.env.OPENAI_API_KEY });
    const model = process.env.OPENAI_MODEL || "gpt-5.6";

    const compactTags = tags.slice(0, 240).map(c => ({
      id: c.id, u: c.u, v: c.v, zone: c.zone,
      xyz_m: [c.x, c.y, c.z].map(n => Number(n.toFixed(3))),
      normal: [c.nx??0,c.ny??1,c.nz??0].map(n=>Number(Number(n).toFixed(3))),
      surface_class: c.surfaceClass || c.surface || "unknown",
      surface_patch: c.surfacePatch || "",
      source_mesh_name: String(c.sourceMeshName||"").slice(0,80),
      structural_role: c.structuralRole || "generic_surface",
      bridge_priority_score: Number(Number(c.bridgePriorityScore||0).toFixed(2)),
      mesh_long_ratio: Number(Number(c.meshLongRatio||0).toFixed(3)),
      accessibility_m: c.accessibility_m==null?null:Number(Number(c.accessibility_m).toFixed(3)),
      supportDistance_m: c.supportDistance == null ? null : Number(c.supportDistance.toFixed(3))
    }));
    const compactAnchors = anchors.slice(0, 180).map(c => ({
      id: c.id, quadrant: c.quadrant,
      xyz_m: [c.x, c.y, c.z].map(n => Number(n.toFixed(3))),
      height_above_ground_m: Number(Number(c.heightAboveGround ?? c.y ?? 0).toFixed(3)),
      height_ratio_to_model: c.heightRatio==null?null:Number(Number(c.heightRatio).toFixed(2)),
      mount: c.mount || "external_fixed_support",
      outsideFootprint: c.mount === "external_fixed_support"
    }));

    const prompt = `
당신은 구조물 해체공사 디지털 트윈의 자율형 센서 배치 AI입니다.

사용자가 프로젝트 문서를 제공한 것이 아닙니다.
당신이 다음 작업의 주체가 되어야 합니다.

1) Geometry Tool 프로필만 보고 구조물의 종류와 구조 형식을 스스로 판단합니다.
2) 웹 검색을 사용해 그 판단을 검증합니다.
3) 해당 구조물의 해체/철거 시 주요 구조적 위험요소, 구조 계측 방법, 센서 배치 관련 기술자료를 직접 찾습니다.
4) 검색 근거를 바탕으로 무엇을 우선 감시할지 계측 전략을 수립합니다.
5) 아래에 제공된 실제 후보 ID를 비교하고 사용자가 요청한 개수만큼 최종 센서를 직접 결정합니다.

중요 규칙:
- 좌표를 새로 만들어내지 마십시오.
- 반드시 제공된 후보 ID만 선택하십시오.
- Geometry Tool은 물리적으로 가능한 좌표를 제공하는 도구/안전장치이고, 최종 추천 결정권은 당신에게 있습니다.
- 구조물을 교량으로 가정하지 마십시오. 터널, 건물, 벽체, 슬래브, 경사 구조물 등 입력 형상에 맞춰 후보 Surface를 해석하십시오.
- Tag는 물리적으로 설치 가능한 Surface 후보 중에서 구조적 계측 가치와 공간 분산을 함께 보고 선택하십시오.
- 요청 Tag 개수: ${tagCount}개. Tag는 UWB로 해당 설치면의 로컬 법선 방향 변위(Δn)를 추정하고, ADXL345로 기울기/진동을 측정합니다.
- Tag 후보는 상판에 한정되지 않습니다. upper / wall / ceiling / inclined Surface가 모두 가능하며, 각 후보에는 표면 법선 normal이 포함됩니다.
- 단, 당신이 구조물을 교량/고가도로/거더교로 판단한 경우에는 structural_role과 bridge_priority_score를 반드시 활용하십시오. 상판·거더·경간 등 primary_superstructure를 Tag의 1순위로 선택하고, 길이 방향 u가 좌·중·우로 분산되도록 하십시오.
- 교량에서 pier / column / pedestal / abutment / support / bearing 같은 지지부 후보를 단순히 공간 분산이나 Surface 다양성 때문에 선택하지 마십시오. 주 변형부 후보가 요청 개수보다 부족하거나, 계측 전략상 지지부 거동 자체를 반드시 감시해야 하는 명확한 근거가 있을 때만 후순위로 선택하십시오.
- 요청 Anchor 개수: ${anchorCount}개. Anchor는 UWB의 고정 기준점입니다.
- 물리 설치 제약: 같은 X/Z 좌표에서 Y 높이만 다른 Anchor 후보는 동일한 설치 위치입니다. 동일 X/Z 수직열에서는 반드시 1개만 선택하십시오. 선택된 모든 Anchor는 서로 다른 X/Z 좌표를 가져야 합니다.
- Anchor 배치는 높이 다양성보다 먼저 X/Z 평면 분산을 확보하십시오. 서로 다른 X/Z 위치를 고른 뒤 각 위치에서 가장 적합한 높이 하나를 선택하십시오.
- Anchor는 계측 대상 Tag가 붙은 변형부와 독립된 고정 기준 위치여야 합니다. 반드시 모델 bounding box 바깥일 필요는 없습니다.
- 터널처럼 내부 공동이 있는 구조물에서는 free_space_fixed_stand 후보를 사용할 수 있습니다. 이는 터널 내부 자유공간의 독립 스탠드/지지대 설치를 의미합니다.
- Anchor는 바닥에만 둘 필요가 없고 높이를 줄 수 있습니다.
- 제공된 Anchor 후보는 이미 모델 높이의 최대 약 2배 이내로 제한되어 있으므로, 후보의 y 좌표를 그대로 비교하십시오.
- UWB 변위 정확도는 단순 높이차가 아니라 각 Tag의 표면 법선 n과 Tag→Anchor 시선벡터의 정렬도 |dot(unit(Tag-Anchor), n)|에 좌우됩니다.
- 교량 상판에서는 이것이 기존 수직 감도와 거의 같고, 터널 측벽/천단에서는 자동으로 수평·방사 방향 감도로 바뀝니다.
- Anchor를 선택할 때 각 Tag에 대한 법선 감도가 너무 작은 후보를 피하고, 여러 Anchor의 감도 부호와 위치/높이를 다양하게 하여 공통 거리 drift를 구분하기 쉽게 하십시오.
- 현재 해체 단계: ${body.stage || "AI가 자체 판단"}

GEOMETRY TOOL 프로필:
${JSON.stringify(body.profile, null, 2)}

TAG 후보:
${JSON.stringify(compactTags)}

ANCHOR 후보:
${JSON.stringify(compactAnchors)}


Anchor 선택 시 평면상 사방 분산만 보지 말고 각 Tag 후보의 normal과 Anchor 시선벡터의 |dot| 법선 감도, 감도 부호 다양성, 높이 다양성을 함께 평가하십시오.
Tag가 벽/천장/경사면에 있으면 Y 높이차만으로 평가하지 마십시오.

출력은 반드시 JSON만 반환하고, 설명 문장은 전부 한국어로 작성하십시오.
출처의 고유 제목/기관명은 원문을 유지해도 되지만 insight는 한국어로 작성하십시오.
후보 ID는 원문 그대로 유지하십시오.

정확한 최상위 JSON 형식:
{
  "structure_assessment": {
    "category": "한국어 구조물 대분류",
    "type": "한국어 구조 형식",
    "confidence": 0.0,
    "summary": "구조물을 그렇게 판단한 핵심 근거를 한국어로 간단히 설명",
    "uncertainties": ["형상만으로 확정할 수 없는 사항을 한국어로"]
  },
  "research_summary": [
    {
      "title":"자료 제목",
      "source":"기관 또는 발행처",
      "url":"https://...",
      "insight":"이 자료가 센서 위치 결정에 어떤 영향을 주었는지 한국어로 설명"
    }
  ],
  "monitoring_strategy": ["한국어 계측 전략"],
  "final_placement": {
    "tag_candidate_ids": ["정확히 ${tagCount}개의 유효한 후보 ID"],
    "anchor_candidate_ids": ["정확히 ${anchorCount}개의 유효한 후보 ID"]
  },
  "placement_rationale": ["최종 위치를 고른 이유를 한국어로"],
  "warnings": ["불확실성 또는 실제 적용 시 주의사항을 한국어로"]
}

자료 검색 우선순위:
공식 기준/지침 > 정부·공공기관 > 대학/연구기관 > 학술·기술자료 > 신뢰할 수 있는 엔지니어링 기관.

길게 늘어놓지 말고, 실제 위치 추천에 영향을 준 근거 중심으로 작성하십시오.
내부 추론 과정(chain-of-thought)은 공개하지 말고 판단 요약만 반환하십시오.
`;

    const response = await callOpenAIWithRetry(client, {
      model,
      tools: [{ type: "web_search" }],
      input: prompt
    });

    const result = extractJson(response.output_text);

    const validTagIds = new Set(tags.map(x => x.id));
    const validAnchorIds = new Set(anchors.map(x => x.id));
    const selectedTags = result?.final_placement?.tag_candidate_ids || [];
    const selectedAnchors = result?.final_placement?.anchor_candidate_ids || [];

    const valid =
      selectedTags.length === tagCount &&
      selectedAnchors.length === anchorCount &&
      selectedTags.every(x => validTagIds.has(x)) &&
      selectedAnchors.every(x => validAnchorIds.has(x));

    if (!valid) {
      const fallback = demoResult(body);
      fallback.mode = "ai_fallback";
      fallback.warnings = [
        ...(result?.warnings || []),
        "AI가 유효하지 않은 후보 ID를 반환하여, 실제 기하 후보 안에서 안전한 대체 배치를 적용했습니다."
      ];
      fallback.structure_assessment = result?.structure_assessment || fallback.structure_assessment;
      fallback.research_summary = result?.research_summary || fallback.research_summary;
      fallback.monitoring_strategy = result?.monitoring_strategy || fallback.monitoring_strategy;
      return res.json(fallback);
    }

    // hard installation guard: one Anchor per X/Z stand/column.
    // AI may choose candidates at different heights from the same generated vertical column;
    // that physical arrangement is not installable, so repair it deterministically.
    const aiAnchorObjs=selectedAnchors.map(id=>anchors.find(a=>a.id===id)).filter(Boolean);
    const aiAnchorColumns=new Set(aiAnchorObjs.map(anchorPlanarKey));
    if(aiAnchorColumns.size<anchorCount){
      const aiTagObjs=selectedTags.map(id=>tags.find(t=>t.id===id)).filter(Boolean);
      const repaired=selectAnchorIdsUniquePlanar(anchors,aiTagObjs.length?aiTagObjs:tags.slice(0,tagCount),anchorCount);
      if(repaired.length===anchorCount){
        result.final_placement.anchor_candidate_ids=repaired;
        result.placement_rationale=[
          ...(Array.isArray(result.placement_rationale)?result.placement_rationale:[]),
          "실제 설치 제약에 따라 같은 X/Z 위치에서 높이만 다른 Anchor 중복을 제거하고, 서로 다른 평면 위치 4곳에서 각각 하나의 높이를 선택하도록 보정했습니다."
        ];
        result.warnings=[
          ...(Array.isArray(result.warnings)?result.warnings:[]),
          "동일 X/Z 수직열에 복수 Anchor를 설치할 수 없으므로 최종 Anchor 배치를 평면 분산 우선 규칙으로 자동 보정했습니다."
        ];
      }
    }

    // Bridge superstructure guard: if the AI identifies a bridge and enough primary
    // superstructure candidates exist, do not allow pier/support Tags to win only due
    // to spatial diversity. This does not affect tunnels or general structures.
    const aiProfile={geometryHypothesis:{
      category:String(result?.structure_assessment?.category||body?.profile?.geometryHypothesis?.category||""),
      type:String(result?.structure_assessment?.type||body?.profile?.geometryHypothesis?.type||"")
    }};
    if(isBridgeProfile(aiProfile)){
      const preferred=tags.filter(t=>tagBridgePriority(t)>=1.8);
      const chosen=selectedTags.map(id=>tags.find(t=>t.id===id)).filter(Boolean);
      const hasSupportLike=chosen.some(t=>tagBridgePriority(t)<.2);
      if(preferred.length>=tagCount&&hasSupportLike){
        result.final_placement.tag_candidate_ids=selectBridgeTagIds(tags,tagCount);
        result.placement_rationale=[
          ...(Array.isArray(result.placement_rationale)?result.placement_rationale:[]),
          "교량 주 변형부 우선 규칙을 적용해 교각·기둥·지지부 대신 상판·거더·경간 후보로 Tag 위치를 보정했습니다."
        ];
      }
    }
    result.mode = "ai";
    return res.json(result);
  } catch (err) {
    console.error(err);
    const fallback = demoResult(body);
    fallback.mode = "error_fallback";
    fallback.warnings = [
      `AI 요청 실패: ${err.message}`,
      ...fallback.warnings
    ];
    return res.json(fallback);
  }
});

app.listen(PORT, "0.0.0.0", () => {
  console.log(`SOC Digital Twin R10.2 running at http://localhost:${PORT}`);
  console.log(`LAN access: http://<THIS_PC_IPV4>:${PORT}`);
  console.log(process.env.OPENAI_API_KEY ? "AI agent: enabled" : "AI agent: DEMO mode (no OPENAI_API_KEY)");
});
