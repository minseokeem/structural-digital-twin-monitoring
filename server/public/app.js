(() => {
  const $ = id => document.getElementById(id);

  const state = {
    sourceType: null,
    files: [],
    modelGroup: null,
    deckMesh: null,
    deckOriginal: null,
    modelMeta: null,
    profile: null,
    candidates: { tags: [], anchors: [] },
    selected: { tags: [], anchors: [] },
    sensorObjects: [],
    candidateObjects: [],
    aiResult: null,
    simOn: false,
    simTimer: null,
    simT: 0,
    history: Array(11).fill(55),
    aiEnabled: false,
    tagCount: 1,
    anchorCount: 4,
    sensorConfigConfirmed: false,
    moduleMappings: {},
    modulePool: [],
    discoveredModules: [],
    liveSensorStates: {},
    moduleSignature: "",
    sensorStreamConnected: false,
    sensorCase: {
      ready:false,
      geometries:[],
      naturalSize:1,
      height:1,
      // actual combined enclosure dimensions from the uploaded STL pair
      // are calculated again in-browser when assets load
      widthMm:69.93,
      heightMm:47.0,
      depthMm:63.73
    },
    deformTargets: [],
    deformationBand: null,
    surfaceAnalysis: null,
    topSurfaceRaycaster: new THREE.Raycaster(),

    // /UX state. Core geometry, AI, UWB and sensor logic remains unchanged.
    uiFlow: "upload",
    workspaceView: "viewer",
    monitoringActive: false,
    calibrationRequested: false,
    calibrationStartedAt: 0,
    calibrationZeroPending: false,
    calibrationGeneration: null,
    selectedTagPopupIndex: null,
    monitorHistory: {},
    monitorHistoryLastMs: {},
    riskEvents: [],
    lastRiskStates: {}
  };

  // live-view rendering budget. Declared up front because the workspace
  // view is initialized before the sensor-stream section later in this file.
  const LIVE_VISUAL_MIN_MS = 200;
  let liveVisualTimer = null;
  let lastLiveVisualMs = 0;
  let liveVisualNeedsDeform = false;
  let lastDeformSignature = "";
  let lastTrendUpdateMs = 0;

  // visual risk thresholds. These affect only UI risk classification/visuals.
  // Core UWB/AI/geometry calculations are untouched.
  const RISK_LIMITS = Object.freeze({
    warningDzMm: 15,
    dangerDzMm: 30,
    warningTiltDeg: 5,
    dangerTiltDeg: 10,
    // Live vibration is treated as dangerous only above this dynamic-acceleration level.
    // This filters tiny desk/handling vibrations without changing firmware acquisition.
    dangerVibrationG: 0.10
  });

  // Viewer-only gains. Sensor values, UWB calculations and CSV values are unchanged.
  const VISUAL_GAINS = Object.freeze({
    displacementMetric: 0.00115,
    displacementRelativeDivisor: 1300,
    tilt: 1.0
  });

  // ---------- Three.js ----------
  const host = $("canvasHost");
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x30343a);

  const camera = new THREE.PerspectiveCamera(42, host.clientWidth / host.clientHeight, 0.01, 2000);
  camera.position.set(13, 8.5, 16);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));
  renderer.setSize(host.clientWidth, host.clientHeight);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.outputEncoding = THREE.sRGBEncoding;
  host.appendChild(renderer.domElement);

  const controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = .06;
  controls.target.set(0, 1.5, 0);

  scene.add(new THREE.HemisphereLight(0xffffff, 0x83939b, 1.08));
  const sun = new THREE.DirectionalLight(0xffffff, 1.25);
  sun.position.set(10, 15, 8);
  sun.castShadow = true;
  sun.shadow.mapSize.set(1024, 1024);
  scene.add(sun);
  const fill = new THREE.DirectionalLight(0xbad9e8, .4);
  fill.position.set(-8, 6, -6);
  scene.add(fill);

  const grid = new THREE.GridHelper(50, 50, 0xffffff, 0xffffff);
  grid.position.y = 0;
  if (Array.isArray(grid.material)) {
    grid.material.forEach(m=>{m.transparent=true;m.opacity=.18;});
  } else {
    grid.material.transparent=true;
    grid.material.opacity=.18;
  }
  scene.add(grid);

  const ground = new THREE.Mesh(
    new THREE.PlaneGeometry(60, 40),
    // Keep the floor close to the dark viewer background without allowing the
    // scene lights to wash it out. MeshBasicMaterial preserves the intended tone.
    new THREE.MeshBasicMaterial({ color: 0x3a3f46 })
  );
  ground.rotation.x = -Math.PI / 2;
  ground.position.y = -0.01;
  ground.receiveShadow = true;
  scene.add(ground);

  function resize() {
    camera.aspect = host.clientWidth / host.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(host.clientWidth, host.clientHeight);
  }
  window.addEventListener("resize", resize);
  const _screenScaleWorldPos=new THREE.Vector3();
  function worldUnitsPerPixelAt(object3d){
    if(!object3d || !host.clientHeight)return 0;
    object3d.getWorldPosition(_screenScaleWorldPos);
    const distance=Math.max(.001,camera.position.distanceTo(_screenScaleWorldPos));
    const verticalWorld=2*distance*Math.tan(THREE.MathUtils.degToRad(camera.fov*.5));
    return verticalWorld/Math.max(1,host.clientHeight);
  }

  function updateAdaptiveSensorVisuals(){
    state.sensorObjects.forEach(o=>{
      if(!o?.group || !o?.body)return;
      const d=o.group.userData||{};
      const unitsPerPx=worldUnitsPerPixelAt(o.group);
      if(!(unitsPerPx>0))return;
      const baseRadius=Math.max(1e-6,Number(o.body.userData?.markerRadius||.01));
      const desiredRadiusPx=d.type==="tag"?7.5:7.0;
      const visualScale=THREE.MathUtils.clamp((unitsPerPx*desiredRadiusPx)/baseRadius,.45,120);
      o.body.scale.setScalar(visualScale);
      if(o.label){
        const labelBase=o.label.userData?.baseScale;
        const baseHeight=Math.max(1e-6,Number(labelBase?.y||o.label.scale.y||.05));
        const desiredLabelHeight=unitsPerPx*24;
        const labelScale=THREE.MathUtils.clamp(desiredLabelHeight/baseHeight,.45,120);
        if(labelBase)o.label.scale.set(labelBase.x*labelScale,labelBase.y*labelScale,1);
        const baseLabelY=Number(o.label.userData?.baseY||0);
        o.label.position.y=baseLabelY + baseRadius*Math.max(0,visualScale-1)*.85;
      }
    });
    state.candidateObjects.forEach(o=>{
      if(!o?.userData?.screenAdaptive)return;
      const unitsPerPx=worldUnitsPerPixelAt(o);
      const baseRadius=Math.max(1e-6,Number(o.userData.baseRadius||.006));
      const sc=THREE.MathUtils.clamp((unitsPerPx*4.2)/baseRadius,.5,120);
      o.scale.setScalar(sc);
    });
  }

  function updateDangerVisuals(){
    const t=performance.now()*0.001;
    const pulse=.5+.5*Math.sin(t*5.2);
    state.sensorObjects.forEach(o=>{
      const body=o?.body;
      if(!body?.userData?.dangerActive)return;
      const aura=body.userData.dangerAura;
      const outer=body.userData.dangerAuraOuter;
      const badge=body.userData.dangerBadge;
      const point=body.userData.point;
      if(aura){
        const s=1+0.24*pulse;
        aura.scale.setScalar(s);
        if(aura.material)aura.material.opacity=.22+.28*pulse;
      }
      if(outer){
        const s=1.05+0.42*pulse;
        outer.scale.setScalar(s);
        if(outer.material)outer.material.opacity=.08+.24*(1-pulse);
      }
      if(badge){
        const base=Number(badge.userData.baseScale||badge.scale.x||1);
        const s=base*(.90+.18*pulse);
        badge.scale.set(s,s,1);
        if(badge.material)badge.material.opacity=.78+.22*pulse;
      }
      if(point?.material)point.material.emissiveIntensity=.48+.46*pulse;
    });
  }

  function animate() {
    requestAnimationFrame(animate);
    // Do not spend GPU/CPU time rendering a hidden Three.js canvas while the
    // statistics/AI pages are open.  Sensor data continues to update normally.
    if (state.workspaceView !== "viewer" || document.hidden) return;
    controls.update();
    updateAdaptiveSensorVisuals();
    updateDangerVisuals();
    renderer.render(scene, camera);
    updateTagInfoPanelPosition();
  }
  animate();

  // ---------- Sensor enclosure / module mapping ----------
  function clampInt(v, min, max, fallback) {
    const n = Number.parseInt(v, 10);
    return Number.isFinite(n) ? Math.max(min, Math.min(max, n)) : fallback;
  }

  function currentRequestedTotal() {
    return state.tagCount + state.anchorCount;
  }

  function refreshDynamicCountLabels() {
    const finalText = `최종 Tag ${state.tagCount} / Anchor ${state.anchorCount} 확정`;
    if ($("aiFinalStepLabel")) $("aiFinalStepLabel").textContent = finalText;
    if ($("loadingFinalStepLabel")) $("loadingFinalStepLabel").textContent = finalText;
    if ($("tagPlacedCount")) $("tagPlacedCount").textContent = `${state.selected.tags.length} / ${state.tagCount} 배치`;
    if ($("anchorPlacedCount")) $("anchorPlacedCount").textContent = `${state.selected.anchors.length} / ${state.anchorCount} 배치`;
    $("healthSensors").textContent = `${state.sensorObjects.length} / ${currentRequestedTotal()}`;
  }

  async function loadSensorCaseAssets() {
    try {
      const loader = new THREE.STLLoader();
      const load = url => new Promise((resolve, reject) => loader.load(url, resolve, undefined, reject));
      const [g1, g2] = await Promise.all([
        load("/assets/sensor_case_part1.stl"),
        load("/assets/sensor_case_part2.stl")
      ]);

      g1.computeBoundingBox();
      g2.computeBoundingBox();

      const combined = g1.boundingBox.clone().union(g2.boundingBox);
      const size = combined.getSize(new THREE.Vector3());

      state.sensorCase = {
        ready: true,
        geometries: [],        // V4.4: STL is not shown in the viewer
        naturalSize: Math.max(size.x,size.y,size.z),
        height: size.y,
        widthMm: size.x,
        heightMm: size.y,
        depthMm: size.z
      };

      addEvent(
        `센서 케이스 실제 크기 반영 · ${size.x.toFixed(1)} × ${size.z.toFixed(1)} × ${size.y.toFixed(1)} mm`,
        "INFO"
      );
    } catch (err) {
      console.error(err);
      // Fallback dimensions measured from the supplied enclosure pair.
      state.sensorCase = {
        ready:false, geometries:[], naturalSize:69.93, height:47.0,
        widthMm:69.93, heightMm:47.0, depthMm:63.73
      };
      addEvent("센서 케이스 크기 파일 로딩 실패 — 저장된 실제 치수 사용","오류");
    }
  }

  function caseMetricsInModelUnits() {
    const bbox = modelBoundingBox();
    const size = bbox.getSize(new THREE.Vector3());
    const maxDim = Math.max(size.x,size.y,size.z,1);

    // DXF reconstructed models and GLB/glTF are handled in metres.
    if (state.modelMeta?.unitsToMeters === 1 || state.modelMeta?.exactUnits) {
      return {
        width: state.sensorCase.widthMm * .001,
        depth: state.sensorCase.depthMm * .001,
        height: state.sensorCase.heightMm * .001,
        clearance: .005,       // 5 mm extra edge clearance
        physical: true
      };
    }

    // OBJ has no formal unit definition. Use a conservative visual-scale
    // equivalent while preserving the same enclosure aspect ratio.
    const width = Math.max(maxDim*.022, .10);
    return {
      width,
      depth: width * (state.sensorCase.depthMm/state.sensorCase.widthMm),
      height: width * (state.sensorCase.heightMm/state.sensorCase.widthMm),
      clearance: width*.08,
      physical: false
    };
  }
  function moduleIdList(role=null) {
    const live=state.discoveredModules.filter(m=>m.online && (!role || m.role===role));
    const ids=live.map(m=>m.node_id);
    // Preserve currently selected IDs even if a module just went offline.
    Object.values(state.moduleMappings).filter(Boolean).forEach(id=>{if(!ids.includes(id))ids.push(id)});
    return ids.sort();
  }

  function mappingKey(type, index) {
    return `${type}-${index}`;
  }

  function mappedModule(type, index) {
    // Anchors are UWB-only responders and never join Wi-Fi.
    // Their logical IDs are fixed by firmware, so they must not depend on
    // Wi-Fi discovery/moduleMappings. Only Tags are user-mapped Wi-Fi modules.
    if(type === "anchor") return `ANCHOR-${String(index).padStart(2,"0")}`;
    return state.moduleMappings[mappingKey(type,index)] || "";
  }

  async function refreshDiscoveredModules(showToast=false) {
    try {
      const r=await fetch("/api/modules");
      const data=await r.json();
      state.discoveredModules=Array.isArray(data.modules)?data.modules:[];
      state.moduleSignature=JSON.stringify(state.discoveredModules.map(m=>[m.node_id,m.role,m.online]));
      renderModuleMapping();
      if(showToast)toast(`실제 모듈 ${state.discoveredModules.filter(m=>m.online).length}개 검색됨`);
    }catch(err){
      if(showToast)toast("모듈 목록을 가져오지 못했습니다");
    }
  }

  function renderModuleMapping() {
    const list = $("moduleMappingList");
    if (!list) return;
    list.innerHTML = "";

    if (!state.sensorConfigConfirmed) {
      const empty = document.createElement("div");
      empty.className = "moduleMappingEmpty";
      empty.textContent = "Tag와 Anchor 개수를 먼저 선택하세요!";
      list.appendChild(empty);
      $("mappingState").textContent = "대기";
      return;
    }

    const specs = [
      ...Array.from({length:state.tagCount}, (_,i)=>({type:"tag", index:i+1, label:`Tag ${i+1}`})),
      ...Array.from({length:state.anchorCount}, (_,i)=>({type:"anchor", index:i+1, label:`Anchor ${i+1}`}))
    ];

    const selectedByOthers = key => new Set(
      Object.entries(state.moduleMappings)
        .filter(([k,v]) => k !== key && v)
        .map(([,v]) => v)
    );

    specs.forEach(spec => {
      const key = mappingKey(spec.type,spec.index);
      const row = document.createElement("div");
      row.className = "moduleMappingRow";
      const titleWrap=document.createElement("div");
      titleWrap.className="moduleMappingTitle";
      const statusDot=document.createElement("span");
      const title = document.createElement("strong");
      title.textContent = spec.label;

      if(spec.type === "anchor") {
        // architecture: Anchor has no Wi-Fi. It is addressed over UWB by
        // fixed logical ID and becomes "응답 확인" only after a Tag ranges it.
        const anchorId=mappedModule("anchor",spec.index);
        const info=state.discoveredModules.find(m=>m.node_id===anchorId);
        const observed=Boolean(info?.online && info?.role==="anchor");
        statusDot.className=`moduleStatusDot uwbOnly${observed?" observed":""}`;
        statusDot.title=observed?"UWB 응답 확인":"UWB 전용 · Wi-Fi 매핑 불필요";
        statusDot.setAttribute("aria-label",statusDot.title);
        titleWrap.append(statusDot,title);

        const fixed=document.createElement("select");
        fixed.disabled=true;
        const opt=document.createElement("option");
        opt.value=anchorId;
        opt.textContent=`${anchorId} · ${observed?"UWB 응답 확인":"UWB 전용"}`;
        fixed.appendChild(opt);
        row.append(titleWrap,fixed);
        list.appendChild(row);
        return;
      }

      const select = document.createElement("select");
      const used = selectedByOthers(key);
      const available=moduleIdList("tag");
      const empty = document.createElement("option");
      empty.value = "";
      empty.textContent = available.length ? "미연결" : "검색된 모듈 없음";
      select.appendChild(empty);
      available.forEach(id => {
        const info=state.discoveredModules.find(m=>m.node_id===id);
        const opt = document.createElement("option");
        opt.value = id;
        opt.textContent = `${id}${info?.online?" · 온라인":" · 오프라인"}${Number.isFinite(info?.wifi_rssi) && info.wifi_rssi !== 0 ? ` · ${info.wifi_rssi} dBm` : ""}`;
        opt.disabled = used.has(id);
        opt.selected = state.moduleMappings[key] === id;
        select.appendChild(opt);
      });
      select.onchange = async () => {
        if (select.value) state.moduleMappings[key] = select.value;
        else delete state.moduleMappings[key];
        renderModuleMapping();
        updateSensorLabels();
        await syncSensorLayout();
        applyLiveSensorData();
      };
      const mappedId=state.moduleMappings[key]||"";
      const mappedInfo=state.discoveredModules.find(m=>m.node_id===mappedId);
      const online=Boolean(mappedId && mappedInfo?.online);
      statusDot.className=`moduleStatusDot${online?" online":""}`;
      statusDot.title=online?"온라인":"오프라인";
      statusDot.setAttribute("aria-label",online?"온라인":"오프라인");
      titleWrap.append(statusDot,title);
      row.append(titleWrap, select);
      list.appendChild(row);
    });

    // Only the Wi-Fi Tags are required to be "connected" in module mapping.
    // Anchors are UWB-only and must not make the UI show 3/7 or block calibration.
    const connectedTags = Array.from({length:state.tagCount},(_,i)=>mappedModule("tag",i+1))
      .filter(id=>id && state.discoveredModules.some(m=>m.node_id===id&&m.role==="tag"&&m.online)).length;
    $("mappingState").textContent = `${connectedTags} / ${state.tagCount}`;
  }

  async function syncSensorLayout(strict=false) {
    if(!state.selected.tags.length && !state.selected.anchors.length)return;
    const tags=state.selected.tags.map((c,i)=>({
      logical_id:`Tag ${i+1}`,
      node_id:mappedModule("tag",i+1),
      x:c.x,y:c.y,z:c.z,
      nx:Number(c.nx??0),ny:Number(c.ny??1),nz:Number(c.nz??0),
      surface_class:c.surfaceClass||c.surface||"unknown",
      surface_patch:c.surfacePatch||""
    })).filter(x=>x.node_id);
    const anchors=state.selected.anchors.map((c,i)=>({
      logical_id:`Anchor ${i+1}`,
      node_id:mappedModule("anchor",i+1),
      x:c.x,y:c.y,z:c.z
    })).filter(x=>x.node_id);
    try{
      const response=await fetch("/api/sensors/layout",{
        method:"POST",headers:{"Content-Type":"application/json"},
        body:JSON.stringify({metric:Boolean(state.profile?.exactUnits),tags,anchors})
      });
      if(!response.ok)throw new Error('센서 좌표 동기화 실패');
      const result=await response.json();
      if(strict && (!result.metric || tags.length!==state.tagCount || anchors.length!==state.anchorCount))
        throw new Error('모델의 m 단위 좌표와 전체 Tag/Anchor 매칭을 확인해주세요.');
      return result;
    }catch(err){console.warn("layout sync failed",err);if(strict)throw err;}
  }

  async function autoMapDiscoveredModules() {
    let mapped = 0;

    for (let i=1; i<=state.tagCount; i++) {
      const expected = `TAG-${String(i).padStart(2,"0")}`;
      const found = state.discoveredModules.find(
        m => m.node_id === expected && m.role === "tag" && m.online
      );
      if (found) {
        state.moduleMappings[mappingKey("tag",i)] = found.node_id;
        mapped++;
      }
    }

    renderModuleMapping();
    updateSensorLabels();
    await syncSensorLayout();
    applyLiveSensorData();

    toast(mapped
      ? `실제 모듈 ${mapped}개를 ID 기준으로 자동 매핑했습니다`
      : "자동 매핑할 온라인 모듈을 찾지 못했습니다");
  }

  function applySensorConfiguration() {
    const nextTag = clampInt($("tagCountInput").value,1,12,4);
    const nextAnchor = clampInt($("anchorCountInput").value,1,12,4);
    const changed = nextTag !== state.tagCount || nextAnchor !== state.anchorCount;
    state.tagCount = nextTag;
    state.anchorCount = nextAnchor;
    state.sensorConfigConfirmed = true;
    $("tagCountInput").value = state.tagCount;
    $("anchorCountInput").value = state.anchorCount;

    // Remove mappings that no longer have a visible slot.
    Object.keys(state.moduleMappings).forEach(key => {
      const [type, num] = key.split("-");
      if ((type==="tag" && Number(num)>state.tagCount) ||
          (type==="anchor" && Number(num)>state.anchorCount)) {
        delete state.moduleMappings[key];
      }
    });

    if (changed) {
      clearSensors();
      state.aiResult = null;
      if (state.modelGroup) generateCandidates();
      $("simBtn").disabled = true;
      $("zeroBtn").disabled = true;
      addEvent(`센서 구성 변경 · Tag ${state.tagCount} / Anchor ${state.anchorCount}`,"INFO");
    }

    refreshDynamicCountLabels();
    renderModuleMapping();
    toast(`Tag ${state.tagCount}개 / Anchor ${state.anchorCount}개로 설정했습니다`);
  }

  $("applySensorConfigBtn").onclick = applySensorConfiguration;
  $("scanModulesBtn").onclick = () => refreshDiscoveredModules(true);
  $("autoMapModulesBtn").onclick = async () => {
    await refreshDiscoveredModules(false);
    await autoMapDiscoveredModules();
  };

  renderModuleMapping();
  refreshDynamicCountLabels();
  loadSensorCaseAssets();

  // ---------- R11.4 chat-style workspace orchestration ----------
  function setSidebarCollapsed(collapsed){
    document.querySelector(".app")?.classList.toggle("sidebar-collapsed",Boolean(collapsed));
    setTimeout(resize,220);
  }

  function setWorkspaceView(view){
    state.workspaceView=view;
    const map={viewer:"viewerView",ai:"aiResultsView",stats:"statsView"};
    Object.entries(map).forEach(([key,id])=>{
      $(id)?.classList.toggle("active",key===view);
    });
    document.querySelectorAll(".sideNavItem").forEach(btn=>btn.classList.remove("active"));
    if(view==="ai")$("aiResultsNavBtn")?.classList.add("active");
    if(view==="stats")$("statsNavBtn")?.classList.add("active");
    if(view==="viewer"&&state.uiFlow==="upload")$("newTaskNavBtn")?.classList.add("active");
    if(view!=="viewer")hideTagInfoPanel();
    if(view==="stats")renderStatsDashboard();
    if(view==="viewer")scheduleLiveVisualRefresh(true);
    setTimeout(resize,40);
    if(window.innerWidth<=820)setSidebarCollapsed(true);
  }

  function setFlowStage(stage){
    state.uiFlow=stage;
    const upload=$("initialUpload"),setup=$("sensorSetupCard"),chrome=$("viewerChrome");
    const flow=$("flowActionWrap"),btn=$("runAiBtn"),label=$("runAiBtnLabel");
    const text=$("flowActionText"),sub=$("flowActionSubtext");

    upload?.classList.toggle("hidden",stage!=="upload");
    setup?.classList.toggle("hidden",stage!=="configure");
    chrome?.classList.toggle("hidden",["upload","loading"].includes(stage));

    if(["upload","loading"].includes(stage)){
      flow?.classList.add("hidden");
    }else{
      flow?.classList.remove("hidden");
    }

    if(stage==="configure"){
      text.textContent="센서의 개수를 입력해 AI 추천을 받으세요";
      sub.textContent="Tag와 Anchor 개수를 확인한 뒤 진행하세요.";
      label.textContent="AI 추천 받기";
      btn.disabled=!state.modelGroup || state.candidates.tags.length<state.tagCount || state.candidates.anchors.length<state.anchorCount;
    }else if(stage==="recommended"){
      text.textContent="모니터링을 시작하시겠습니까 ?";
      sub.textContent="확인을 누르면 실제 좌표를 동기화하고 영점 보정을 시작합니다.";
      label.textContent="확인";
      btn.disabled=state.selected.tags.length<state.tagCount || state.selected.anchors.length<state.anchorCount;
    }else if(stage==="calibrating"){
      text.textContent="영점 보정을 진행하고 있습니다";
      sub.textContent="구조물을 원위치에 고정한 상태로 유지해주세요.";
      label.textContent="보정 중";
      btn.disabled=true;
    }else if(stage==="monitoring"){
      text.textContent="실시간 모니터링 중";
      sub.textContent="Tag를 클릭해 계측값을 확인하거나, 필요하면 재보정을 눌러 현재 원위치에서 영점 보정을 다시 시작할 수 있습니다.";
      label.textContent="재보정";
      btn.disabled=false;
    }
    if(stage==="upload")$("newTaskNavBtn")?.classList.add("active");
  }

  let sensorSetupWasDragged=false;

  function setSensorSetupCollapsed(collapsed){
    const card=$("sensorSetupCard");
    const button=$("sensorSetupCollapseBtn");
    if(!card||!button)return;
    card.classList.toggle("collapsed",Boolean(collapsed));
    button.setAttribute("aria-expanded",String(!collapsed));
    button.title=collapsed?"센서 설정 펼치기":"센서 설정 접기";
    button.innerHTML=collapsed
      ? '<svg viewBox="0 0 24 24" aria-hidden="true"><path d="m7 14 5-5 5 5"/></svg>'
      : '<svg viewBox="0 0 24 24" aria-hidden="true"><path d="m7 10 5 5 5-5"/></svg>';
    requestAnimationFrame(clampSensorSetupPosition);
  }

  function resetSensorSetupPosition(){
    const card=$("sensorSetupCard");
    if(!card)return;
    card.style.left="50%";
    card.style.top="50%";
    card.style.transform="translate(-50%,-50%)";
    sensorSetupWasDragged=false;
    setSensorSetupCollapsed(false);
  }

  function clampSensorSetupPosition(){
    const card=$("sensorSetupCard");
    const viewer=card?.closest(".viewer");
    if(!card||!viewer||!sensorSetupWasDragged||card.classList.contains("hidden"))return;
    const vr=viewer.getBoundingClientRect();
    const cr=card.getBoundingClientRect();
    const margin=10;
    const maxLeft=Math.max(margin,vr.width-cr.width-margin);
    const maxTop=Math.max(margin,vr.height-cr.height-margin);
    const currentLeft=Number.parseFloat(card.style.left)||margin;
    const currentTop=Number.parseFloat(card.style.top)||margin;
    card.style.left=`${Math.max(margin,Math.min(maxLeft,currentLeft))}px`;
    card.style.top=`${Math.max(margin,Math.min(maxTop,currentTop))}px`;
  }

  function initSensorSetupDrag(){
    const card=$("sensorSetupCard");
    const handle=$("sensorSetupDragHandle");
    const viewer=card?.closest(".viewer");
    if(!card||!handle||!viewer)return;

    let drag=null;
    handle.addEventListener("pointerdown",e=>{
      if(e.button!==undefined&&e.button!==0)return;
      const vr=viewer.getBoundingClientRect();
      const cr=card.getBoundingClientRect();
      card.style.transform="none";
      card.style.left=`${cr.left-vr.left}px`;
      card.style.top=`${cr.top-vr.top}px`;
      sensorSetupWasDragged=true;
      drag={
        pointerId:e.pointerId,
        startX:e.clientX,
        startY:e.clientY,
        startLeft:cr.left-vr.left,
        startTop:cr.top-vr.top
      };
      card.classList.add("dragging");
      handle.setPointerCapture?.(e.pointerId);
      e.preventDefault();
    });

    handle.addEventListener("pointermove",e=>{
      if(!drag||e.pointerId!==drag.pointerId)return;
      const vr=viewer.getBoundingClientRect();
      const cr=card.getBoundingClientRect();
      const margin=10;
      const maxLeft=Math.max(margin,vr.width-cr.width-margin);
      const maxTop=Math.max(margin,vr.height-cr.height-margin);
      const nextLeft=Math.max(margin,Math.min(maxLeft,drag.startLeft+(e.clientX-drag.startX)));
      const nextTop=Math.max(margin,Math.min(maxTop,drag.startTop+(e.clientY-drag.startY)));
      card.style.left=`${nextLeft}px`;
      card.style.top=`${nextTop}px`;
      e.preventDefault();
    });

    const endDrag=e=>{
      if(!drag||e.pointerId!==drag.pointerId)return;
      try{handle.releasePointerCapture?.(e.pointerId)}catch{}
      drag=null;
      card.classList.remove("dragging");
      clampSensorSetupPosition();
    };
    handle.addEventListener("pointerup",endDrag);
    handle.addEventListener("pointercancel",endDrag);
    window.addEventListener("resize",()=>requestAnimationFrame(clampSensorSetupPosition));
  }

  function initSensorSetupCollapse(){
    const button=$("sensorSetupCollapseBtn");
    const card=$("sensorSetupCard");
    if(!button||!card)return;
    button.addEventListener("pointerdown",e=>e.stopPropagation());
    button.addEventListener("click",e=>{
      e.stopPropagation();
      setSensorSetupCollapsed(!card.classList.contains("collapsed"));
    });
  }

  function resetToNewTask(){
    if(state.simOn)setSimulation(false);
    resetSensorSetupPosition();
    clearTwin();
    resetPipeline();
    state.files=[];
    state.sensorConfigConfirmed=false;
    state.sourceType=null;
    state.monitoringActive=false;
    state.calibrationRequested=false;
    state.calibrationStartedAt=0;
    state.selectedTagPopupIndex=null;
    state.monitorHistory={};
    state.monitorHistoryLastMs={};
    state.riskEvents=[];
    state.lastRiskStates={};
    if($("fileInput"))$("fileInput").value="";
    $("projectPath").textContent="새 작업";
    $("inputState").textContent="미입력";
    $("sourceStat").textContent="—";
    $("viewerTitle").textContent="디지털 트윈 미생성";
    $("viewerSubtitle").textContent="3D 모델 또는 도면 세트를 입력하세요";
    $("aiStructure").textContent="AI 분석 전입니다.";
    $("rationaleList").textContent="—";
    $("researchList").textContent="—";
    $("strategyList").textContent="—";
    $("placementCoordinateList").textContent="AI 추천이 확정되면 Tag와 Anchor의 절대좌표가 표시됩니다.";
    hideCalibrationToast();
    hideTagInfoPanel();
    setWorkspaceView("viewer");
    setFlowStage("upload");
    renderStatsDashboard();
  }

  async function handlePrimaryAction(){
    if(state.uiFlow==="configure"){
      applySensorConfiguration();
      if(state.candidates.tags.length<state.tagCount || state.candidates.anchors.length<state.anchorCount){
        toast("요청한 센서 수만큼 설치 후보를 만들 수 없습니다.");
        return;
      }
      await runAi();
      return;
    }
    if(state.uiFlow==="recommended"){
      await startMonitoringFlow();
      return;
    }
    if(state.uiFlow==="monitoring"){
      const ok=confirm("현재 구조물을 기준 위치에 고정한 뒤 재보정을 시작합니다. 기존 영점 기준은 새 보정값으로 교체됩니다. 계속할까요?");
      if(ok)await startMonitoringFlow();
    }
  }

  function showCalibrationProgress(percent,message,title="영점 보정 중"){
    const card=$("calibrationToast");
    card.classList.remove("hidden","done","fail");
    $("calibrationTitle").textContent=title;
    $("calibrationPercent").textContent=`${Math.max(0,Math.min(100,Math.round(percent)))}%`;
    $("calibrationMessage").textContent=message||"구조물을 원위치에 고정해주세요.";
    $("calibrationFill").style.width=`${Math.max(0,Math.min(100,percent))}%`;
  }

  function hideCalibrationToast(){
    $("calibrationToast")?.classList.add("hidden");
  }

  async function startMonitoringFlow(){
    if(state.selected.tags.length<state.tagCount || state.selected.anchors.length<state.anchorCount){
      toast("AI 추천 배치가 먼저 필요합니다.");
      return;
    }
    state.monitoringActive=false;
    state.calibrationRequested=true;
    state.calibrationZeroPending=true;
    state.calibrationGeneration=null;
    state.calibrationStartedAt=Date.now();
    setWorkspaceView("viewer");
    setFlowStage("calibrating");
    showCalibrationProgress(0,"실제 센서와 추천 좌표를 연결하고 있습니다.");

    try{
      await refreshDiscoveredModules(false);
      const onlineMappedTags=Array.from({length:state.tagCount},(_,i)=>mappedModule("tag",i+1))
        .filter(id=>id && state.discoveredModules.some(m=>m.node_id===id&&m.role==="tag"&&m.online)).length;
      if(onlineMappedTags<state.tagCount)await autoMapDiscoveredModules();
      await syncSensorLayout(true);

      const ids=Array.from({length:state.tagCount},(_,i)=>mappedModule("tag",i+1));
      const missing=ids.map((id,i)=>({id,i})).filter(({id})=>!id ||
        !state.discoveredModules.some(m=>m.node_id===id&&m.online));
      if(missing.length)throw new Error(`연결 확인: ${missing.map(({id,i})=>id||`Tag ${i+1} 미매칭`).join(', ')}`);
      const oldFirmware=ids.filter(id=>{const m=state.discoveredModules.find(x=>x.node_id===id);return m?.slot_protocol!==3||!['unified_information_v2','information_window_v1'].includes(m?.ranging_mode);});
      if(oldFirmware.length)throw new Error(`${oldFirmware.join(', ')}에 최신 Adaptive Quality-Aware 태그 펌웨어를 업로드해주세요.`);

      showCalibrationProgress(0,"좌표 동기화 완료 · 실제 UWB 보정 샘플을 기다립니다.");
      const ok=await performZeroCalibration();
      if(ok===false)throw new Error("영점 보정 명령을 전송하지 못했습니다.");
      state.calibrationZeroPending=false;
      const snapshot=await fetch('/api/sensors/state').then(r=>r.json());
      updateCalibrationFromSnapshot(snapshot);
    }catch(err){
      console.error(err);
      state.calibrationRequested=false;
      state.calibrationZeroPending=false;
      const card=$("calibrationToast");
      showCalibrationProgress(0,err.message||"영점 보정을 시작하지 못했습니다.","보정 시작 실패");
      card.classList.add("fail");
      setFlowStage("recommended");
      setTimeout(hideCalibrationToast,3600);
    }
  }

  function updateCalibrationFromSnapshot(payload){
    if(!state.calibrationRequested || state.calibrationZeroPending)return;
    if(payload?.calibration_generation!==state.calibrationGeneration)return;
    const sensors=Array.isArray(payload?.sensors)?payload.sensors:[];
    // Keep missing mappings in the denominator; old completed snapshots cannot finish a new Zero.
    const ids=Array.from({length:state.tagCount},(_,i)=>mappedModule("tag",i+1));
    const tags=ids.map(id=>sensors.find(s=>s.node_id===id));
    const ready=tags.length>0 && tags.every(raw=>raw?.online && raw?.uwb_calibration?.ready);
    const minProgress=tags.length?Math.min(...tags.map(raw=>raw?.uwb_calibration?.ready?1:
      Math.max(0,Math.min(.99,Number(raw?.uwb_calibration?.progress||0))))):0;
    const pct=ready?100:Math.round(minProgress*100);
    const perTag=tags.map((raw,i)=>{
      const name=ids[i]||`Tag ${i+1}`;
      if(!raw?.online)return `${name}: 연결 대기`;
      const cal=raw.uwb_calibration||{};
      if(cal.ready)return `${name}: 완료`;
      const p=Math.round(Math.max(0,Math.min(.99,Number(cal.progress||0)))*100);
      const sample=cal.target_samples?` · 최소 ${cal.min_samples||0}/${cal.target_samples}개`:'';
      const issue=raw.tdma?.status;
      const labels={waiting_slot:'측정 차례 대기',start_ack_missing:'시작 명령 응답 대기',
        no_range:'거리 응답 부족',insufficient_samples:'유효 샘플 부족 · 다음 차례 재시도',firmware_update_required:'태그 펌웨어 업데이트 필요',waiting_ranges:'거리 수신 대기'};
      const reason=(issue && labels[issue]) || (!raw.range_count?'거리 수신 대기':cal.reason||'정지 상태 수집');
      return `${name}: ${p}%${sample} · ${reason}`;
    }).join('\n');
    showCalibrationProgress(pct,perTag,ready?'보정 완료':'영점 보정 중');

    if(ready){
      state.calibrationRequested=false;
      state.monitoringActive=true;
      setFlowStage("monitoring");
      const card=$("calibrationToast");
      card.classList.add("done");
      $("calibrationTitle").textContent="보정 완료";
      $("calibrationPercent").textContent="100%";
      $("calibrationFill").style.width="100%";
      addEvent("영점 보정 완료 · 실시간 모니터링 시작","INFO");
      setTimeout(hideCalibrationToast,1900);
      renderStatsDashboard();
    }
  }

  $("sidebarCloseBtn")?.addEventListener("click",()=>setSidebarCollapsed(true));
  $("sidebarOpenBtn")?.addEventListener("click",()=>setSidebarCollapsed(false));
  $("sidebarLogoBtn")?.addEventListener("click",()=>setWorkspaceView("viewer"));
  $("newTaskNavBtn")?.addEventListener("click",resetToNewTask);
  $("aiResultsNavBtn")?.addEventListener("click",()=>setWorkspaceView("ai"));
  $("statsNavBtn")?.addEventListener("click",()=>setWorkspaceView("stats"));
  $("backToViewerFromAi")?.addEventListener("click",()=>setWorkspaceView("viewer"));
  $("backToViewerFromStats")?.addEventListener("click",()=>setWorkspaceView("viewer"));
  $("editSensorConfigBtn")?.addEventListener("click",()=>{
    if(state.monitoringActive && !confirm("센서 개수를 바꾸면 현재 추천 배치와 모니터링 상태가 초기화됩니다. 계속할까요?"))return;
    if(state.modelGroup){
      state.monitoringActive=false;
      state.calibrationRequested=false;
      setWorkspaceView("viewer");
      setFlowStage("configure");
    }
  });
  $("loadingDetailsToggle")?.addEventListener("click",()=>{
    const card=document.querySelector(".analysisLoadingCard");
    const open=!card.classList.contains("details-open");
    card.classList.toggle("details-open",open);
    $("loadingDetailsToggle").textContent=open?"간단히 보기":"자세히 보기";
  });

  // ---------- UI helpers ----------
  let toastTimer = null;
  function toast(msg) {
    const el = $("toast");
    el.textContent = msg;
    el.classList.add("show");
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => el.classList.remove("show"), 1900);
  }

  function addEvent(text, level = "INFO") {
    const log = $("eventLog");
    const row = document.createElement("div");
    row.className = "eventRow";
    row.innerHTML = `<span>${new Date().toLocaleTimeString()}</span><b>${text}</b><span>${level}</span>`;
    log.prepend(row);
    while (log.children.length > 8) log.removeChild(log.lastChild);
  }

  function setStep(name, status) {
    const row = document.querySelector(`[data-step="${name}"]`);
    if (!row) return;
    row.className = status || "";
    row.querySelector("b").textContent = status === "done" ? "완료" : status === "run" ? "진행" : status === "fail" ? "오류" : "대기";
  }

  function resetPipeline() {
    ["read", "views", "reconstruct", "geometry", "candidates"].forEach(s => setStep(s, ""));
  }

  function setAgent(text, mode = "neutral") {
    $("activityBody").innerHTML = text;
    const badge = $("agentMode");
    badge.className = `badge ${mode}`;
    badge.textContent = mode === "ai" ? "AI + 웹검색" : mode === "demo" ? "데모 AI" : mode === "neutral" ? "대기" : mode.toUpperCase();
  }

  function setAiStep(name, status) {
    const row = document.querySelector(`[data-ai-step="${name}"]`);
    if (!row) return;
    row.className = status || "";
    row.querySelector("b").textContent =
      status === "done" ? "완료" :
      status === "run" ? "진행 중" :
      status === "fail" ? "오류" : "대기";
  }

  function resetAiSteps() {
    ["identify","research","strategy","evaluate","final"].forEach(x => setAiStep(x, ""));
  }

  function finishAiSteps() {
    ["identify","research","strategy","evaluate","final"].forEach(x => setAiStep(x, "done"));
  }

  function setLoadingStep(name, status) {
    const row = document.querySelector(`[data-loading-step="${name}"]`);
    if (!row) return;
    row.className = status || "";
    row.querySelector("b").textContent =
      status === "done" ? "완료" :
      status === "run" ? "진행 중" :
      status === "fail" ? "오류" : "대기";
  }

  function resetLoadingSteps() {
    ["identify","research","strategy","evaluate","final"].forEach(x => setLoadingStep(x, ""));
  }

  function showAnalysisLoading() {
    resetLoadingSteps();
    document.querySelector(".analysisLoadingCard")?.classList.remove("details-open");
    if($("loadingDetailsToggle"))$("loadingDetailsToggle").textContent="자세히 보기";
    $("loadingProgressFill").style.width = "4%";
    $("loadingPercent").textContent = "4%";
    $("loadingTitle").textContent = "구조물을 분석하고 있습니다";
    $("loadingMessage").textContent = "기하 분석 결과를 바탕으로 구조 형식을 확인하고 있습니다.";
    $("analysisLoading").classList.add("show");
    $("analysisLoading").setAttribute("aria-hidden","false");
  }

  function hideAnalysisLoading() {
    $("analysisLoading").classList.remove("show");
    $("analysisLoading").setAttribute("aria-hidden","true");
  }

  function updateAnalysisLoading(step, title, message, percent) {
    const order = ["identify","research","strategy","evaluate","final"];
    const idx = order.indexOf(step);

    order.forEach((name, i) => {
      if (i < idx) setLoadingStep(name, "done");
      else if (i === idx) setLoadingStep(name, "run");
      else setLoadingStep(name, "");
    });

    $("loadingTitle").textContent = title;
    $("loadingMessage").textContent = message;
    $("loadingProgressFill").style.width = `${percent}%`;
    $("loadingPercent").textContent = `${percent}%`;
  }

  function completeAnalysisLoading() {
    ["identify","research","strategy","evaluate","final"].forEach(x => setLoadingStep(x,"done"));
    $("loadingTitle").textContent = "AI 분석이 완료되었습니다";
    $("loadingMessage").textContent = `최종 Tag ${state.tagCount}개와 Anchor ${state.anchorCount}개의 추천 위치를 디지털 트윈에 적용했습니다.`;
    $("loadingProgressFill").style.width = "100%";
    $("loadingPercent").textContent = "100%";

    setTimeout(() => {
      hideAnalysisLoading();
    }, 850);
  }

  function failAnalysisLoading(message) {
    const running = document.querySelector('.loadingSteps>div.run');
    if (running) {
      running.classList.remove("run");
      running.classList.add("fail");
      running.querySelector("b").textContent = "오류";
    }
    $("loadingTitle").textContent = "AI 분석 중 오류가 발생했습니다";
    $("loadingMessage").textContent = message || "잠시 후 다시 시도해 주세요.";
    setTimeout(() => hideAnalysisLoading(), 1500);
  }

  // ---------- Cleanup ----------
  function disposeObject(obj) {
    obj.traverse?.(child => {
      if (child.geometry) child.geometry.dispose?.();
      if (child.material) {
        const mats = Array.isArray(child.material) ? child.material : [child.material];
        mats.forEach(m => m.dispose?.());
      }
    });
    obj.parent?.remove(obj);
  }

  function clearTwin() {
    if (state.modelGroup) disposeObject(state.modelGroup);
    state.modelGroup = null;
    state.deckMesh = null;
    state.deckOriginal = null;
    clearSensors();
    clearCandidates();
    state.modelMeta = null;
    state.profile = null;
    state.surfaceAnalysis = null;
    state.candidates = { tags: [], anchors: [] };
    state.selected = { tags: [], anchors: [] };
    state.aiResult = null;
    $("runAiBtn").disabled = true;
    $("candidateBtn").disabled = true;
    $("simBtn").disabled = true;
    $("zeroBtn").disabled = true;
    $("clearRecBtn").disabled = true;
    $("modelTree").className = "modelTree empty";
    $("modelTree").textContent = "모델을 먼저 불러오세요.";
    if ($("geometrySummary")) $("geometrySummary").textContent = "아직 분석된 모델이 없습니다.";
    if ($("componentList")) $("componentList").innerHTML = "";
    $("structureStat").textContent = "미판단";
    $("tagCandidateStat").textContent = "0";
    $("anchorCandidateStat").textContent = "0";
    renderSensors();
    refreshDynamicCountLabels();
    renderModuleMapping();
  }

  // ---------- DXF parser ----------
  function parseDxf(text, filename) {
    const lines = text.replace(/\r/g, "").split("\n");
    const pairs = [];
    for (let i = 0; i + 1 < lines.length; i += 2) {
      const code = Number(lines[i].trim());
      const value = lines[i + 1].trim();
      if (!Number.isNaN(code)) pairs.push({ code, value });
    }

    let inEntities = false;
    const entities = [];
    let current = null;
    let currentType = null;

    function finish() {
      if (!current) return;
      current.type = currentType;
      entities.push(current);
      current = null;
      currentType = null;
    }

    for (let i = 0; i < pairs.length; i++) {
      const p = pairs[i];
      if (p.code === 0 && p.value === "SECTION") {
        const next = pairs[i + 1];
        if (next && next.code === 2 && next.value === "ENTITIES") inEntities = true;
        continue;
      }
      if (inEntities && p.code === 0 && p.value === "ENDSEC") {
        finish();
        inEntities = false;
        continue;
      }
      if (!inEntities) continue;

      if (p.code === 0) {
        finish();
        currentType = p.value;
        current = { raw: [], layer: "0", vertices: [] };
        continue;
      }
      if (!current) continue;
      current.raw.push(p);
      if (p.code === 8) current.layer = p.value;
    }
    finish();

    const parsed = [];
    for (const e of entities) {
      if (e.type === "LINE") {
        const get = code => {
          const x = e.raw.find(p => p.code === code);
          return x ? Number(x.value) : 0;
        };
        parsed.push({
          type: "LINE", layer: e.layer,
          p1: { x: get(10), y: get(20), z: get(30) },
          p2: { x: get(11), y: get(21), z: get(31) }
        });
      } else if (e.type === "LWPOLYLINE") {
        const verts = [];
        let x = null;
        for (const p of e.raw) {
          if (p.code === 10) x = Number(p.value);
          else if (p.code === 20 && x !== null) {
            verts.push({ x, y: Number(p.value), z: 0 });
            x = null;
          }
        }
        const flag = e.raw.find(p => p.code === 70);
        parsed.push({
          type: "LWPOLYLINE", layer: e.layer, vertices: verts,
          closed: flag ? (Number(flag.value) & 1) === 1 : false
        });
      } else if (e.type === "TEXT" || e.type === "MTEXT") {
        const get = code => e.raw.find(p => p.code === code)?.value;
        parsed.push({
          type: e.type, layer: e.layer,
          text: get(1) || get(3) || "",
          x: Number(get(10) || 0), y: Number(get(20) || 0)
        });
      }
    }

    return { filename, entities: parsed };
  }

  function entityPoints(e) {
    if (e.type === "LINE") return [e.p1, e.p2];
    if (e.type === "LWPOLYLINE") return e.vertices;
    return [];
  }

  function bboxEntities(entities) {
    const pts = entities.flatMap(entityPoints);
    if (!pts.length) return null;
    const xs = pts.map(p => p.x), ys = pts.map(p => p.y);
    return {
      minX: Math.min(...xs), maxX: Math.max(...xs),
      minY: Math.min(...ys), maxY: Math.max(...ys),
      width: Math.max(...xs) - Math.min(...xs),
      height: Math.max(...ys) - Math.min(...ys)
    };
  }

  function layerEntities(dxf, layer) {
    return dxf.entities.filter(e => String(e.layer).toUpperCase() === layer.toUpperCase());
  }

  function rectsForLayer(dxf, layer) {
    return layerEntities(dxf, layer)
      .filter(e => e.type === "LWPOLYLINE" && e.vertices.length >= 4)
      .map(e => {
        const xs = e.vertices.map(v => v.x), ys = e.vertices.map(v => v.y);
        return {
          minX: Math.min(...xs), maxX: Math.max(...xs),
          minY: Math.min(...ys), maxY: Math.max(...ys),
          cx: (Math.min(...xs) + Math.max(...xs)) / 2,
          cy: (Math.min(...ys) + Math.max(...ys)) / 2,
          w: Math.max(...xs) - Math.min(...xs),
          h: Math.max(...ys) - Math.min(...ys)
        };
      });
  }

  function classifyDxfView(dxf) {
    const deck = rectsForLayer(dxf, "DECK")[0];
    const all = bboxEntities(dxf.entities);
    if (!deck) return "unknown";
    // Plan has wide transverse deck dimension; elevation is long + thin; section is short + thin.
    if (deck.w > 10000 && deck.h > 1500) return "plan";
    if (deck.w > 10000 && deck.h <= 1200) return "elevation";
    if (deck.w < 10000 && deck.h <= 1200) return "section";
    if (all && all.width > all.height * 3) return "elevation";
    return "unknown";
  }

  function createMaterial(color, roughness = .82, metalness = .02) {
    return new THREE.MeshStandardMaterial({ color, roughness, metalness });
  }

  function boxMesh(group, name, sx, sy, sz, x, y, z, color, segmentsX = 1, segmentsZ = 1) {
    const geo = new THREE.BoxGeometry(sx, sy, sz, segmentsX, 2, segmentsZ);
    const mesh = new THREE.Mesh(geo, createMaterial(color));
    mesh.name = name;
    mesh.position.set(x, y, z);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    group.add(mesh);
    return mesh;
  }

  function reconstructFromDxf(dxfs) {
    const byView = {};
    dxfs.forEach(d => byView[classifyDxfView(d)] = d);
    if (!byView.plan || !byView.elevation || !byView.section) {
      throw new Error("평면도·입면도·단면도를 모두 식별하지 못했습니다. DXF layer와 좌표를 확인하세요.");
    }

    const plan = byView.plan, elev = byView.elevation, section = byView.section;
    const planDeck = rectsForLayer(plan, "DECK")[0];
    const elevDeck = rectsForLayer(elev, "DECK")[0];
    const secGirders = rectsForLayer(section, "GIRDER");
    const planColumns = rectsForLayer(plan, "COLUMN");
    const planFootings = rectsForLayer(plan, "FOOTING");
    const planCaps = rectsForLayer(plan, "PIER_CAP");
    const elevColumn = rectsForLayer(elev, "COLUMN")[0];
    const elevFooting = rectsForLayer(elev, "FOOTING")[0];
    const elevCap = rectsForLayer(elev, "PIER_CAP")[0];

    if (!planDeck || !elevDeck || !secGirders.length) {
      throw new Error("DECK/GIRDER 핵심 layer 정보를 충분히 찾지 못했습니다.");
    }

    const mm = 0.001;
    const length = planDeck.w * mm;
    const width = planDeck.h * mm;
    const deckThickness = elevDeck.h * mm;
    const deckBottom = elevDeck.minY * mm;
    const deckTop = elevDeck.maxY * mm;
    const xCenterMm = (planDeck.minX + planDeck.maxX) / 2;

    const group = new THREE.Group();
    group.name = "Reconstructed_Viaduct";

    const deck = boxMesh(
      group, "DECK",
      length, deckThickness, width,
      0, (deckBottom + deckTop) / 2, 0,
      0x8b969e, 120, 24
    );

    const girders = [];
    for (let i = 0; i < secGirders.length; i++) {
      const r = secGirders[i];
      const g = boxMesh(
        group, `GIRDER_${i + 1}`,
        length, r.h * mm, r.w * mm,
        0, (r.minY + r.maxY) * .5 * mm, r.cx * mm,
        0x5b6871, 40, 2
      );
      girders.push(g);
    }

    const columns = [];
    const columnHeight = elevColumn ? elevColumn.h * mm : 2.2;
    const columnBottom = elevColumn ? elevColumn.minY * mm : .3;
    for (let i = 0; i < planColumns.length; i++) {
      const r = planColumns[i];
      columns.push(boxMesh(
        group, `COLUMN_${i + 1}`,
        r.w * mm, columnHeight, r.h * mm,
        (r.cx - xCenterMm) * mm,
        columnBottom + columnHeight / 2,
        r.cy * mm,
        0x747f87
      ));
    }

    const footings = [];
    const footingHeight = elevFooting ? elevFooting.h * mm : .3;
    for (let i = 0; i < planFootings.length; i++) {
      const r = planFootings[i];
      footings.push(boxMesh(
        group, `FOOTING_${i + 1}`,
        r.w * mm, footingHeight, r.h * mm,
        (r.cx - xCenterMm) * mm,
        footingHeight / 2,
        r.cy * mm,
        0x879198
      ));
    }

    const caps = [];
    const capHeight = elevCap ? elevCap.h * mm : .5;
    const capBottom = elevCap ? elevCap.minY * mm : 2.5;
    for (let i = 0; i < planCaps.length; i++) {
      const r = planCaps[i];
      caps.push(boxMesh(
        group, `PIER_CAP_${i + 1}`,
        r.w * mm, capHeight, r.h * mm,
        (r.cx - xCenterMm) * mm,
        capBottom + capHeight / 2,
        r.cy * mm,
        0x6c777f
      ));
    }

    // Simple guardrails generated from deck edges.
    boxMesh(group, "GUARDRAIL_L", length, .35, .08, 0, deckTop + .175, -width / 2 + .05, 0x5a656c, 40, 1);
    boxMesh(group, "GUARDRAIL_R", length, .35, .08, 0, deckTop + .175, width / 2 - .05, 0x5a656c, 40, 1);

    const pierXs = [...new Set(planCaps.map(r => Number(((r.cx - xCenterMm) * mm).toFixed(4))))].sort((a,b)=>a-b);

    return {
      group,
      deck,
      exactUnits: true,
      meta: {
        source: "DXF 도면 세트",
        reconstruction: "plan + elevation + section",
        unitsToMeters: 1,
        length, width, deckThickness, deckTop,
        girderCount: secGirders.length,
        columnCount: planColumns.length,
        footingCount: planFootings.length,
        pierCapCount: planCaps.length,
        pierXs,
        supportXs: pierXs,
        componentCounts: {
          Deck: 1,
          Girder: secGirders.length,
          Column: planColumns.length,
          Footing: planFootings.length,
          "Pier cap": planCaps.length,
          Guardrail: 2
        },
        geometryHypothesis: {
          category: "bridge",
          type: "multi_span_girder_viaduct",
          confidence: .96
        },
        uncertainties: [
          "Material properties are not encoded in the geometry reconstruction.",
          "Connection/reinforcement details require structural drawings or BIM metadata."
        ]
      }
    };
  }

  // ---------- 3D file loaders ----------
  async function loadGlb(file) {
    const buffer = await file.arrayBuffer();
    return await new Promise((resolve, reject) => {
      const loader = new THREE.GLTFLoader();
      loader.parse(buffer, "", gltf => resolve(gltf.scene), reject);
    });
  }

  async function loadObj(file) {
    const text = await file.text();
    return new THREE.OBJLoader().parse(text);
  }

  function prepareImportedGroup(group, sourceType) {
    let bbox = new THREE.Box3().setFromObject(group);
    let size = bbox.getSize(new THREE.Vector3());

    if (sourceType === "obj") {
      // OBJ has no standard unit. Keep previous visualization normalization.
      const maxDim = Math.max(size.x,size.y,size.z) || 1;
      const scale = 12 / maxDim;
      group.scale.setScalar(scale);
      bbox = new THREE.Box3().setFromObject(group);
      size = bbox.getSize(new THREE.Vector3());
    }

    // Keep original physical GLB scale, but center the model in X/Z and
    // place its lowest point on y=0.
    const c = bbox.getCenter(new THREE.Vector3());
    group.position.x -= c.x;
    group.position.z -= c.z;
    bbox = new THREE.Box3().setFromObject(group);
    group.position.y -= bbox.min.y;

    group.traverse(ch => {
      if (!ch.isMesh) return;
      ch.castShadow = true;
      ch.receiveShadow = true;
      const mats=Array.isArray(ch.material)?ch.material:[ch.material].filter(Boolean);
      if(!mats.length){ch.material=createMaterial(0x87929a);ch.material.side=THREE.DoubleSide;return;}
      if(!Array.isArray(ch.material)&&!ch.material.isMeshStandardMaterial)ch.material=createMaterial(0x87929a);
      const finalMats=Array.isArray(ch.material)?ch.material:[ch.material];
      finalMats.forEach(m=>{if(m)m.side=THREE.DoubleSide;});
    });

    return group;
  }

  function detectDeckMesh(group) {
    let named = null;
    group.traverse(ch => {
      if (ch.isMesh && /deck|slab|bridge.?deck/i.test(ch.name || "")) named = named || ch;
    });
    if (named) return named;

    const candidates = [];
    group.updateMatrixWorld(true);
    group.traverse(ch => {
      if (!ch.isMesh || !ch.geometry) return;
      const b = new THREE.Box3().setFromObject(ch);
      const s = b.getSize(new THREE.Vector3());
      const c = b.getCenter(new THREE.Vector3());
      const horiz = Math.max(s.x, s.z);
      const aspect = horiz / Math.max(s.y, .001);
      candidates.push({ mesh: ch, aspect, top: c.y, area: s.x * s.z });
    });
    candidates.sort((a,b) => (b.aspect * b.area) - (a.aspect * a.area));
    return candidates[0]?.mesh || null;
  }

  // ---------- Geometry analysis ----------
  function countComponents(group) {
    const counts = {};
    group.traverse(ch => {
      if (!ch.isMesh) return;
      const n = (ch.name || "Mesh").toUpperCase();
      let key = "Mesh";
      if (n.includes("DECK") || n.includes("SLAB") || n.includes("ROADWAY") || n.includes("SUPERSTRUCTURE")) key = "Deck/Slab";
      else if (n.includes("GIRDER") || n.includes("BEAM") || n.includes("SPAN")) key = "Girder/Beam";
      else if (n.includes("COLUMN") || n.includes("PIER") || n.includes("PEDESTAL") || n.includes("ABUTMENT") || n.includes("SUPPORT")) key = "Column/Pier";
      else if (n.includes("TUNNEL") || n.includes("LINING") || n.includes("CROWN")) key = "Tunnel/Lining";
      else if (n.includes("WALL")) key = "Wall";
      else if (n.includes("FOOT")) key = "Footing";
      else if (n.includes("CAP")) key = "Pier cap";
      counts[key] = (counts[key] || 0) + 1;
    });
    return counts;
  }

  function analyzeGeneric(group, sourceLabel, sourceType = "obj") {
    group.updateMatrixWorld(true);
    const bbox = new THREE.Box3().setFromObject(group);
    const size = bbox.getSize(new THREE.Vector3());
    const long = Math.max(size.x, size.z);
    const short = Math.min(size.x, size.z);
    const bridgeLike = long > short * 2.2 && long > size.y * 2.3;
    const componentCounts = countComponents(group);
    const namedBridge=Boolean(componentCounts["Deck/Slab"]||componentCounts["Girder/Beam"]||componentCounts["Column/Pier"]);
    const namedTunnel=Boolean(componentCounts["Tunnel/Lining"]);

    return {
      source: sourceLabel,
      reconstruction: "직접 입력 3D 모델",
      exactUnits: sourceType === "glb",
      unitsToMeters: sourceType === "glb" ? 1 : null,
      bbox: { x: size.x, y: size.y, z: size.z },
      length: long,
      width: short,
      deckTop: bbox.max.y,
      componentCounts,
      supportXs: [],
      geometryHypothesis: {
        category: namedTunnel?"tunnel":(namedBridge?"bridge":"structure"),
        type: namedTunnel?"tunnel_or_lining_structure":(namedBridge&&bridgeLike?"elongated_bridge_or_viaduct":"unclassified_structure"),
        confidence: namedTunnel?.82:(namedBridge?.78:.42)
      },
      uncertainties: [
        sourceType === "glb"
          ? "GLB/glTF의 선형 단위를 metre로 사용하여 센서 케이스 실제 크기와 배치 여유를 계산했습니다."
          : "OBJ는 파일 자체에 단위 정의가 없어 케이스 치수 제약은 상대 크기로 적용됩니다.",
        "Structure type is only a geometry-tool hypothesis until the AI independently verifies it."
      ]
    };
  }

  function modelBoundingBox() {
    if (!state.modelGroup) return new THREE.Box3();
    state.modelGroup.updateMatrixWorld(true);
    return new THREE.Box3().setFromObject(state.modelGroup);
  }

  function surfaceClassFromNormal(n){
    if(n.y>.68)return "upper";
    if(n.y<-.68)return "ceiling";
    if(Math.abs(n.y)<.30)return "wall";
    return "inclined";
  }

  function surfaceBasis(normal){
    const n=normal.clone().normalize();
    const ref=Math.abs(n.y)<.88?new THREE.Vector3(0,1,0):new THREE.Vector3(1,0,0);
    const u=ref.clone().cross(n).normalize();
    const v=n.clone().cross(u).normalize();
    return {u,v};
  }

  function rayClearance(point,dir,maxDim){
    const d=dir.clone().normalize();
    const eps=Math.max(maxDim*.0005,.003);
    const ray=state.topSurfaceRaycaster;
    ray.set(point.clone().addScaledVector(d,eps),d);
    ray.near=eps*.2;ray.far=maxDim*3+2;
    const hits=ray.intersectObject(state.modelGroup,true);
    return hits.length?hits[0].distance:maxDim*3+2;
  }

  function accessibleNormal(point,rawNormal,maxDim){
    const n=rawNormal.clone().normalize();
    const plus=rayClearance(point,n,maxDim);
    const minus=rayClearance(point,n.clone().multiplyScalar(-1),maxDim);
    const blocked=Math.max(maxDim*.035,.12);

    // If one side is physically blocked and the other is open, that determines the
    // installation side unambiguously.  If both sides are open (very common with
    // zero-thickness CAD/tunnel surfaces), do not trust triangle winding: mark it as
    // ambiguous so a structure-aware canonical direction can be chosen below.
    if(plus<blocked && minus>Math.max(plus*1.6,blocked*1.8))
      return {normal:n.clone().multiplyScalar(-1),clearance:minus,ambiguous:false};
    if(minus<blocked && plus>Math.max(minus*1.6,blocked*1.8))
      return {normal:n,clearance:plus,ambiguous:false};
    return {normal:n,clearance:Math.max(plus,minus),ambiguous:true};
  }

  function canonicalInstallationNormal(point,rawNormal,analysis){
    const n=rawNormal.clone().normalize();
    const center=analysis?.center || modelBoundingBox().getCenter(new THREE.Vector3());
    const category=String(state.profile?.geometryHypothesis?.category||"").toLowerCase();
    const towardCenter=center.clone().sub(point);
    const awayFromCenter=point.clone().sub(center);

    // Tunnel/lining sensors are normally attached to the inner lining surface, so
    // their usable free-space normal points into the tunnel cavity (toward the model
    // centre).  Ordinary exterior structures use the opposite convention.  This
    // removes random sign flips caused by inconsistent GLB triangle winding.
    const target=category.includes("tunnel")?towardCenter:awayFromCenter;
    if(target.lengthSq()>1e-10 && n.dot(target)<0)n.multiplyScalar(-1);
    return n;
  }

  function analyzeInstallSurfaces(){
    const bbox=modelBoundingBox();
    const size=bbox.getSize(new THREE.Vector3());
    const center=bbox.getCenter(new THREE.Vector3());
    const maxDim=Math.max(size.x,size.y,size.z,1);
    const samples=[];
    const summary={upper:0,ceiling:0,wall:0,inclined:0};

    state.modelGroup.updateMatrixWorld(true);
    const meshes=[];
    state.modelGroup.traverse(mesh=>{if(mesh.isMesh&&mesh.geometry?.attributes?.position)meshes.push(mesh);});
    const budgetPerMesh=Math.max(80,Math.floor(9000/Math.max(meshes.length,1)));
    for(const mesh of meshes){
      const geom=mesh.geometry,pos=geom.attributes.position,index=geom.index;
      const meshBox=new THREE.Box3().setFromObject(mesh);
      const meshSize=meshBox.getSize(new THREE.Vector3());
      const triCount=index?Math.floor(index.count/3):Math.floor(pos.count/3);
      const stride=Math.max(1,Math.floor(triCount/budgetPerMesh));
      const getVertex=vi=>new THREE.Vector3().fromBufferAttribute(pos,vi).applyMatrix4(mesh.matrixWorld);
      for(let t=0;t<triCount;t+=stride){
        const ia=index?index.getX(t*3):t*3;
        const ib=index?index.getX(t*3+1):t*3+1;
        const ic=index?index.getX(t*3+2):t*3+2;
        const a=getVertex(ia),b=getVertex(ib),c=getVertex(ic);
        const cross=b.clone().sub(a).cross(c.clone().sub(a));
        const area=cross.length()*.5;
        if(area<1e-9)continue;
        const normal=cross.normalize();
        const centroid=a.clone().add(b).add(c).multiplyScalar(1/3);
        const cls=surfaceClassFromNormal(normal);
        const basis=surfaceBasis(normal);
        summary[cls]+=area;
        samples.push({
          a,b,c,centroid,area,normal,uAxis:basis.u,vAxis:basis.v,
          surfaceClass:cls,meshUuid:mesh.uuid,meshName:mesh.name||"mesh",
          meshSize:{x:meshSize.x,y:meshSize.y,z:meshSize.z}
        });
      }
    }
    const dominant=Object.entries(summary).sort((a,b)=>b[1]-a[1])[0]?.[0]||"unknown";
    const analysis={bbox,size,center,maxDim,samples,summary,dominant};
    state.surfaceAnalysis=analysis;
    return analysis;
  }

  // continuous deformer.
  // CAD/GLB assemblies are commonly split into many independent meshes. The previous
  // implementation limited deformation to the exact mesh that owned the Tag candidate,
  // making only one bridge part move. Every mesh now shares one world-space deformation
  // field so neighbouring components follow the same continuous sag/tilt curve.
  function prepareDeckDeformer(deck) {
    state.deckMesh=deck||null;
    state.deckOriginal=null;
    state.deformTargets=[];
    if(!state.modelGroup)return;
    state.modelGroup.updateMatrixWorld(true);
    state.modelGroup.traverse(mesh=>{
      if(!mesh.isMesh||!mesh.geometry?.attributes?.position)return;
      mesh.geometry=mesh.geometry.clone();
      const wb=new THREE.Box3().setFromObject(mesh);
      state.deformTargets.push({
        mesh,mode:"continuousSurfaceVertex",sourceMeshUuid:mesh.uuid,
        original:mesh.geometry.attributes.position.array.slice(),
        basePosition:mesh.position.clone(),baseQuaternion:mesh.quaternion.clone(),
        worldBox:wb.clone(),
        worldCenter:wb.getCenter(new THREE.Vector3()),
        worldSize:wb.getSize(new THREE.Vector3())
      });
    });
  }

  function samplePointOnTriangle(a,b,c,w){
    return a.clone().multiplyScalar(w[0]).add(b.clone().multiplyScalar(w[1])).add(c.clone().multiplyScalar(w[2]));
  }

  function hitSurfaceNear(point,normal,maxDim){
    const n=normal.clone().normalize();
    const probe=Math.max(maxDim*.012,.06);
    const ray=state.topSurfaceRaycaster;
    ray.set(point.clone().addScaledVector(n,probe),n.clone().multiplyScalar(-1));
    ray.near=0;ray.far=probe*2.5;
    const hits=ray.intersectObject(state.modelGroup,true);
    if(!hits.length)return null;
    const h=hits[0];
    let hn=n.clone();
    if(h.face)hn=h.face.normal.clone().transformDirection(h.object.matrixWorld).normalize();
    if(hn.dot(n)<0)hn.multiplyScalar(-1);
    return {point:h.point.clone(),normal:hn,object:h.object,distance:h.distance};
  }

  function footprintFitsSurface(surfacePoint,normal,uAxis,vAxis,maxDim,orientation=0){
    const cm=caseMetricsInModelUnits();
    const ca=Math.cos(orientation),sa=Math.sin(orientation);
    const u=uAxis.clone().multiplyScalar(ca).add(vAxis.clone().multiplyScalar(sa)).normalize();
    const v=normal.clone().cross(u).normalize();
    const hu=cm.width/2+cm.clearance,hv=cm.depth/2+cm.clearance;
    const grid=[-1,0,1];
    const tol=Math.max(cm.height*.55,Math.min(cm.width,cm.depth)*.22,maxDim*.004,.025);
    for(const gu of grid){
      for(const gv of grid){
        const q=surfacePoint.clone().addScaledVector(u,gu*hu).addScaledVector(v,gv*hv);
        const hit=hitSurfaceNear(q,normal,maxDim);
        if(!hit)return null;
        if(hit.normal.dot(normal)<.58)return null;
        if(hit.point.distanceTo(q)>tol)return null;
      }
    }
    return {u,v};
  }

  function bridgeCandidateStructuralMeta(smp,surfaceClass,p,n,analysis,uNorm){
    const category=String(state.profile?.geometryHypothesis?.category||"").toLowerCase();
    const type=String(state.profile?.geometryHypothesis?.type||"").toLowerCase();
    const isBridge=category.includes("bridge")||/bridge|viaduct|girder/.test(type);
    if(!isBridge)return {structuralRole:"generic_surface",bridgePriorityScore:0,meshLongRatio:0};

    const name=String(smp.meshName||"").toLowerCase();
    const supportNamed=/pier|column|pedestal|abutment|support|bearing|foot|foundation|cap/.test(name);
    const superNamed=/deck|slab|girder|beam|span|roadway|superstructure/.test(name);
    const ms=smp.meshSize||{x:0,y:0,z:0};
    const longAxisX=analysis.size.x>=analysis.size.z;
    const modelLong=Math.max(longAxisX?analysis.size.x:analysis.size.z,.001);
    const meshLong=Math.max(0,longAxisX?Number(ms.x||0):Number(ms.z||0));
    const meshLongRatio=meshLong/modelLong;
    const heightRatio=(p.y-analysis.bbox.min.y)/Math.max(analysis.size.y,.001);
    const upper=surfaceClass==="upper"&&n.y>.55;
    const endZone=uNorm<.07||uNorm>.93;

    let structuralRole="secondary_structure";
    let score=0;
    if(supportNamed){
      structuralRole="support_reference";
      score=-3.0;
    }else if(superNamed&&upper){
      structuralRole="primary_superstructure";
      score=3.2;
    }else if(upper&&meshLongRatio>=.16&&heightRatio>=.48){
      structuralRole="primary_superstructure";
      score=2.2;
    }else if(upper&&heightRatio>=.55){
      structuralRole="secondary_superstructure";
      score=1.0;
    }else if(surfaceClass==="wall"&&Number(ms.y||0)>Math.max(meshLong*.55,.04)){
      structuralRole="support_or_vertical";
      score=-1.4;
    }
    score+=Math.min(1,meshLongRatio)*.8;
    if(upper)score+=.45;
    if(endZone&&!superNamed)score-=.35;
    return {structuralRole,bridgePriorityScore:score,meshLongRatio};
  }

  function currentAnchorLimits(){
    return ModelAnchorLimits.forFiles(state.files.map(f=>f.name));
  }

  function generateCandidates() {
    const tags=[];
    const anchors=[];
    const analysis=state.surfaceAnalysis||analyzeInstallSurfaces();
    const {bbox,size,center,maxDim}=analysis;
    const cm=caseMetricsInModelUnits();
    const spacing=Math.max(Math.min(cm.width,cm.depth)*.78,maxDim*.006,.035);
    const accepted=[];
    const seeds=[
      [1/3,1/3,1/3],[.56,.22,.22],[.22,.56,.22],[.22,.22,.56],
      [.68,.16,.16],[.16,.68,.16],[.16,.16,.68]
    ];

    // Prefer large patches first. Expensive free-space ray checks are done lazily
    // only for samples that are actually considered for Tag placement.
    const ordered=[...analysis.samples].sort((a,b)=>b.area-a.area);
    let sampleAttempts=0,footprintAttempts=0;

    for(const smp of ordered){
      if(++sampleAttempts>1800)break;
      const access=accessibleNormal(smp.centroid,smp.normal,maxDim);
      let n=access.normal;
      if(access.ambiguous)n=canonicalInstallationNormal(smp.centroid,n,analysis);
      const usableClearance=rayClearance(smp.centroid,n,maxDim);
      if(usableClearance<Math.max(cm.height*1.1,.03))continue;
      const basis=surfaceBasis(n);
      const surfaceClass=surfaceClassFromNormal(n);
      const localCount=Math.max(1,Math.min(seeds.length,Math.ceil(smp.area/Math.max(spacing*spacing*.45,1e-6))));
      for(let si=0;si<localCount;si++){
        if(++footprintAttempts>3600)break;
        const p=samplePointOnTriangle(smp.a,smp.b,smp.c,seeds[si]);
        if(accepted.some(q=>q.surfacePoint.distanceTo(p)<spacing))continue;

        let fit=footprintFitsSurface(p,n,basis.u,basis.v,maxDim,0);
        let orientation=0;
        if(!fit){fit=footprintFitsSurface(p,n,basis.u,basis.v,maxDim,Math.PI/2);orientation=Math.PI/2;}
        if(!fit)continue;

        const offset=cm.height/2+.002;
        const centerPoint=p.clone().addScaledVector(n,offset);
        const bboxAxis=size.x>=size.y&&size.x>=size.z?'x':(size.z>=size.y?'z':'y');
        const coord=bboxAxis==='x'?p.x:(bboxAxis==='z'?p.z:p.y);
        const min=bboxAxis==='x'?bbox.min.x:(bboxAxis==='z'?bbox.min.z:bbox.min.y);
        const span=bboxAxis==='x'?size.x:(bboxAxis==='z'?size.z:size.y);
        const uNorm=(coord-min)/Math.max(span,.001);
        const normalBin=`${Math.round(n.x*3)}_${Math.round(n.y*3)}_${Math.round(n.z*3)}`;
        const patchId=`${smp.meshName||'mesh'}:${surfaceClass}:${normalBin}`;

        const structureMeta=bridgeCandidateStructuralMeta(smp,surfaceClass,p,n,analysis,uNorm);
        const item={
          id:`T${String(tags.length+1).padStart(3,"0")}`,
          x:centerPoint.x,y:centerPoint.y,z:centerPoint.z,
          surfaceX:p.x,surfaceY:p.y,surfaceZ:p.z,
          nx:n.x,ny:n.y,nz:n.z,
          tux:fit.u.x,tuy:fit.u.y,tuz:fit.u.z,
          tvx:fit.v.x,tvy:fit.v.y,tvz:fit.v.z,
          sourceMeshUuid:smp.meshUuid,
          sourceMeshName:smp.meshName,
          structuralRole:structureMeta.structuralRole,
          bridgePriorityScore:structureMeta.bridgePriorityScore,
          meshLongRatio:structureMeta.meshLongRatio,
          surfacePatch:patchId,
          surfaceClass,
          surface:surfaceClass,
          accessibility_m:usableClearance,
          caseOrientation:orientation,
          caseWidth:cm.width,caseDepth:cm.depth,caseHeight:cm.height,
          u:uNorm,
          v:0,
          zone:surfaceClass,
          supportDistance:null
        };
        tags.push(item);
        accepted.push({surfacePoint:p,candidate:item});
        if(tags.length>=480)break;
      }
      if(tags.length>=480||footprintAttempts>3600)break;
    }

    if(tags.length>240){
      const groups=new Map();
      for(const t of tags){
        const key=t.surfacePatch||t.surfaceClass||"surface";
        if(!groups.has(key))groups.set(key,[]);
        groups.get(key).push(t);
      }
      const balanced=[];
      const queues=[...groups.values()].sort((a,b)=>b.length-a.length);
      let progress=true;
      while(balanced.length<240&&progress){
        progress=false;
        for(const q of queues){
          if(q.length&&balanced.length<240){balanced.push(q.shift());progress=true;}
        }
      }
      tags.splice(0,tags.length,...balanced);
      tags.forEach((t,i)=>t.id=`T${String(i+1).padStart(3,"0")}`);
    }

    // Anchors remain independent fixed references. They can be placed at multiple
    // heights up to 2x model height. This is intentionally independent of Tag surface
    // orientation so bridges, walls and tunnel interiors all use the same UWB solver.
    const neededPerimeterPoints=Math.max(12,state.anchorCount*3);
    const marginX=Math.max(cm.width*.8,size.x*.12,.6);
    const marginZ=Math.max(cm.depth*.8,size.z*.18,.6);
    const groundY=Math.max(0,bbox.min.y);
    const anchorMinY=groundY+cm.height/2+.002;
    const modelHeight=Math.max(size.y,.05);
    const limits=currentAnchorLimits();
    const defaultAnchorMaxY=Math.max(anchorMinY,groundY+modelHeight*2.0);
    // Model-specific hard ceiling. For assembly_model_raised_35mm.glb the user
    // requested an absolute 60 cm maximum recommendation height.
    const anchorMaxY=Number.isFinite(limits?.maxY)?Math.max(anchorMinY,limits.maxY):defaultAnchorMaxY;
    const heightRatios=[0,.4,.8,1.2,1.6,2.0];
    const heightLevels=heightRatios.map(r=>({ratio:r,y:r===0?anchorMinY:Math.min(anchorMaxY,groundY+modelHeight*r)}))
      .filter((v,i,a)=>i===0||Math.abs(v.y-a[i-1].y)>1e-5);
    if(Number.isFinite(limits?.maxY)&&limits.maxY>anchorMinY&&
       !heightLevels.some(v=>Math.abs(v.y-limits.maxY)<1e-5)){
      heightLevels.push({ratio:(limits.maxY-groundY)/Math.max(modelHeight,1e-6),y:limits.maxY});
    }
    const pMinX=limits?limits.minX:bbox.min.x-marginX,pMaxX=limits?limits.maxX:bbox.max.x+marginX;
    const pMinZ=limits?limits.minZ:bbox.min.z-marginZ,pMaxZ=limits?limits.maxZ:bbox.max.z+marginZ;
    const perSide=Math.max(4,Math.ceil(neededPerimeterPoints/4));
    const perimeter=[];
    for(let i=0;i<perSide;i++){
      const t=i/Math.max(1,perSide-1);
      perimeter.push([pMinX+(pMaxX-pMinX)*t,pMinZ,"N"]);
      perimeter.push([pMinX+(pMaxX-pMinX)*t,pMaxZ,"S"]);
      perimeter.push([pMinX,pMinZ+(pMaxZ-pMinZ)*t,"W"]);
      perimeter.push([pMaxX,pMinZ+(pMaxZ-pMinZ)*t,"E"]);
    }
    const uniquePerimeter=[],seen=new Set();
    for(const q of perimeter){const k=`${q[0].toFixed(4)}:${q[1].toFixed(4)}`;if(seen.has(k))continue;seen.add(k);uniquePerimeter.push(q);}
    for(const level of heightLevels){
      for(const q of uniquePerimeter){
        anchors.push({
          id:`A${String(anchors.length+1).padStart(3,"0")}`,
          x:q[0],y:level.y,z:q[1],surfaceY:groundY,groundY,
          caseWidth:cm.width,caseDepth:cm.depth,caseHeight:cm.height,
          quadrant:q[2],heightRatio:level.ratio,heightAboveGround:Math.max(0,level.y-groundY),maxAllowedY:anchorMaxY,
          mount:"external_fixed_support"
        });
      }
    }

    // Cavity / interior support candidates. A tunnel's useful UWB references must
    // often sit inside the free tunnel volume, not outside the model bounding box.
    // Generate free-space stand points along exposed Surface normals while respecting
    // the same maximum Y = ground + 2*modelHeight rule.
    const anchorSeen=new Set(anchors.map(a=>`${a.x.toFixed(3)}:${a.y.toFixed(3)}:${a.z.toFixed(3)}`));
    const tagStride=Math.max(1,Math.floor(tags.length/28));
    const normalDistances=[Math.max(maxDim*.08,.35),Math.max(maxDim*.18,.65),Math.max(maxDim*.30,1.0)];
    for(let ti=0;ti<tags.length;ti+=tagStride){
      const t=tags[ti];
      const p0=new THREE.Vector3(t.surfaceX,t.surfaceY,t.surfaceZ);
      const n=new THREE.Vector3(t.nx,t.ny,t.nz).normalize();
      const free=rayClearance(p0,n,maxDim);
      for(const dist of normalDistances){
        if(free<dist+Math.max(cm.width,cm.depth)*.8)continue;
        const q=p0.clone().addScaledVector(n,dist);
        if(q.y<anchorMinY||q.y>anchorMaxY)continue;
        if(!ModelAnchorLimits.contains(q,limits))continue;
        const key=`${q.x.toFixed(3)}:${q.y.toFixed(3)}:${q.z.toFixed(3)}`;
        if(anchorSeen.has(key))continue;anchorSeen.add(key);
        anchors.push({
          id:`A${String(anchors.length+1).padStart(3,"0")}`,
          x:q.x,y:q.y,z:q.z,surfaceY:groundY,groundY,
          caseWidth:cm.width,caseDepth:cm.depth,caseHeight:cm.height,
          quadrant:`FREE_${t.surfaceClass||"SURFACE"}`,
          heightRatio:(q.y-groundY)/Math.max(modelHeight,1e-6),
          heightAboveGround:Math.max(0,q.y-groundY),maxAllowedY:anchorMaxY,
          mount:"free_space_fixed_stand",sourceSurface:t.surfaceClass||"unknown"
        });
      }
    }

    state.candidates={tags,anchors};
    state.deformationBand={bbox,size,center,maxDim,surfaceMode:true};
    $("tagCandidateStat").textContent=tags.length;
    $("anchorCandidateStat").textContent=anchors.length;
    $("candidateBtn").disabled=!tags.length;

    const s=analysis.summary;
    const bridgePrimary=tags.filter(t=>t.structuralRole==="primary_superstructure").length;
    const bridgeNote=bridgePrimary?` · 교량 주 변형부 우선 후보 ${bridgePrimary}개`:"";
    addEvent(`Surface+Normal 분석 완료 · Tag 후보 ${tags.length}개 (상부 ${Math.round(s.upper)} / 천장 ${Math.round(s.ceiling)} / 벽 ${Math.round(s.wall)} / 경사 ${Math.round(s.inclined)} 면적가중)${bridgeNote} · Anchor ${anchors.length}개`,tags.length>=state.tagCount?"INFO":"오류");
    if(tags.length<state.tagCount)addEvent(`센서 케이스가 실제로 놓일 수 있는 표면 후보가 부족합니다 (${tags.length}/${state.tagCount})`,`오류`);
  }

  function buildProfile() {
    const m = state.modelMeta || {};
    if(!state.surfaceAnalysis)analyzeInstallSurfaces();
    const bbox = modelBoundingBox();
    const size = bbox.getSize(new THREE.Vector3());
    const profile = {
      input_source: m.source || state.sourceType,
      reconstruction_method: m.reconstruction || "unknown",
      dimensions_m: {
        x: Number(size.x.toFixed(3)),
        y: Number(size.y.toFixed(3)),
        z: Number(size.z.toFixed(3))
      },
      known_components: m.componentCounts || countComponents(state.modelGroup),
      geometryHypothesis: m.geometryHypothesis || {
        category:"structure",type:"unknown",confidence:.3
      },
      exactUnits: Boolean(m.exactUnits),
      supports_longitudinal_m: m.supportXs || [],
      install_surface_summary: state.surfaceAnalysis ? {
        dominant: state.surfaceAnalysis.dominant,
        weighted_area: Object.fromEntries(Object.entries(state.surfaceAnalysis.summary||{}).map(([k,v])=>[k,Number(v.toFixed(3))])),
        candidate_mode: "surface_patch_plus_normal"
      } : null,
      uncertainties: m.uncertainties || []
    };
    state.profile = profile;
    return profile;
  }

  function renderGeometryAnalysis() {
    const p = state.profile;
    if (!p) return;
    const surf=p.install_surface_summary;
    const surfText=surf?`<br><span style="color:#637983">설치면 분석 · ${surf.dominant} 우세 · Surface + Normal 후보 방식</span>`:"";
    $("geometrySummary").innerHTML = `
      <b>Geometry Tool 가설</b><br>
      ${p.geometryHypothesis.type} · confidence ${(p.geometryHypothesis.confidence*100).toFixed(0)}%${surfText}<br>
      <span style="color:#8998a0">AI는 이 기하 분석 결과를 참고하되 그대로 따르지 않고, 웹 검색을 통해 구조 형식과 관련 근거를 독립적으로 확인합니다.</span>
    `;
    const list = $("componentList");
    list.innerHTML = "";
    Object.entries(p.known_components || {}).forEach(([k,v])=>{
      const row=document.createElement("div");row.className="componentRow";
      row.innerHTML=`<span>${k}</span><b>${v}</b>`;list.appendChild(row);
    });
    $("structureStat").textContent = p.geometryHypothesis.type.replaceAll("_"," ");
  }

  function renderTree() {
    const counts = state.profile?.known_components || {};
    const el = $("modelTree");
    el.className = "modelTree";
    el.innerHTML = `<div class="treeItem"><b>${state.modelGroup?.name || "Structure"}</b><span>${Object.values(counts).reduce((a,b)=>a+b,0)} meshes</span></div>` +
      Object.entries(counts).map(([k,v])=>`<div class="treeItem"><span>↳ ${k}</span><span>${v}</span></div>`).join("");
  }

  function fitCamera() {
    if (!state.modelGroup) return;
    const bbox = modelBoundingBox();
    const size = bbox.getSize(new THREE.Vector3());
    const center = bbox.getCenter(new THREE.Vector3());
    const radius = Math.max(size.x,size.y,size.z) * .8;
    controls.target.copy(center);
    camera.position.set(center.x + radius, center.y + radius*.65, center.z + radius);
    camera.near = Math.max(.001, radius/1000);
    camera.far = Math.max(100, radius*20);
    camera.updateProjectionMatrix();
    controls.update();
  }

  async function finalizeLoadedTwin(result, sourceLabel) {
    state.modelGroup = result.group;
    state.modelGroup.name = state.modelGroup.name || "Imported_Structure";
    scene.add(state.modelGroup);
    state.modelMeta = result.meta || analyzeGeneric(state.modelGroup, sourceLabel);
    prepareDeckDeformer(result.deck || detectDeckMesh(state.modelGroup));

    setStep("geometry","run");
    await sleep(160);
    buildProfile();
    renderGeometryAnalysis();
    renderTree();
    setStep("geometry","done");

    setStep("candidates","run");
    await sleep(150);
    generateCandidates();
    setStep("candidates","done");

    $("sourceStat").textContent = state.modelMeta.source || sourceLabel;
    $("viewerTitle").textContent = state.modelGroup.name || "Digital twin";
    $("viewerSubtitle").textContent = state.modelMeta.reconstruction || "3D 모델";
    $("projectPath").textContent = `프로젝트 / ${sourceLabel} / 기하 분석 완료`;
    $("inputState").textContent = "준비 완료";
    $("runAiBtn").disabled = state.candidates.tags.length<state.tagCount || state.candidates.anchors.length<state.anchorCount;
    $("clearRecBtn").disabled = false;
    fitCamera();
    addEvent("Geometry analysis completed","INFO");
    setAgent("Geometry Tool 분석이 끝났습니다. 이제 AI가 구조물을 독립적으로 식별하고, 관련 해체·구조·계측 자료를 웹에서 조사한 뒤 최종 위치를 선택할 수 있습니다.", state.aiEnabled ? "ai":"demo");
    setWorkspaceView("viewer");
    setFlowStage("configure");
  }

  // ---------- Input flow ----------
  const sleep = ms => new Promise(r=>setTimeout(r,ms));

  async function handleFiles(fileList) {
    const files = [...fileList];
    if (!files.length) return;
    clearTwin();
    resetPipeline();
    setWorkspaceView("viewer");
    setFlowStage("loading");
    state.files = files;
    setStep("read","run");
    $("inputState").textContent = "읽는 중";
    try {
      const ext = f => f.name.split(".").pop().toLowerCase();
      const types = new Set(files.map(ext));
      await sleep(120);
      setStep("read","done");
      setStep("views","run");

      if (types.size === 1 && types.has("dxf")) {
        state.sourceType = "DXF 도면 세트";
        const dxfs = [];
        for (const f of files) dxfs.push(parseDxf(await f.text(), f.name));
        const labels = dxfs.map(d=>classifyDxfView(d));
        if (!["plan","elevation","section"].every(v=>labels.includes(v))) {
          throw new Error(`DXF 뷰 자동식별 실패: ${labels.join(", ")}`);
        }
        setStep("views","done");
        setStep("reconstruct","run");
        await sleep(180);
        const result = reconstructFromDxf(dxfs);
        setStep("reconstruct","done");
        await finalizeLoadedTwin(result, "DXF 기반 재구성 모델");
        toast("DXF 3장으로 3D 구조물을 재구성했습니다");
      } else if (files.length === 1 && (types.has("glb") || types.has("obj"))) {
        state.sourceType = "3D 모델";
        setStep("views","done");
        setStep("reconstruct","run");
        let group;
        const sourceType = types.has("glb") ? "glb" : "obj";
        if (sourceType === "glb") group = await loadGlb(files[0]);
        else group = await loadObj(files[0]);
        prepareImportedGroup(group, sourceType);
        group.name = files[0].name;
        setStep("reconstruct","done");
        const meta = analyzeGeneric(group, files[0].name, sourceType);
        const deck = detectDeckMesh(group);
        await finalizeLoadedTwin({ group, deck, meta }, files[0].name);
        toast("3D 모델을 불러왔습니다");
      } else {
        throw new Error("GLB/OBJ는 1개씩, DXF는 평면·입면·단면 세트로 업로드해 주세요.");
      }
    } catch (err) {
      console.error(err);
      ["read","views","reconstruct","geometry","candidates"].forEach(s=>{
        const row=document.querySelector(`[data-step="${s}"]`);
        if(row && !row.classList.contains("done")) setStep(s,"fail");
      });
      $("inputState").textContent = "오류";
      toast(err.message);
      addEvent(err.message,"오류");
      setAgent(`<b>입력 처리 실패</b><br>${escapeHtml(err.message)}`, "neutral");
      setFlowStage("upload");
    }
  }

  function escapeHtml(s) {
    return String(s).replace(/[&<>"']/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"}[c]));
  }

  $("startIntroBtn")?.addEventListener("click", () => {
    const landing = $("landingScreen");
    const button = $("startIntroBtn");
    if (!landing || landing.classList.contains("isLeaving")) return;
    if (button) button.disabled = true;
    landing.classList.add("isLeaving");
    window.setTimeout(() => landing.classList.add("hidden"), 920);
  });

  $("fileInput").addEventListener("change", e => handleFiles(e.target.files));
  const drop = $("dropZone");
  ["dragenter","dragover"].forEach(ev=>drop.addEventListener(ev,e=>{e.preventDefault();drop.classList.add("drag")}));
  ["dragleave","drop"].forEach(ev=>drop.addEventListener(ev,e=>{e.preventDefault();drop.classList.remove("drag")}));
  drop.addEventListener("drop",e=>handleFiles(e.dataTransfer.files));


  // ---------- Candidate visualization ----------
  function clearCandidates() {
    state.candidateObjects.forEach(o=>disposeObject(o));
    state.candidateObjects=[];
  }
  let candidatesVisible=false;
  $("candidateBtn").onclick=()=>{
    candidatesVisible=!candidatesVisible;
    clearCandidates();
    if(!candidatesVisible){toast("후보점 숨김");return;}
    const matT=new THREE.MeshBasicMaterial({color:0x35a36b,transparent:true,opacity:.90,depthTest:false,depthWrite:false});
    const matA=new THREE.MeshBasicMaterial({color:0x3b82f6,transparent:true,opacity:.90,depthTest:false,depthWrite:false});
    const candidateRadius=Math.max(.006,Math.min(.016,modelVisualSize()*.006));
    const sphere=new THREE.SphereGeometry(candidateRadius,14,14);
    state.candidates.tags.slice(0,120).forEach(c=>{
      const m=new THREE.Mesh(sphere.clone(),matT.clone());
      m.position.set(c.x,c.y,c.z);m.renderOrder=850;
      m.userData.screenAdaptive=true;m.userData.baseRadius=candidateRadius;
      scene.add(m);state.candidateObjects.push(m);
    });
    state.candidates.anchors.forEach(c=>{
      const m=new THREE.Mesh(sphere.clone(),matA.clone());
      m.position.set(c.x,c.y,c.z);m.renderOrder=850;
      m.userData.screenAdaptive=true;m.userData.baseRadius=candidateRadius;
      scene.add(m);state.candidateObjects.push(m);
    });
    toast("Geometry 후보점 표시");
  };

  // ---------- AI ----------
  async function runAi() {
    if(!state.profile || !state.candidates.tags.length) return;

    $("runAiBtn").disabled = true;
    switchTab("ai");
    resetAiSteps();
    showAnalysisLoading();

    setAiStep("identify","run");
    updateAnalysisLoading(
      "identify",
      "구조물 종류를 판별하고 있습니다",
      "업로드된 모델의 길이, 폭, 높이, 부재 구성과 배치 특징을 분석하고 있습니다.",
      12
    );

    setAgent(
      "<b>AI 분석을 시작했습니다.</b><br>기하 분석 결과와 후보 위치를 바탕으로 구조물 종류를 판별하고 있습니다.",
      state.aiEnabled ? "ai" : "demo"
    );
    addEvent("AI 자율 분석 시작","AI");

    let progressTimers = [];
    const advance = (delay, doneStep, runStep, title, message, percent) => {
      const id = setTimeout(() => {
        if(doneStep) setAiStep(doneStep,"done");
        if(runStep) setAiStep(runStep,"run");
        updateAnalysisLoading(runStep || doneStep, title, message, percent);
        $("activityBody").innerHTML = `<b>${title}</b><br>${message}`;
      }, delay);
      progressTimers.push(id);
    };

    advance(
      1700,
      "identify",
      "research",
      "관련 구조·해체 기술자료를 찾고 있습니다",
      "구조 형식에 맞는 공식 지침, 공공기관 자료, 연구·기술자료를 검색하고 있습니다.",
      32
    );

    advance(
      4100,
      "research",
      "strategy",
      "계측 전략을 수립하고 있습니다",
      "검색된 자료를 바탕으로 어떤 부위를 우선 감시해야 하는지 판단하고 있습니다.",
      54
    );

    advance(
      6600,
      "strategy",
      "evaluate",
      "센서 후보 위치를 비교하고 있습니다",
      "Tag와 Anchor 후보의 위치, 분산도, 계측 범위와 중복 여부를 검토하고 있습니다.",
      74
    );

    advance(
      9000,
      "evaluate",
      "final",
      "최종 센서 배치를 확정하고 있습니다",
      `기하 제약을 만족하는 후보 중 최종 Tag ${state.tagCount}개와 Anchor ${state.anchorCount}개를 선택하고 있습니다.`,
      91
    );

    try {
      const r = await fetch("/api/ai/recommend",{
        method:"POST",
        headers:{"Content-Type":"application/json"},
        body:JSON.stringify({
          stage:$("stageSelect").value,
          tagCount:state.tagCount,
          anchorCount:state.anchorCount,
          profile:state.profile,
          sourceFiles:state.files.map(f=>f.name),
          candidates:state.candidates
        })
      });

      if(!r.ok) {
        const e = await r.json().catch(()=>({}));
        throw new Error(e.error || "AI 요청에 실패했습니다.");
      }

      const result = await r.json();
      state.aiResult = result;
      renderAiResult(result);
      applyAiPlacement(result);
      setWorkspaceView("viewer");
      setFlowStage("recommended");

      progressTimers.forEach(clearTimeout);
      finishAiSteps();
      completeAnalysisLoading();

      const mode = result.mode === "ai" ? "ai" : "demo";
      setAgent(
        `<b>AI 최종 분석 완료</b><br>
        구조물: ${escapeHtml(result.structure_assessment?.type || "미판단")} · 신뢰도 ${Math.round((result.structure_assessment?.confidence || 0)*100)}%<br>
        구조물 판별 → 자료 조사 → 계측 전략 수립 → 후보 검토 → 최종 Tag ${state.tagCount}개 / Anchor ${state.anchorCount}개 배치를 완료했습니다.`,
        mode
      );

      addEvent(`AI 최종 배치 완료 (${result.mode})`,"AI");
      toast(result.mode === "ai"
        ? "AI가 자료 검색 후 최종 센서 위치를 결정했습니다."
        : "데모 AI가 센서 위치를 결정했습니다.");
    } catch(err) {
      progressTimers.forEach(clearTimeout);
      console.error(err);

      const running = document.querySelector('.agentProgress>div.run');
      if(running) {
        running.classList.remove("run");
        running.classList.add("fail");
        running.querySelector("b").textContent = "오류";
      }

      failAnalysisLoading(err.message);
      toast(err.message);
      addEvent(err.message,"오류");
      setAgent(`<b>AI 실행 오류</b><br>${escapeHtml(err.message)}`,"neutral");
      setFlowStage("configure");
    } finally {
      $("runAiBtn").disabled = state.candidates.tags.length<state.tagCount || state.candidates.anchors.length<state.anchorCount;
    }
  }
  $("runAiBtn").onclick=handlePrimaryAction;

  function renderAiResult(r){
    const a = r.structure_assessment || {};
    $("aiStructure").innerHTML = `
      <b>${escapeHtml(a.type || "미판단")}</b><br>
      구조 분류: ${escapeHtml(a.category || "—")} · 신뢰도 ${Math.round((a.confidence || 0)*100)}%<br><br>
      ${escapeHtml(a.summary || "")}<br>
      ${(a.uncertainties || []).map(x => `<span style="color:#8998a0">• 불확실성: ${escapeHtml(x)}</span><br>`).join("")}
    `;

    $("strategyList").innerHTML =
      (r.monitoring_strategy || []).map(x => `<div>• ${escapeHtml(x)}</div>`).join("") || "—";

    $("rationaleList").innerHTML =
      (r.placement_rationale || []).map(x => `<div>• ${escapeHtml(x)}</div>`).join("") || "—";

    const research = $("researchList");
    research.className = "researchList";
    research.innerHTML = "";

    (r.research_summary || []).forEach(src => {
      const item = document.createElement("div");
      item.className = "researchItem";
      const link = src.url
        ? `<a href="${escapeHtml(src.url)}" target="_blank" rel="noopener">출처 보기 ↗</a>`
        : "";
      item.innerHTML = `
        <strong>${escapeHtml(src.title || src.source || "참고자료")}</strong><br>
        <span style="font-size:8.5px;color:#8998a0">${escapeHtml(src.source || "")}</span> ${link}
        <p>${escapeHtml(src.insight || "")}</p>`;
      research.appendChild(item);
    });

    $("structureStat").textContent = (a.type || "미판단").replaceAll("_"," ");
  }

  function getCandidate(id){
    return [...state.candidates.tags,...state.candidates.anchors].find(c=>c.id===id);
  }

  function clearSensors(){
    state.sensorObjects.forEach(o=>disposeObject(o.group||o));
    state.sensorObjects=[];
    state.selected={tags:[],anchors:[]};
    renderSensors();
    refreshDynamicCountLabels();
  }

  function modelVisualSize() {
    const b=modelBoundingBox();
    const sz=b.getSize(new THREE.Vector3());
    return Math.max(sz.x,sz.y,sz.z,1);
  }

  function makeLabelTexture(title, subtitle, colorCss) {
    const canvas=document.createElement("canvas");
    canvas.width=512;canvas.height=150;
    const ctx=canvas.getContext("2d");
    ctx.clearRect(0,0,canvas.width,canvas.height);

    ctx.fillStyle="rgba(255,255,255,.96)";
    roundRect(ctx,12,12,488,126,22);
    ctx.fill();

    ctx.strokeStyle="rgba(154,171,181,.7)";
    ctx.lineWidth=3;
    roundRect(ctx,12,12,488,126,22);
    ctx.stroke();

    ctx.fillStyle=colorCss;
    roundRect(ctx,30,34,12,70,6);
    ctx.fill();

    ctx.fillStyle="#243139";
    ctx.font="700 42px Arial, sans-serif";
    ctx.textBaseline="middle";
    ctx.fillText(title,62,58);

    ctx.fillStyle="#73838c";
    ctx.font="500 25px Arial, sans-serif";
    ctx.fillText(subtitle||"미연결",62,100);

    const tex=new THREE.CanvasTexture(canvas);
    tex.minFilter=THREE.LinearFilter;
    tex.magFilter=THREE.LinearFilter;
    tex.needsUpdate=true;
    return tex;
  }

  function roundRect(ctx,x,y,w,h,r){
    ctx.beginPath();
    ctx.moveTo(x+r,y);
    ctx.arcTo(x+w,y,x+w,y+h,r);
    ctx.arcTo(x+w,y+h,x,y+h,r);
    ctx.arcTo(x,y+h,x,y,r);
    ctx.arcTo(x,y,x+w,y,r);
    ctx.closePath();
  }

  function makeLabelSprite(title, subtitle, type, targetSize) {
    const color=type==="tag"?"#35a36b":"#3b82f6";
    const mat=new THREE.SpriteMaterial({
      map:makeLabelTexture(title,subtitle,color),
      transparent:true,
      opacity:.78,
      depthTest:false,
      depthWrite:false
    });
    const sprite=new THREE.Sprite(mat);
    // compact labels. The old 2.25x0.66 scale covered a large fraction of
    // this small bridge model when zoomed in.
    sprite.scale.set(targetSize*1.35,targetSize*.40,1);
    sprite.renderOrder=1000;
    sprite.userData={
      title,subtitle,type,targetSize,
      baseScale:new THREE.Vector3(targetSize*1.35,targetSize*.40,1),
      baseY:0
    };
    return sprite;
  }

  function refreshLabelSprite(sprite, subtitle) {
    if(!sprite?.material)return;
    const d=sprite.userData;
    sprite.material.map?.dispose?.();
    sprite.material.map=makeLabelTexture(
      d.title,
      subtitle,
      d.type==="tag"?"#35a36b":"#3b82f6"
    );
    sprite.material.needsUpdate=true;
    d.subtitle=subtitle;
  }

  function updateSensorLabels() {
    state.sensorObjects.forEach(o=>{
      const d=o.group.userData;
      if(!d || !o.label)return;
      refreshLabelSprite(o.label,mappedModule(d.type,d.index)||"미연결");
    });
  }

  function makeDangerBadgeSprite(radius){
    const canvas=document.createElement("canvas");
    canvas.width=128;canvas.height=128;
    const ctx=canvas.getContext("2d");
    ctx.clearRect(0,0,128,128);
    ctx.beginPath();ctx.arc(64,64,48,0,Math.PI*2);
    ctx.fillStyle="#d92525";ctx.fill();
    ctx.lineWidth=8;ctx.strokeStyle="#ffffff";ctx.stroke();
    ctx.fillStyle="#ffffff";
    ctx.font="bold 72px system-ui, sans-serif";
    ctx.textAlign="center";ctx.textBaseline="middle";
    ctx.fillText("!",64,68);
    const tex=new THREE.CanvasTexture(canvas);
    tex.needsUpdate=true;
    const sprite=new THREE.Sprite(new THREE.SpriteMaterial({
      map:tex,transparent:true,depthTest:false,depthWrite:false
    }));
    const size=Math.max(radius*2.6,.02);
    sprite.scale.set(size,size,1);
    sprite.position.set(0,radius*3.7,0);
    sprite.visible=false;
    sprite.renderOrder=960;
    sprite.userData.baseScale=size;
    return sprite;
  }

  function buildPointVisual(type,index,targetSize,candidate) {
    const wrapper=new THREE.Group();
    const accent=type==="tag"?0x35a36b:0x3b82f6;
    const cm=caseMetricsInModelUnits();
    const maxDim=modelVisualSize();

    // Pure visual marker. Its coordinate is the physical enclosure centre.
    // Marker size is enlarged only for visibility and does NOT represent
    // the installation footprint used by candidate validation.
    // sensor markers are visual pointers, not full-size enclosures. Keep
    // them small enough that the underlying structural surface remains visible.
    const radius=Math.max(Math.min(maxDim*.0024,.035),Math.min(cm.width,cm.depth)*.055,.008);
    const point=new THREE.Mesh(
      new THREE.SphereGeometry(radius,20,20),
      new THREE.MeshStandardMaterial({
        color:accent,roughness:.42,metalness:.04,
        emissive:accent,emissiveIntensity:.14,
        transparent:true,opacity:.86,depthTest:false,depthWrite:false
      })
    );
    point.castShadow=true;
    wrapper.add(point);

    const halo=new THREE.Mesh(
      new THREE.RingGeometry(radius*1.28,radius*1.52,32),
      new THREE.MeshBasicMaterial({
        color:accent,side:THREE.DoubleSide,transparent:true,opacity:.20,
        depthTest:false,depthWrite:false
      })
    );
    halo.rotation.x=-Math.PI/2;
    halo.position.y=-radius*.12;
    wrapper.add(halo);

    const dangerAura=new THREE.Mesh(
      new THREE.RingGeometry(radius*1.95,radius*2.65,42),
      new THREE.MeshBasicMaterial({
        color:0xd33b3b,side:THREE.DoubleSide,transparent:true,opacity:.22,
        depthTest:false,depthWrite:false
      })
    );
    dangerAura.rotation.x=-Math.PI/2;
    dangerAura.position.y=-radius*.10;
    dangerAura.visible=false;
    wrapper.add(dangerAura);

    const dangerAuraOuter=new THREE.Mesh(
      new THREE.RingGeometry(radius*3.05,radius*3.38,48),
      new THREE.MeshBasicMaterial({
        color:0xff3b30,side:THREE.DoubleSide,transparent:true,opacity:.16,
        depthTest:false,depthWrite:false
      })
    );
    dangerAuraOuter.rotation.x=-Math.PI/2;
    dangerAuraOuter.position.y=-radius*.09;
    dangerAuraOuter.visible=false;
    wrapper.add(dangerAuraOuter);

    const dangerBadge=type==="tag"?makeDangerBadgeSprite(radius):null;
    if(dangerBadge)wrapper.add(dangerBadge);

    wrapper.renderOrder=900;
    wrapper.userData.markerRadius=radius;
    wrapper.userData.point=point;
    wrapper.userData.halo=halo;
    wrapper.userData.dangerAura=dangerAura;
    wrapper.userData.dangerAuraOuter=dangerAuraOuter;
    wrapper.userData.dangerBadge=dangerBadge;
    wrapper.userData.dangerActive=false;
    return wrapper;
  }

  function makeSensor(type,index,c){
    const group=new THREE.Group();
    const targetSize=Math.max(.10,Math.min(.28,modelVisualSize()*.017));
    const marker=buildPointVisual(type,index,targetSize,c);
    group.add(marker);

    if(type==="tag"){
      const n=new THREE.Vector3(Number(c.nx??0),Number(c.ny??1),Number(c.nz??0));
      if(n.lengthSq()>1e-8){
        n.normalize();
        marker.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),n);
      }
    }
    const baseBodyQuaternion=marker.quaternion.clone();

    const radius=marker.userData.markerRadius||targetSize*.18;
    const label=makeLabelSprite(
      `${type==="tag"?"Tag":"Anchor"} ${index}`,
      mappedModule(type,index)||"미연결",
      type,
      Math.max(targetSize,.11)
    );
    label.position.y=radius*1.35+Math.max(c.caseHeight||0,.025)+targetSize*.18;
    label.userData.baseY=label.position.y;
    group.add(label);

    // Elevated Anchor visual: show a thin fixed-support stem down to the ground so
    // a high Anchor does not look like an accidental floating marker.
    if(type==="anchor" && Number.isFinite(Number(c.groundY)) && c.y-Number(c.groundY)>targetSize*.7){
      const stemBottom=Number(c.groundY)-c.y;
      const stemTop=-Math.max(radius*.55,targetSize*.08);
      const stemGeom=new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(0,stemBottom,0),
        new THREE.Vector3(0,stemTop,0)
      ]);
      const stem=new THREE.Line(stemGeom,new THREE.LineBasicMaterial({
        color:0x819098,transparent:true,opacity:.28
      }));
      stem.userData.anchorHeightGuide=true;
      group.add(stem);
    }

    group.position.set(c.x,c.y,c.z);
    group.userData={
      type,index,candidateId:c.id,zMm:0,tilt:0,vibration:0,
      baseY:c.y,targetSize,
      physicalSurfaceY:c.surfaceY,
      caseWidth:c.caseWidth,
      caseDepth:c.caseDepth,
      caseHeight:c.caseHeight,
      nx:Number(c.nx??0),ny:Number(c.ny??1),nz:Number(c.nz??0),
      tux:Number(c.tux??1),tuy:Number(c.tuy??0),tuz:Number(c.tuz??0),
      tvx:Number(c.tvx??0),tvy:Number(c.tvy??0),tvz:Number(c.tvz??1),
      surfaceX:Number(c.surfaceX??c.x),surfaceY:Number(c.surfaceY??c.y),surfaceZ:Number(c.surfaceZ??c.z),
      sourceMeshUuid:c.sourceMeshUuid||null,surfaceClass:c.surfaceClass||c.surface||"unknown",
      basePosition:new THREE.Vector3(c.x,c.y,c.z),
      baseQuaternion:group.quaternion.clone(),
      baseBodyQuaternion
    };
    scene.add(group);
    return {group,body:marker,ring:marker.userData.halo,label};
  }
  function applyAiPlacement(r){
    clearSensors();
    const tIds=r?.final_placement?.tag_candidate_ids||[];
    const aIds=r?.final_placement?.anchor_candidate_ids||[];
    state.selected.tags=tIds.map(getCandidate).filter(Boolean).slice(0,state.tagCount);
    state.selected.anchors=aIds.map(getCandidate).filter(c=>c && ModelAnchorLimits.contains(c,currentAnchorLimits())).slice(0,state.anchorCount);

    state.selected.tags.forEach((c,i)=>state.sensorObjects.push(makeSensor("tag",i+1,c)));
    state.selected.anchors.forEach((c,i)=>state.sensorObjects.push(makeSensor("anchor",i+1,c)));

    refreshDynamicCountLabels();
    $("simBtn").disabled=state.selected.tags.length!==state.tagCount;
    $("zeroBtn").disabled=state.selected.tags.length!==state.tagCount;
    $("clearRecBtn").disabled=false;
    renderSensors();
    updateSensorLabels();
  }

  $("clearRecBtn").onclick=()=>{
    clearSensors();state.aiResult=null;
    state.monitoringActive=false;state.calibrationRequested=false;
    hideTagInfoPanel();hideCalibrationToast();
    $("simBtn").disabled=true;$("zeroBtn").disabled=true;
    if(state.modelGroup)setFlowStage("configure");
    toast("AI 추천 배치를 지웠습니다");
  };

  // ---------- REAL SENSOR STREAM ----------
  // Raw sensor packets may arrive much faster than the viewer needs to redraw.
  // Keep telemetry ingestion independent and coalesce heavy 3D/DOM work to 5 Hz.
  function currentDeformSignature(){
    return tagSensors().map(t=>{
      const d=t.group.userData;
      const z=Math.round(Number(d.zMm||0)*4)/4;      // 0.25 mm visual granularity
      const tilt=Math.round(Number(d.tilt||0)*10)/10;
      return [d.index,z,tilt,Boolean(d.vibration),Boolean(d.measurementValid),Boolean(d.imuOk)].join(":");
    }).join("|");
  }

  function scheduleLiveVisualRefresh(needsDeform=false){
    liveVisualNeedsDeform = liveVisualNeedsDeform || needsDeform;
    if(liveVisualTimer)return;
    const wait=Math.max(0,LIVE_VISUAL_MIN_MS-(performance.now()-lastLiveVisualMs));
    liveVisualTimer=setTimeout(()=>{
      liveVisualTimer=null;
      lastLiveVisualMs=performance.now();
      if(state.workspaceView!=="viewer")return;
      const sig=currentDeformSignature();
      if(liveVisualNeedsDeform && sig!==lastDeformSignature){
        deformDeck();
        lastDeformSignature=sig;
      }
      liveVisualNeedsDeform=false;
      renderSensors();
      if(performance.now()-lastTrendUpdateMs>=700){
        updateTrend();
        lastTrendUpdateMs=performance.now();
      }
    },wait);
  }

  function mappedLogicalSensor(nodeId) {
    for (let i=1;i<=state.tagCount;i++) if(mappedModule("tag",i)===nodeId)return {type:"tag",index:i};
    for (let i=1;i<=state.anchorCount;i++) if(mappedModule("anchor",i)===nodeId)return {type:"anchor",index:i};
    return null;
  }

  function applyLiveSensorData() {
    if(state.simOn)return;
    let anyLiveTag=false;
    Object.values(state.liveSensorStates).forEach(raw=>{
      const logical=mappedLogicalSensor(raw.node_id);
      if(!logical)return;
      const obj=state.sensorObjects.find(o=>o.group.userData.type===logical.type && o.group.userData.index===logical.index);
      if(!obj)return;
      const d=obj.group.userData;
      d.liveOnline=Boolean(raw.online);
      d.dzMethod=raw.dz_method||"";
      d.rawZMm=Number(raw.raw_dz_mm ?? raw.dz_mm ?? 0);
      d.measurementValid=raw.measurement_valid===true;
      d.measurementHeld=raw.measurement_held===true;
      d.holdAgeMs=Number(raw.hold_age_ms||0);
      d.referenceOnly=!!raw.reference_only;
      d.resolutionLimited=!!raw.resolution_limited;
      d.uwbCalibration=raw.uwb_calibration||null;
      d.uwbUncertaintyMm=raw.uwb_uncertainty_mm;
      d.uwbResidualRmsM=raw.uwb_residual_rms_m;
      d.wifiRssi=raw.wifi_rssi;
      d.batteryV=Number(raw.battery_v ?? 0);
      d.dynamicG=Number(raw.accel_g ?? 0);
      d.imuOk=raw.imu_ok !== false;
      d.axG=Number(raw.ax_g ?? 0);
      d.ayG=Number(raw.ay_g ?? 0);
      d.azG=Number(raw.az_g ?? 0);
      d.imuFailures=Number(raw.imu_failures ?? 0);
      d.nodeId=raw.node_id||"";
      d.lastSeenMs=Number(raw.last_seen_ms||Date.now());
      d.fw=raw.fw||"";
      d.uwbActive=raw.uwb_active !== false;
      if(logical.type==="tag" && raw.online){
        d.measurementValid=raw.measurement_valid===true;
        d.measurementHeld=raw.measurement_held===true;
        d.holdAgeMs=Number(raw.hold_age_ms||0);
      d.referenceOnly=!!raw.reference_only;
        if(d.measurementValid && raw.dz_mm!=null) d.zMm=Number(raw.dz_mm);
        if(d.imuOk && Number.isFinite(Number(raw.tilt_deg))) d.tilt=Number(raw.tilt_deg);
        // Keep the firmware flag for diagnostics, but raise the UI danger criterion
        // so tiny vibration does not immediately create a danger state.
        d.rawVibration=d.imuOk && raw.vibration?1:0;
        d.vibration=d.imuOk && Math.abs(Number(d.dynamicG||0))>=RISK_LIMITS.dangerVibrationG?1:0;
        d.rangeCount=Number(raw.range_count||0);
        anyLiveTag=true;
      }
    });
    if(anyLiveTag){
      $("twinMode").textContent=tagSensors().some(t=>t.group.userData.referenceOnly)?"실센서 · 참고 추정":"실센서";
    }
    scheduleLiveVisualRefresh(anyLiveTag);
  }

  function handleSensorSnapshot(payload){
    const modules=Array.isArray(payload.modules)?payload.modules:[];
    const sig=JSON.stringify(modules.map(m=>[m.node_id,m.role,m.online]));
    state.discoveredModules=modules;
    if(sig!==state.moduleSignature){
      state.moduleSignature=sig;
      renderModuleMapping();
      updateSensorLabels();
    }
    state.liveSensorStates={};
    (payload.sensors||[]).forEach(x=>state.liveSensorStates[x.node_id]=x);
    applyLiveSensorData();
    renderRawLiveTag();
    updateCalibrationFromSnapshot(payload);
    captureMonitoringSamples(payload?.server_time_ms||Date.now());
    updateTagInfoPanel();
    if(state.workspaceView==="stats")scheduleStatsRender();
    window.dispatchEvent(new CustomEvent("uwb-snapshot",{detail:payload}));
  }


  function renderRawLiveTag() {
    const raw = state.liveSensorStates["TAG-01"]
      || Object.values(state.liveSensorStates).find(x=>x?.role==="tag")
      || null;

    const online = Boolean(raw?.online);
    const dot = $("liveDot");
    if(dot) dot.classList.toggle("online", online);

    if($("liveConnectionText")){
      $("liveConnectionText").textContent = online
        ? `Wi-Fi 연결 · ${raw.node_id || "TAG-01"} · Info ${Number(raw.requested_links||raw.range_profile||raw.range_count||0)}A · UWB ${Number(raw.range_count||0)}/${raw.uwb_calibration?.total_links||4}`
        : "데이터 수신 대기";
    }

    if($("liveDz")) $("liveDz").textContent =
      online && raw?.measurement_valid===true && raw?.dz_mm!=null && Number.isFinite(Number(raw.dz_mm))
        ? `${Number(raw.dz_mm).toFixed(1)} mm`
        : "—";

    if($("liveRawDz")) $("liveRawDz").textContent =
      online && raw?.raw_dz_mm!=null && Number.isFinite(Number(raw.raw_dz_mm))
        ? `${Number(raw.raw_dz_mm).toFixed(1)} mm`
        : "—";

    const cal=raw?.uwb_calibration||{};
    if($("liveCalibration")){
      if(!online) $("liveCalibration").textContent="—";
      else if(cal.full_ready) $("liveCalibration").textContent=`완료 ${cal.calibrated_links||0}/${cal.total_links||4}`;
      else if(cal.ready) $("liveCalibration").textContent=`사용 가능 ${cal.calibrated_links||0}/${cal.total_links||4}`;
      else $("liveCalibration").textContent=`${cal.reason||"보정 중"} ${Math.round(Number(cal.progress||0)*100)}%`;
    }
    if($("liveUncertainty")) $("liveUncertainty").textContent =
      online && raw?.uwb_uncertainty_mm!=null && Number.isFinite(Number(raw.uwb_uncertainty_mm))
        ? `±${Number(raw.uwb_uncertainty_mm).toFixed(1)} mm`
        : "—";
    if($("liveResidual")) $("liveResidual").textContent =
      online && raw?.uwb_residual_rms_m!=null && Number.isFinite(Number(raw.uwb_residual_rms_m))
        ? `${(Number(raw.uwb_residual_rms_m)*1000).toFixed(1)} mm`
        : "—";

    if($("liveSensitivity")) $("liveSensitivity").textContent=online && (raw?.normal_sensitivity??raw?.vertical_sensitivity)!=null?
      `${Number(raw.normal_sensitivity??raw.vertical_sensitivity).toFixed(2)} · 낮을수록 불리`:"—";
    if($("liveTemporalMode")){
      const temporalLabels={
        stable_noise_average:"정지 안정화 · 긴 거리 평균",
        static_coordinate_hold:"정지 홀드 · 좌표 기반 고정",
        precision_time_filter:"정지 평균 · 1.8초 거리 평균",
        responsive_time_filter:"변형 추적 · 안정 추적",
        robust_sliding_window:"R13 Robust Window · 시간융합",
        unified_robust_sliding_window:"R13.1 Unified Adaptive · 시간융합",
        unified_window_warmup:"R13.1 Unified Adaptive · 준비 중",
        trimmed7_median5_time_filter:"시간 필터 · 지속 변화 추종",
        instant_geometry_consensus:"기하 일치 · 즉시값",
        median9_outlier_recovery:"이상치 복구",
        "":"—"
      };
      $("liveTemporalMode").textContent=online
        ? (temporalLabels[raw?.uwb_temporal_mode]||raw?.uwb_temporal_mode||"—")
        : "—";
    }

    const imuReadable=online && raw?.imu_ok !== false;
    if($("liveTilt")) $("liveTilt").textContent =
      imuReadable && Number.isFinite(Number(raw?.tilt_deg))
        ? `${Number(raw.tilt_deg).toFixed(2)}°`
        : online && raw?.imu_ok===false ? "IMU 오류" : "—";

    if($("liveVibration")) $("liveVibration").textContent =
      imuReadable ? (raw.vibration ? "감지" : "없음") : online && raw?.imu_ok===false ? "IMU 오류" : "—";

    const ranges = Array.isArray(raw?.ranges) ? raw.ranges : [];
    const count = Number.isFinite(Number(raw?.range_count))
      ? Number(raw.range_count)
      : ranges.length;
    if($("liveRangeCount")) $("liveRangeCount").textContent = `${count} active / 4 · Info target ${Number(raw?.requested_links||raw?.range_profile||count||0)}`;

    const rangeMap = new Map(
      ranges
        .filter(r=>r && r.anchor_id)
        .map(r=>[String(r.anchor_id), r])
    );

    const rangeList = $("liveRangeList");
    if(rangeList){
      rangeList.innerHTML = "";
      for(let i=1;i<=4;i++){
        const id=`ANCHOR-${String(i).padStart(2,"0")}`;
        const r=rangeMap.get(id);
        const scheduled=r?.scheduled !== false;
        const ok=scheduled && r?.ok !== false;
        const m=ok?Number(r?.m):NaN;
        const corrected=ok&&r?.corrected_m!=null?Number(r.corrected_m):NaN;
        const bias=Number(r?.bias_m);
        const outlier=Boolean(r?.outlier);
        const calibrated=Boolean(r?.calibrated);
        const rate=Number(r?.success_rate);
        const attempts=Number(r?.attempts);
        const successes=Number(r?.successes);
        const recentSamples=Number(r?.recent_samples);
        const recentSuccesses=Number(r?.recent_successes);
        const nlosScore=Number(r?.nlos_score);
        const nlosSuspect=Boolean(r?.nlos_suspect);
        const fpGap=Number(r?.fp_peak_gap);

        let label="응답 없음";
        let cls="fail";
        if(r && !scheduled){
          label="이번 cycle 미사용";cls="";
        }else if(r && !ok){
          label="이번 측정 실패";cls="fail";
        }else if(Number.isFinite(rate)){
          if(rate>=85){label="통신 양호";cls="ok";}
          else if(rate>=60){label="양호";cls="warn";}
          else if(rate>0){label="불안정";cls="fail";}
        }else if(Number.isFinite(m)){
          label="이번 측정 성공";cls="ok";
        }

        const row=document.createElement("div");
        row.className=cls;
        row.innerHTML=`
          <span>${id}<small>${Number.isFinite(rate)
            ? `${outlier?"기하 이상치 · ":""}${nlosSuspect?"NLOS 의심 · ":""}${label} · 최근 연결률 ${rate.toFixed(0)}%${Number.isFinite(recentSamples)?` (${Number.isFinite(recentSuccesses)?recentSuccesses:0}/${recentSamples})`:Number.isFinite(attempts)?` (${Number.isFinite(successes)?successes:0}/${attempts})`:""}${Number.isFinite(nlosScore)?` · Q ${(100*(1-nlosScore)).toFixed(0)}%`:""}`
            : label}</small></span>
          <b>${Number.isFinite(corrected)?`${corrected.toFixed(3)} m`:Number.isFinite(m)?`${m.toFixed(3)} m`:"—"}
            <small style="display:block;font-size:9px;font-weight:500;color:#657983">${Number.isFinite(corrected)?"시간 필터 후 거리":"현재 원시 거리"}</small>
            ${Number.isFinite(m)&&calibrated?`<small style="display:block;font-size:9px;font-weight:500;color:#657983">원시 ${m.toFixed(3)} m<br>현재 보정 ${r.instant_corrected_m!=null?Number(r.instant_corrected_m).toFixed(3):"—"} m<br>고정 bias ${bias>=0?"+":""}${bias.toFixed(3)} m<br>거리 잡음 σ ${r.noise_m!=null?(Number(r.noise_m)*1000).toFixed(1):"—"} mm${Number.isFinite(fpGap)?`<br>CIR FP↔Peak ${fpGap.toFixed(2)} sample`:""}</small>`:""}
          </b>`;
        rangeList.appendChild(row);
      }
    }

    if($("liveRssi")) $("liveRssi").textContent =
      online && Number.isFinite(Number(raw?.wifi_rssi))
        ? `${Number(raw.wifi_rssi)} dBm`
        : "—";

    if($("liveAccel")) $("liveAccel").textContent =
      online && Number.isFinite(Number(raw?.accel_g))
        ? `${Number(raw.accel_g).toFixed(3)} g`
        : "—";

    if($("liveBattery")) $("liveBattery").textContent =
      online && Number(raw?.battery_v) > 0
        ? `${Number(raw.battery_v).toFixed(2)} V`
        : "—";

    if($("liveImu")) $("liveImu").textContent =
      online ? (raw?.imu_ok === false ? "확인 필요" : "정상") : "—";

    if($("liveSeq")) $("liveSeq").textContent =
      online && raw?.seq != null ? String(raw.seq) : "—";

    if($("liveDzMethod")){
      const labels={
        metric_pair_calibrated_surface_normal_geometry:"좌표보정 + Surface Normal Geometry",
        metric_pair_calibrated_robust_geometry:"좌표보정 + Robust Geometry",
        calibrating_metric_pairs:"절대좌표 Pair 보정 중",
        range_delta_proxy_nonmetric:"상대거리 기준",
        stale_data:"새 거리 데이터 없음",
        no_range:"UWB 거리 없음",
        filter_warmup:"거리 필터 준비 중",
        coordinates_required:"실제 좌표·단위 확인 필요",
        insufficient_geometry:"유효 Anchor 부족 / 계산 불가",
        weak_vertical_geometry:"법선 감도 부족 · Anchor 배치 확인",
        weak_normal_geometry:"법선 감도 부족 · Anchor 배치 확인",
        inconsistent_ranges:"거리 불일치 · 측정 보류"
      };
      $("liveDzMethod").textContent=online ? (labels[raw?.dz_method]||raw?.dz_method||"—") : "—";
    }
  }

  function startSensorStream(){
    const es=new EventSource("/api/sensors/stream");
    es.onopen=()=>{state.sensorStreamConnected=true;addEvent("실센서 브리지 연결됨","INFO")};
    es.onmessage=e=>{
      try{handleSensorSnapshot(JSON.parse(e.data))}
      catch(err){console.warn("sensor SSE parse error",err)}
    };
    es.onerror=()=>{
      state.sensorStreamConnected=false;
      Object.values(state.liveSensorStates).forEach(x=>x.online=false);
      renderRawLiveTag();
      window.dispatchEvent(new CustomEvent('uwb-disconnected'));
    };

    // Robust fallback: raw Tag data still updates even if SSE is interrupted.
    setInterval(async()=>{
      try{
        const r=await fetch("/api/sensors/state",{cache:"no-store"});
        if(r.ok) handleSensorSnapshot(await r.json());
      }catch{}
    },1000);
  }

  // ---------- Sensors + deformation ----------
  function tagSensors(){return state.sensorObjects.filter(o=>o.group.userData.type==="tag")}
  function classification(s){
    const d=s.group.userData;
    if(!state.simOn && !d.liveOnline)return "unknown";
    const uwbReadable=state.simOn || (d.measurementValid&&!d.resolutionLimited);
    const imuReadable=state.simOn || d.imuOk;
    if(!uwbReadable&&!imuReadable)return "unknown";
    const az=Math.abs(Number(d.zMm||0));
    const at=Math.abs(Number(d.tilt||0));
    if((uwbReadable&&az>=RISK_LIMITS.dangerDzMm) || (imuReadable&&(at>=RISK_LIMITS.dangerTiltDeg || Boolean(d.vibration))))return "danger";
    if((uwbReadable&&az>=RISK_LIMITS.warningDzMm) || (imuReadable&&at>=RISK_LIMITS.warningTiltDeg))return "warning";
    return "normal";
  }

  function riskItemsForData(d,severity){
    const items=[];
    const uwbReadable=state.simOn || (d.liveOnline&&d.measurementValid&&!d.resolutionLimited);
    const imuReadable=state.simOn || (d.liveOnline&&d.imuOk);
    const absDz=Math.abs(Number(d.zMm||0));
    const absTilt=Math.abs(Number(d.tilt||0));
    if(severity==="danger"){
      if(uwbReadable&&absDz>=RISK_LIMITS.dangerDzMm)items.push("변위");
      if(imuReadable&&absTilt>=RISK_LIMITS.dangerTiltDeg)items.push("기울기");
      if(imuReadable&&d.vibration)items.push("진동");
    }else if(severity==="warning"){
      if(uwbReadable&&absDz>=RISK_LIMITS.warningDzMm)items.push("변위");
      if(imuReadable&&absTilt>=RISK_LIMITS.warningTiltDeg)items.push("기울기");
    }
    return [...new Set(items)];
  }

  function captureMonitoringSamples(now=Date.now()){
    if(!state.monitoringActive&&!state.simOn)return;
    tagSensors().forEach(tag=>{
      const d=tag.group.userData;
      const idx=Number(d.index);
      const valid=state.simOn || (d.liveOnline&&(d.measurementValid||d.imuOk));
      if(!valid)return;
      const last=Number(state.monitorHistoryLastMs[idx]||0);
      if(now-last<800)return;
      state.monitorHistoryLastMs[idx]=now;
      const arr=state.monitorHistory[idx] ||= [];
      arr.push({t:now,dz:Number(d.zMm||0),tilt:Number(d.tilt||0),vibration:Boolean(d.vibration)});
      while(arr.length>900)arr.shift();

      const st=classification(tag);
      const prev=state.lastRiskStates[idx]||"normal";
      if((st==="warning"||st==="danger")&&st!==prev){
        const items=riskItemsForData(d,st);
        state.riskEvents.unshift({
          t:now,tag:`Tag ${idx}`,severity:st,
          items:items.length?items:["센서 상태"],
          dz:Number(d.zMm||0),tilt:Number(d.tilt||0),vibration:Boolean(d.vibration)
        });
        while(state.riskEvents.length>100)state.riskEvents.pop();
      }
      state.lastRiskStates[idx]=st;
    });
  }

  let statsRenderPending=false;
  function scheduleStatsRender(){
    if(statsRenderPending)return;
    statsRenderPending=true;
    setTimeout(()=>{statsRenderPending=false;renderStatsDashboard();},180);
  }

  function renderStatsDashboard(){
    const tags=tagSensors();
    const gridHost=$("statsTagGrid");
    if(!gridHost)return;

    if(!tags.length){
      gridHost.innerHTML='<div class="emptyState">AI 추천 후 모니터링을 시작하면 각 Tag의 실시간 데이터가 표시됩니다.</div>';
    }else{
      gridHost.innerHTML=tags.map(tag=>{
        const d=tag.group.userData;
        const st=classification(tag);
        const valid=state.simOn || (d.liveOnline&&d.measurementValid);
        const imuReadable=state.simOn || (d.liveOnline&&d.imuOk);
        const readable=d.measurementHeld&&!state.simOn?"일시 유지":st==="danger"?"위험":st==="warning"?"주의":st==="normal"?"정상":"대기";
        return `<div class="statsTagCard ${st==="danger"?"danger":""}">
          <div class="statsTagHead"><strong>Tag ${d.index}</strong><span class="statusChip ${st==="danger"?"danger":""}">${readable}</span></div>
          <div class="statsValues">
            <div><small>법선 변위</small><b>${valid?`${Number(d.zMm||0).toFixed(1)} mm`:"—"}</b></div>
            <div><small>기울기 X</small><b>${imuReadable?`${Number(d.tilt||0).toFixed(2)}°`:d.liveOnline?"IMU 오류":"—"}</b></div>
            <div><small>진동</small><b>${imuReadable?(d.vibration?"감지":"없음"):d.liveOnline?"IMU 오류":"—"}</b></div>
          </div>
        </div>`;
      }).join("");
    }

    $("statsLiveStatus").textContent=state.monitoringActive
      ? `실시간 수집 중 · ${tags.filter(t=>t.group.userData.liveOnline).length}/${tags.length} Tag`
      : state.simOn?"시뮬레이션 데이터":"모니터링 대기";

    const select=$("statsTagSelect");
    const old=select.value;
    const options=tags.map(t=>`<option value="${t.group.userData.index}">Tag ${t.group.userData.index}</option>`).join("");
    if(select.innerHTML!==options)select.innerHTML=options;
    if(old&&[...select.options].some(o=>o.value===old))select.value=old;
    const selected=Number(select.value||tags[0]?.group.userData.index||1);
    renderStatsChart(selected);

    const log=$("riskEventLog");
    $("riskEventCount").textContent=`${state.riskEvents.length}건`;
    if(!state.riskEvents.length){
      log.innerHTML='<div class="emptyState">위험 신호가 발생하면 시간과 Tag, 발생 항목을 기록합니다.</div>';
    }else{
      log.innerHTML=state.riskEvents.map(e=>`
        <div class="riskEventRow ${e.severity}">
          <time>${new Date(e.t).toLocaleTimeString()}</time>
          <strong>${escapeHtml(e.tag)}</strong>
          <span>${escapeHtml(e.items.join(" · "))} · Δn ${e.dz.toFixed(1)} mm · Tilt ${e.tilt.toFixed(1)}°${e.vibration?" · 진동":""}</span>
          <span class="eventBadge ${e.severity}">${e.severity==="danger"?"위험":"주의"}</span>
        </div>`).join("");
    }
  }

  function renderStatsChart(tagIndex){
    const arr=(state.monitorHistory[tagIndex]||[]).slice(-600);
    const empty=$("statsChartEmpty");
    const dzLine=$("statsDisplacementLine");
    const tiltLine=$("statsTiltLine");
    const grid=$("statsChartGrid");
    if(arr.length<2){
      empty.classList.remove("hidden");
      dzLine.setAttribute("points","");
      tiltLine.setAttribute("points","");
      grid.innerHTML="";
      return;
    }
    empty.classList.add("hidden");

    const W=900,H=260,left=50,right=50,top=18,bottom=30;
    const plotW=W-left-right,plotH=H-top-bottom;
    const t0=arr[0].t,t1=Math.max(t0+1,arr[arr.length-1].t);
    const maxDz=Math.max(5,...arr.map(x=>Math.abs(x.dz)))*1.15;
    const maxTilt=Math.max(3,...arr.map(x=>Math.abs(x.tilt)))*1.15;
    const x=t=>left+(t-t0)/(t1-t0)*plotW;
    const y=(v,max)=>top+plotH/2-(v/max)*(plotH/2);

    dzLine.setAttribute("points",arr.map(p=>`${x(p.t).toFixed(1)},${y(p.dz,maxDz).toFixed(1)}`).join(" "));
    tiltLine.setAttribute("points",arr.map(p=>`${x(p.t).toFixed(1)},${y(p.tilt,maxTilt).toFixed(1)}`).join(" "));

    const rows=[0,.25,.5,.75,1].map(q=>{
      const yy=top+q*plotH;
      const dz=(maxDz*(1-2*q)).toFixed(0);
      const ti=(maxTilt*(1-2*q)).toFixed(1);
      return `<line class="chartGridLine" x1="${left}" y1="${yy}" x2="${W-right}" y2="${yy}"/>
        <text class="chartAxisText" x="2" y="${yy+3}">${dz} mm</text>
        <text class="chartAxisText" text-anchor="end" x="${W-2}" y="${yy+3}">${ti}°</text>`;
    }).join("");
    const timeLabels=`<text class="chartAxisText" x="${left}" y="${H-6}">${new Date(t0).toLocaleTimeString()}</text>
      <text class="chartAxisText" text-anchor="end" x="${W-right}" y="${H-6}">${new Date(t1).toLocaleTimeString()}</text>`;
    grid.innerHTML=rows+timeLabels;
  }

  function hideTagInfoPanel(){
    state.selectedTagPopupIndex=null;
    $("tagInfoPanel")?.classList.add("hidden");
  }

  function updateTagInfoPanel(){
    const idx=Number(state.selectedTagPopupIndex);
    if(!idx)return;
    const tag=tagSensors().find(t=>Number(t.group.userData.index)===idx);
    if(!tag){hideTagInfoPanel();return;}
    const d=tag.group.userData;
    const valid=state.simOn || (d.liveOnline&&d.measurementValid);
    const imuReadable=state.simOn || (d.liveOnline&&d.imuOk);
    $("tagInfoTitle").textContent=`Tag ${idx}`;
    $("tagInfoDisplacement").textContent=valid?`${Number(d.zMm||0).toFixed(1)} mm`:"—";
    $("tagInfoTilt").textContent=imuReadable?`${Number(d.tilt||0).toFixed(2)}°`:d.liveOnline?"IMU 오류":"—";
    $("tagInfoVibration").textContent=imuReadable?(d.vibration?"감지":"없음"):d.liveOnline?"IMU 오류":"—";
    $("tagInfoTime").textContent=d.lastSeenMs?new Date(d.lastSeenMs).toLocaleTimeString():new Date().toLocaleTimeString();

    const st=classification(tag);
    const dangerItems=new Set(st==="danger"?riskItemsForData(d,"danger"):[]);
    $("tagInfoDisplacementRow").classList.toggle("risk",dangerItems.has("변위"));
    $("tagInfoTiltRow").classList.toggle("risk",dangerItems.has("기울기"));
    $("tagInfoVibrationRow").classList.toggle("risk",dangerItems.has("진동"));
  }

  function updateTagInfoPanelPosition(){
    const panel=$("tagInfoPanel");
    if(!panel || panel.classList.contains("hidden"))return;
    const idx=Number(state.selectedTagPopupIndex);
    const tag=tagSensors().find(t=>Number(t.group.userData.index)===idx);
    if(!tag||state.workspaceView!=="viewer"){panel.style.opacity="0";return;}
    const p=new THREE.Vector3();
    tag.group.getWorldPosition(p);
    p.project(camera);
    if(p.z<-1||p.z>1){panel.style.opacity="0";return;}
    panel.style.opacity="1";
    const w=$("canvasHost").clientWidth,h=$("canvasHost").clientHeight;
    let left=(p.x*.5+.5)*w+14;
    let top=(-p.y*.5+.5)*h-24;
    const pw=panel.offsetWidth||220,ph=panel.offsetHeight||110;
    left=Math.max(8,Math.min(w-pw-8,left));
    top=Math.max(8,Math.min(h-ph-8,top));
    panel.style.left=`${left}px`;
    panel.style.top=`${top}px`;
  }

  const sensorClickRaycaster=new THREE.Raycaster();
  const sensorClickPointer=new THREE.Vector2();
  renderer.domElement.addEventListener("click",event=>{
    if((!state.monitoringActive&&!state.simOn)||state.workspaceView!=="viewer")return;
    const rect=renderer.domElement.getBoundingClientRect();
    sensorClickPointer.x=((event.clientX-rect.left)/rect.width)*2-1;
    sensorClickPointer.y=-((event.clientY-rect.top)/rect.height)*2+1;
    sensorClickRaycaster.setFromCamera(sensorClickPointer,camera);
    const tags=tagSensors();
    const hits=sensorClickRaycaster.intersectObjects(tags.map(t=>t.group),true);
    if(!hits.length){hideTagInfoPanel();return;}
    const hit=hits[0].object;
    const found=tags.find(t=>{
      let node=hit;
      while(node){if(node===t.group)return true;node=node.parent;}
      return false;
    });
    if(!found){hideTagInfoPanel();return;}
    state.selectedTagPopupIndex=Number(found.group.userData.index);
    $("tagInfoPanel").classList.remove("hidden");
    updateTagInfoPanel();
    updateTagInfoPanelPosition();
  });
  $("tagInfoClose")?.addEventListener("click",hideTagInfoPanel);
  $("statsTagSelect")?.addEventListener("change",renderStatsDashboard);
  setInterval(()=>{
    if(state.monitoringActive||state.simOn)captureMonitoringSamples(Date.now());
    if(state.workspaceView==="stats")renderStatsDashboard();
    updateTagInfoPanel();
  },1000);

  function renderPlacementCoordinates(){
    const host=$("placementCoordinateList");
    if(!host)return;

    const entries=[
      ...state.selected.tags.map((c,i)=>({type:"tag",index:i+1,candidate:c})),
      ...state.selected.anchors.map((c,i)=>({type:"anchor",index:i+1,candidate:c}))
    ];

    if(!entries.length){
      host.className="placementCoordinateList emptyText";
      host.textContent="AI 추천이 확정되면 Tag와 Anchor의 절대좌표가 표시됩니다.";
      return;
    }

    const metric=Boolean(state.modelMeta?.unitsToMeters===1 || state.modelMeta?.exactUnits);
    const unit=metric?"m":"model unit";
    const digits=metric?3:4;
    host.className="placementCoordinateList";
    host.innerHTML=entries.map(({type,index,candidate:c})=>`
      <div class="coordinateItem ${type}">
        <div class="coordinateHead">
          <strong>${type==="tag"?"Tag":"Anchor"} ${index}</strong>
          <span>${escapeHtml(c.id)} · 모델 기준 절대좌표</span>
        </div>
        <div class="coordinateAxes">
          <div class="coordinateAxis"><span>X</span><b>${Number(c.x).toFixed(digits)} ${unit}</b></div>
          <div class="coordinateAxis"><span>Y</span><b>${Number(c.y).toFixed(digits)} ${unit}</b></div>
          <div class="coordinateAxis"><span>Z</span><b>${Number(c.z).toFixed(digits)} ${unit}</b></div>
        </div>
        ${type==="tag"?`<div style="margin-top:5px;font-size:8px;color:#7d8d95">Surface ${escapeHtml(c.surfaceClass||c.surface||"unknown")} · n=(${Number(c.nx??0).toFixed(2)}, ${Number(c.ny??1).toFixed(2)}, ${Number(c.nz??0).toFixed(2)})</div>`:""}
      </div>`).join("");
  }

  function renderSensors(){
    renderPlacementCoordinates();
    const list=$("sensorList"); if(list) list.innerHTML="";
    let warnings=0;
    state.sensorObjects.forEach(o=>{
      const d=o.group.userData;
      if(!d)return;
      const st=d.type==="tag"?classification(o):"normal";
      if(st!=="normal")warnings++;
      const isDanger=d.type==="tag"&&st==="danger";
      const isWarning=d.type==="tag"&&st==="warning";
      const dangerAura=o.body?.userData?.dangerAura;
      const dangerAuraOuter=o.body?.userData?.dangerAuraOuter;
      const dangerBadge=o.body?.userData?.dangerBadge;
      if(dangerAura)dangerAura.visible=isDanger;
      if(dangerAuraOuter)dangerAuraOuter.visible=isDanger;
      if(dangerBadge)dangerBadge.visible=isDanger;
      if(o.body?.userData)o.body.userData.dangerActive=isDanger;
      const markerPoint=o.body?.userData?.point;
      const markerHalo=o.body?.userData?.halo;
      if(markerPoint?.material){
        const normalAccent=d.type==="tag"?0x35a36b:0x3b82f6;
        const activeAccent=isDanger?0xe32626:isWarning?0xf3a623:normalAccent;
        markerPoint.material.color.setHex(activeAccent);
        markerPoint.material.emissive.setHex(activeAccent);
        markerPoint.material.emissiveIntensity=isDanger?.65:isWarning?.22:.07;
        markerPoint.material.opacity=isDanger?1:.86;
        markerPoint.material.needsUpdate=true;
        if(markerHalo?.material){
          markerHalo.material.color.setHex(activeAccent);
          markerHalo.material.opacity=isDanger?.46:isWarning?.25:.13;
          markerHalo.material.needsUpdate=true;
        }
      }
      const item=document.createElement("div");item.className="sensorItem";
      item.innerHTML=`
        <div class="sensorHead"><strong>${d.type==="tag"?"Tag":"Anchor"} ${d.index}<span style="display:block;font-size:7.8px;color:#87959d;font-weight:500;margin-top:2px">${mappedModule(d.type,d.index)||"미매핑"}${d.liveOnline?" · 수신중":""}</span></strong><span class="sensorState">${d.liveOnline?"실시간 · ":""}${st==="unknown"?"판단 보류":st==="normal"?"정상":st==="warning"?"주의":"위험"}</span></div>
        ${d.type==="tag"?`<div class="sensorVals">
          <div class="sensorVal"><span>Δn</span><b>${!state.simOn&&!d.measurementValid?"—":d.zMm.toFixed(1)+" mm"}</b></div>
          <div class="sensorVal"><span>Tilt X</span><b>${state.simOn||d.imuOk?d.tilt.toFixed(1)+"°":"IMU 오류"}</b></div>
          <div class="sensorVal"><span>진동 / UWB</span><b>${state.simOn||d.imuOk?(d.vibration?"감지":"없음"):"IMU 오류"} · ${d.rangeCount??0}개</b></div>
        </div>
        <div style="margin-top:5px;font-size:8px;color:${st==="danger"?"#d33b3b":"#819099"}">위험 기준 · |Δn| ≥ ${RISK_LIMITS.dangerDzMm} mm · |Tilt X| ≥ ${RISK_LIMITS.dangerTiltDeg}° · 동적가속도 ≥ ${RISK_LIMITS.dangerVibrationG.toFixed(2)} g</div>
        <div style="margin-top:6px;font-size:8.2px;color:#819099;line-height:1.45">
          ${d.liveOnline ? `RSSI ${Number.isFinite(d.wifiRssi)?d.wifiRssi:"—"} dBm` : "실시간 데이터 대기"}
          ${d.batteryV > 0 ? ` · 배터리 ${d.batteryV.toFixed(2)} V` : ""}
          ${d.dynamicG > 0 ? ` · 동적가속도 ${d.dynamicG.toFixed(3)} g` : ""}
          ${d.liveOnline ? ` · IMU ${d.imuOk?"정상":"연결 오류"}` : ""}
          ${d.liveOnline && d.imuFailures>0 ? ` · I²C 실패 ${d.imuFailures}회` : ""}
        </div>`:`<div class="sensorVals"><div class="sensorVal"><span>노드</span><b>고정</b></div><div class="sensorVal"><span>UWB</span><b>기준점</b></div><div class="sensorVal"><span>ID</span><b>${d.candidateId}</b></div></div>`}
      `;
      if(list) list.appendChild(item);
    });
    $("warningCount").textContent=warnings;
    const unknown=tagSensors().some(o=>classification(o)==='unknown');
    const health=Math.max(0,100-warnings*9);
    $("healthPct").textContent=unknown?"—":health;
    $("healthRing").style.background=`conic-gradient(${warnings?"var(--warn)":"var(--good)"} 0 ${health}%,#e6ecef ${health}% 100%)`;
    updateDangerToast();
  }

  let dangerToastHideTimer=null;
  function updateDangerToast(){
    const toast=$("dangerToast");
    const list=$("dangerToastList");
    const meta=$("dangerToastMeta");
    if(!toast||!list)return;

    const dangers=tagSensors().map(tag=>{
      const d=tag.group.userData;
      if(classification(tag)!=="danger")return null;
      const items=riskItemsForData(d,"danger");
      if(!items.length)return null;
      return {index:Number(d.index),items,d};
    }).filter(Boolean);

    if(dangers.length){
      if(dangerToastHideTimer){clearTimeout(dangerToastHideTimer);dangerToastHideTimer=null;}
      list.innerHTML=dangers.map(({index,items,d})=>{
        const detail=[];
        if(items.includes("변위"))detail.push(`${Number(d.zMm||0).toFixed(1)} mm`);
        if(items.includes("기울기"))detail.push(`${Number(d.tilt||0).toFixed(1)}°`);
        if(items.includes("진동"))detail.push(`${Number(d.dynamicG||0).toFixed(3)} g`);
        return `<div class="dangerToastRow"><strong>Tag ${index}</strong><span>${escapeHtml(items.join(" · "))}${detail.length?` <small>${escapeHtml(detail.join(" / "))}</small>`:""}</span></div>`;
      }).join("");
      if(meta)meta.textContent=`${new Date().toLocaleTimeString()} · ${dangers.length}개 Tag 위험`;
      toast.classList.remove("hidden");
      document.body.classList.add("danger-toast-visible");
      return;
    }

    // Keep the last danger visible briefly so momentary vibration events are readable.
    if(!toast.classList.contains("hidden")&&!dangerToastHideTimer){
      dangerToastHideTimer=setTimeout(()=>{
        toast.classList.add("hidden");
        document.body.classList.remove("danger-toast-visible");
        dangerToastHideTimer=null;
      },4500);
    }
  }

  function smoothstep(edge0,edge1,x){
    const t=Math.max(0,Math.min(1,(x-edge0)/Math.max(edge1-edge0,1e-6)));
    return t*t*(3-2*t);
  }

  function softVisualMm(mm){
    const v=Number(mm||0);
    const limit=60;
    return limit*Math.tanh(v/limit);
  }

  function deformationVisualScale(maxDim){
    // Viewer-only displacement gain. Reduced so small UWB changes no longer look
    // like large structural deformation. Measurement values themselves are untouched.
    if(state.modelMeta?.unitsToMeters===1 || state.modelMeta?.exactUnits)return VISUAL_GAINS.displacementMetric;
    return Math.max(maxDim/VISUAL_GAINS.displacementRelativeDivisor,1e-5);
  }

  function candidateFrame(base){
    const n=new THREE.Vector3(Number(base?.nx??0),Number(base?.ny??1),Number(base?.nz??0));
    if(n.lengthSq()<1e-8)n.set(0,1,0);n.normalize();
    let u=new THREE.Vector3(Number(base?.tux??1),Number(base?.tuy??0),Number(base?.tuz??0));
    if(u.lengthSq()<1e-8||Math.abs(u.clone().normalize().dot(n))>.96)u=surfaceBasis(n).u;else u.normalize();
    const v=n.clone().cross(u).normalize();
    const p=new THREE.Vector3(Number(base?.surfaceX??base?.x??0),Number(base?.surfaceY??base?.y??0),Number(base?.surfaceZ??base?.z??0));
    return {n,u,v,p};
  }

  function bboxSpanAlong(axis,bbox){
    const size=bbox.getSize(new THREE.Vector3());
    return Math.abs(axis.x)*size.x+Math.abs(axis.y)*size.y+Math.abs(axis.z)*size.z;
  }

  function deformationFrame(base){
    const cf=candidateFrame(base);
    const bbox=state.surfaceAnalysis?.bbox||modelBoundingBox();
    const worldAxes=[
      new THREE.Vector3(1,0,0),
      new THREE.Vector3(0,1,0),
      new THREE.Vector3(0,0,1)
    ];

    // Use the longest model direction lying on the installation surface as the
    // structural longitudinal axis. For the supplied bridge GLB this resolves to X,
    // allowing Left/Center/Right span meshes to share the same bending curve.
    let longAxis=null,longScore=-1;
    worldAxes.forEach(a=>{
      const proj=a.clone().addScaledVector(cf.n,-a.dot(cf.n));
      const len=proj.length();
      if(len<1e-5)return;
      proj.multiplyScalar(1/len);
      const score=bboxSpanAlong(proj,bbox);
      if(score>longScore){longScore=score;longAxis=proj;}
    });
    if(!longAxis)longAxis=cf.u.clone();
    const transverse=cf.n.clone().cross(longAxis).normalize();
    if(transverse.lengthSq()<1e-8)transverse.copy(cf.v);

    const longSpan=Math.max(bboxSpanAlong(longAxis,bbox),.01);
    const transSpan=Math.max(bboxSpanAlong(transverse,bbox),.01);
    const normalSpan=Math.max(bboxSpanAlong(cf.n,bbox),.01);
    const caseLong=Math.max(Number(base?.caseWidth||.07),Number(base?.caseDepth||.06));
    const caseTrans=Math.min(Number(base?.caseWidth||.07),Number(base?.caseDepth||.06));

    // Long influence reaches through adjacent GLB span meshes, while the transverse
    // radius moves most of the deck width together. The normal band keeps lower fixed
    // support regions from following the deck rigidly.
    const radiusLong=Math.max(longSpan*.52,caseLong*5.0,longSpan*.24);
    const radiusTrans=Math.max(transSpan*1.50,caseTrans*5.0,transSpan*.65);
    const normalBand=Math.max(normalSpan*.35,Number(base?.caseHeight||.047)*2.2,.055);

    return {...cf,longAxis,transverse,longSpan,transSpan,normalSpan,radiusLong,radiusTrans,normalBand};
  }

  function deformationKernel(frame,world){
    const rel=world.clone().sub(frame.p);
    const localLong=rel.dot(frame.longAxis);
    const localTrans=rel.dot(frame.transverse);
    const localN=rel.dot(frame.n);
    if(Math.abs(localN)>=frame.normalBand)return null;

    const r2=(localLong/frame.radiusLong)**2+(localTrans/frame.radiusTrans)**2;
    if(r2>=1)return null;
    const r=Math.sqrt(Math.max(0,r2));

    // Central plateau keeps the Tag physically attached to the deformed surface.
    // A raised-cosine tail then returns smoothly toward zero at the influence edge.
    const tangentWeight=r<=.25
      ?1
      :.5*(1+Math.cos(Math.PI*((r-.25)/.75)));

    // Move the full deck thickness together and fade only deeper support/body regions.
    const depthWeight=1-smoothstep(frame.normalBand*.70,frame.normalBand,Math.abs(localN));
    const weight=tangentWeight*depthWeight;
    if(weight<=1e-6)return null;
    return {weight,localLong,localTrans,localN};
  }

  function boxCanReceiveTag(target,frame){
    if(!target.worldBox)return true;
    const c=target.worldCenter;
    const half=target.worldSize.clone().multiplyScalar(.5);
    const rel=c.clone().sub(frame.p);
    const projectedHalf=axis=>Math.abs(axis.x)*half.x+Math.abs(axis.y)*half.y+Math.abs(axis.z)*half.z;
    const dLong=Math.max(0,Math.abs(rel.dot(frame.longAxis))-projectedHalf(frame.longAxis));
    const dTrans=Math.max(0,Math.abs(rel.dot(frame.transverse))-projectedHalf(frame.transverse));
    const dNormal=Math.max(0,Math.abs(rel.dot(frame.n))-projectedHalf(frame.n));
    return dNormal<frame.normalBand &&
      (dLong/frame.radiusLong)**2+(dTrans/frame.radiusTrans)**2<1;
  }

  function deformationVectorAt(world,frames,visualScale,out){
    out.set(0,0,0);
    let totalWeight=0;
    for(const {frame,data} of frames){
      const rx=world.x-frame.p.x, ry=world.y-frame.p.y, rz=world.z-frame.p.z;
      const localLong=rx*frame.longAxis.x+ry*frame.longAxis.y+rz*frame.longAxis.z;
      const localTrans=rx*frame.transverse.x+ry*frame.transverse.y+rz*frame.transverse.z;
      const localN=rx*frame.n.x+ry*frame.n.y+rz*frame.n.z;
      if(Math.abs(localN)>=frame.normalBand)continue;
      const r2=(localLong/frame.radiusLong)**2+(localTrans/frame.radiusTrans)**2;
      if(r2>=1)continue;
      const r=Math.sqrt(Math.max(0,r2));
      const tangentWeight=r<=.25?1:.5*(1+Math.cos(Math.PI*((r-.25)/.75)));
      const depthWeight=1-smoothstep(frame.normalBand*.70,frame.normalBand,Math.abs(localN));
      const weight=tangentWeight*depthWeight;
      if(weight<=1e-6)continue;
      const normalDisp=softVisualMm(data.zMm)*visualScale*weight;
      const tiltRad=THREE.MathUtils.degToRad(THREE.MathUtils.clamp(Number(data.tilt||0),-8,8)*VISUAL_GAINS.tilt);
      const amount=normalDisp+Math.tan(tiltRad)*localTrans*weight;
      out.x+=frame.n.x*amount;out.y+=frame.n.y*amount;out.z+=frame.n.z*amount;
      totalWeight+=weight;
    }
    if(totalWeight>1)out.multiplyScalar(1/totalWeight);
    return out;
  }

  const _deformWorld=new THREE.Vector3();
  const _deformDelta=new THREE.Vector3();
  function deformDeck(){
    const tags=tagSensors();
    if(!tags.length||!state.deformTargets.length)return;

    const bbox=state.surfaceAnalysis?.bbox||modelBoundingBox();
    const sz=state.surfaceAnalysis?.size||bbox.getSize(new THREE.Vector3());
    const visualScale=deformationVisualScale(Math.max(sz.x,sz.y,sz.z,.001));

    const frames=tags.map(t=>{
      const d=t.group.userData;
      const base=state.selected.tags[d.index-1];
      return base?{tag:t,data:d,base,frame:deformationFrame(base)}:null;
    }).filter(Boolean);
    if(!frames.length)return;

    // Mesh transforms stay at their authored locations. Every frame is rebuilt from
    // immutable source vertices so deformation never accumulates or explodes over time.
    state.deformTargets.forEach(target=>{
      target.mesh.position.copy(target.basePosition);
      target.mesh.quaternion.copy(target.baseQuaternion);
    });
    state.modelGroup.updateMatrixWorld(true);

    state.deformTargets.forEach(target=>{
      const mesh=target.mesh;
      if(!target.original)return;
      const activeFrames=frames.filter(x=>boxCanReceiveTag(target,x.frame));
      const pos=mesh.geometry.attributes.position;
      const arr=pos.array,original=target.original;

      if(!activeFrames.length){
        let changed=false;
        for(let i=0;i<arr.length;i++)if(arr[i]!==original[i]){changed=true;break;}
        if(changed){
          arr.set(original);
          pos.needsUpdate=true;
          mesh.geometry.computeVertexNormals();
          mesh.geometry.computeBoundingSphere?.();
        }
        return;
      }

      const worldMatrix=mesh.matrixWorld.clone();
      const inv=new THREE.Matrix4().copy(worldMatrix).invert();
      let maxDeltaSq=0;
      for(let i=0;i<pos.count;i++){
        const k=i*3;
        _deformWorld.set(original[k],original[k+1],original[k+2]).applyMatrix4(worldMatrix);
        deformationVectorAt(_deformWorld,activeFrames,visualScale,_deformDelta);
        maxDeltaSq=Math.max(maxDeltaSq,_deformDelta.lengthSq());
        _deformWorld.add(_deformDelta).applyMatrix4(inv);
        arr[k]=_deformWorld.x;arr[k+1]=_deformWorld.y;arr[k+2]=_deformWorld.z;
      }
      pos.needsUpdate=true;
      if(maxDeltaSq>1e-12){
        mesh.geometry.computeVertexNormals();
        // The displacement is only a few cm on a model whose bounding volume is
        // much larger. Recomputing every bounding sphere on every sensor packet
        // was unnecessary and caused visible main-thread stalls.
      }
    });

    // The sensor marker follows exactly the same local structure displacement.
    frames.forEach(({tag:t,data:d,frame})=>{
      t.group.position.copy(d.basePosition).addScaledVector(frame.n,softVisualMm(d.zMm)*visualScale);
      t.group.quaternion.copy(d.baseQuaternion||new THREE.Quaternion());
      if(t.body){
        const tiltQ=new THREE.Quaternion().setFromAxisAngle(
          frame.longAxis,
          THREE.MathUtils.degToRad(THREE.MathUtils.clamp(Number(d.tilt||0),-8,8)*VISUAL_GAINS.tilt)
        );
        t.body.quaternion.copy(tiltQ.multiply((d.baseBodyQuaternion||new THREE.Quaternion()).clone()));
      }
    });
  }

  function resetDeckGeometry(){
    state.deformTargets.forEach(target=>{
      const mesh=target.mesh;
      mesh.position.copy(target.basePosition);
      mesh.quaternion.copy(target.baseQuaternion);
      if(target.original && mesh.geometry?.attributes?.position){
        const pos=mesh.geometry.attributes.position;
        pos.array.set(target.original);
        pos.needsUpdate=true;
        mesh.geometry.computeVertexNormals();
        mesh.geometry.computeBoundingSphere?.();
      }
    });
    tagSensors().forEach((t,i)=>{
      const base=state.selected.tags[i];
      const d=t.group.userData;
      if(base)t.group.position.set(base.x,base.y,base.z);
      t.group.quaternion.copy(d?.baseQuaternion || new THREE.Quaternion());
      if(t.body)t.body.quaternion.copy(d?.baseBodyQuaternion || new THREE.Quaternion());
    });
  }

  function updateTrend(){
    const tags=tagSensors();
    if(!state.simOn && tags.some(t=>!t.group.userData.liveOnline||!t.group.userData.measurementValid))return;
    const avg=tags.length?tags.reduce((a,t)=>a+t.group.userData.zMm,0)/tags.length:0;
    state.history.push(Math.max(10,Math.min(100,55-avg*4)));
    if(state.history.length>11)state.history.shift();
    $("trendLine").setAttribute("points",state.history.map((y,i)=>`${i*70},${y}`).join(" "));
  }

  function setSimulation(on){
    state.simOn=on;
    $("simBtn").textContent=on?"시뮬레이션 정지":"센서 시뮬레이션";
    $("twinMode").textContent=on?"실시간 테스트":"기하 분석";
    if(on){
      state.simTimer=setInterval(()=>{
        state.simT+=.14;
        const ts=tagSensors();
        ts.forEach((o,i)=>{
          const d=o.group.userData;
          const u=(i+1)/(ts.length+1);
          const shape=.52+.48*Math.sin(Math.PI*u);
          const amp=shape*7.2;
          d.zMm=-(amp*(.58+.42*Math.sin(state.simT*.8+i*.51)));
          d.tilt=amp*.42*Math.sin(state.simT*.55+i*.77);
          d.vibration=Math.sin(state.simT*2.7+i*1.43)>.94?1:0;
        });
        deformDeck();renderSensors();updateTrend();
      },160);
      addEvent("Sensor simulation started","INFO");
    }else{
      clearInterval(state.simTimer);state.simTimer=null;addEvent("Sensor simulation stopped","INFO");
    }
  }
  $("simBtn").onclick=()=>setSimulation(!state.simOn);
  async function performZeroCalibration(){
    if(state.simOn)setSimulation(false);
    const hasLiveMapped=Object.values(state.moduleMappings).some(id=>state.liveSensorStates[id]?.online)
      || Object.values(state.moduleMappings).some(id=>state.discoveredModules.some(m=>m.node_id===id&&m.online));
    if(hasLiveMapped){
      try{
        const response=await fetch("/api/sensors/zero",{method:"POST"});
        if(!response.ok)throw new Error("Zero failed");
        const result=await response.json();
        state.calibrationGeneration=result.calibration_generation;
        tagSensors().forEach((o,i)=>{
          const d=o.group.userData;d.measurementValid=false;d.zMm=0;d.tilt=0;d.vibration=0;
          const base=state.selected.tags[i];
          if(base)o.group.position.set(base.x,base.y,base.z);
        });
        resetDeckGeometry();renderSensors();updateTrend();
        toast(state.profile?.exactUnits
          ? "UWB 절대좌표 Pair 보정을 다시 시작했습니다. 잠시 구조물을 고정하세요."
          : "실센서 UWB 기준거리와 기울기 0점을 설정했습니다");
        addEvent(state.profile?.exactUnits?"UWB Pair Calibration 재시작":"실센서 기준값 Zero 적용","INFO");
        return true;
      }catch(err){
        toast("실센서 Zero 명령 실패");
        return false;
      }
    }else{
      tagSensors().forEach((o,i)=>{
        const d=o.group.userData;d.measurementValid=false;d.zMm=0;d.tilt=0;d.vibration=0;
        const base=state.selected.tags[i];
        if(base)o.group.position.set(base.x,base.y,base.z);
      });
      resetDeckGeometry();renderSensors();updateTrend();toast("센서값을 0으로 설정했습니다");
      return true;
    }
  }
  $("zeroBtn").onclick=performZeroCalibration;

  // ---------- Tabs/tools ----------
  function switchTab(tab){
    // keeps AI results and UWB details in separate chat-style views.
    // Preserve the legacy call sites without hiding either functional panel.
    if(tab==="live") renderRawLiveTag();
  }
  $("resetViewBtn").onclick=fitCamera;
  let wire=false;
  $("wireBtn").onclick=()=>{
    wire=!wire;
    state.modelGroup?.traverse(ch=>{if(ch.isMesh&&ch.material&&!Array.isArray(ch.material))ch.material.wireframe=wire});
    toast(wire?"Wireframe on":"Wireframe off");
  };
  $("gridBtn").onclick=()=>{grid.visible=!grid.visible;toast(grid.visible?"Grid on":"Grid off")};
  $("analyticsHandle").onclick=()=>{
    const app = document.querySelector(".app");
    const open = !app.classList.contains("analytics-open");
    app.classList.toggle("analytics-open", open);
    $("analytics").classList.toggle("open", open);
    setTimeout(resize, 240);
  };

  // ---------- API status ----------
  initSensorSetupDrag();
  initSensorSetupCollapse();
  setFlowStage("upload");
  setWorkspaceView("viewer");
  if(window.innerWidth<=820)setSidebarCollapsed(true);
  startSensorStream();
  refreshDiscoveredModules(false);

  fetch("/api/status").then(r=>r.json()).then(s=>{
    state.aiEnabled=Boolean(s.aiEnabled);
    $("apiDot").classList.toggle("on",state.aiEnabled);
    $("apiStatus").textContent=state.aiEnabled?`AI 준비 완료 · ${s.model}`:"데모 AI · API 키 없음";
  }).catch(()=>{
    $("apiStatus").textContent="서버 상태 확인 실패";
  });

})();
