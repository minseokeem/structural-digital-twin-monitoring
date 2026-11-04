(function(root){
  class StaticCheck {
    constructor(){this.reset();}
    reset(){this.status='idle';this.rows=[];this.seen=new Set();this.result=null;this.lastPacket=null;this.maxGap=0;}
    start(now){this.reset();this.status='waiting';this.requestedAt=now;}
    cancel(){this.status='cancelled';}
    push(raw,now){
      if(!['waiting','running'].includes(this.status))return;
      if(this.status==='waiting'){
        if(now-this.requestedAt>300000){this.status='timeout';return;}
        if(!raw?.online||!raw.measurement_valid||!raw.uwb_calibration?.ready||!raw.averaging_ready)return;
        this.status='running';this.startedAt=now;this.calibration=raw.uwb_calibration.id;
        this.mode=raw.uwb_filter?.mode;this.node=raw.node_id;this.boot=raw.boot_id;
      }
      if(raw.node_id!==this.node||raw.boot_id!==this.boot||raw.uwb_filter?.mode!==this.mode||raw.uwb_calibration?.id!==this.calibration){this.status='interrupted';return;}
      const key=`${raw.boot_id}:${raw.seq}:${raw.last_seen_ms}`;
      if(this.seen.has(key))return;
      this.seen.add(key);
      if(this.lastPacket!=null)this.maxGap=Math.max(this.maxGap,now-this.lastPacket);
      this.lastPacket=now;
      this.rows.push({elapsed_ms:now-this.startedAt,raw:JSON.parse(JSON.stringify(raw))});
      if(now-this.startedAt>=60000)this.finish(now);
    }
    tick(now){
      if(this.status==='waiting'&&now-this.requestedAt>300000)this.status='timeout';
      if(this.status==='running'&&now-this.startedAt>=60000)this.finish(now);
    }
    finish(now){
      if(this.status!=='running')return;
      this.maxGap=Math.max(this.maxGap,now-(this.lastPacket??this.startedAt));
      const valid=this.rows.filter(x=>x.raw.online&&x.raw.measurement_valid&&x.raw.uwb_calibration?.ready&&x.raw.averaging_ready&&Number.isFinite(x.raw.dz_mm));
      const values=valid.map(x=>x.raw.dz_mm);
      const mean=values.length?values.reduce((a,b)=>a+b,0)/values.length:null;
      const min=values.length?Math.min(...values):null,max=values.length?Math.max(...values):null;
      const maxAbs=values.length?Math.max(...values.map(Math.abs)):null;
      const enough=values.length>=100&&valid.length>=this.rows.length*.95&&this.maxGap<=1500;
      const movement=this.rows.some(x=>x.raw.vibration);
      const pass=enough&&!movement&&maxAbs<=5;
      this.result={pass,enough,movement,min_mm:min,max_mm:max,max_abs_mm:maxAbs,mean_mm:mean,
        rmse_mm:values.length?Math.sqrt(values.reduce((a,b)=>a+b*b,0)/values.length):null,
        within_5mm_percent:values.length?100*values.filter(x=>Math.abs(x)<=5).length/values.length:null,
        samples:values.length,total_samples:this.rows.length,max_gap_ms:this.maxGap};
      this.status='done';
    }
  }
  if(typeof module==='object'&&module.exports)module.exports={StaticCheck};
  if(!root?.document)return;
  root.StaticCheck=StaticCheck;
  const $=id=>root.document.getElementById(id),check=new StaticCheck();
  let latest=null;
  function render(){
    $('staticZeroBtn').disabled=!!$('zeroBtn')?.disabled;
    $('staticTestBtn').textContent=['waiting','running'].includes(check.status)?'검사 취소':'정지 60초 검사';
    $('staticDownloadBtn').disabled=!check.rows.length;
    const host=$('staticTestStatus');host.dataset.result='';
    if(check.status==='waiting')host.textContent='평균 수집 완료 후 검사 시작 · 계속 원위치에 고정하세요';
    else if(check.status==='running')host.textContent=`정지 검사 ${Math.min(60,Math.floor((Date.now()-check.startedAt)/1000))}/60초 · Zero를 누르지 마세요`;
    else if(check.status==='done'){
      const r=check.result;host.dataset.result=r.pass?'pass':'fail';
      host.textContent=!r.enough?'판정 불가 · 수신 누락이 많습니다':r.movement?'판정 불가 · 진동이 감지되었습니다':
        `${r.pass?'이번 60초 목표 충족':'±5 mm 목표 미달'} · ${r.min_mm.toFixed(1)} ~ ${r.max_mm.toFixed(1)} mm`;
    }else if(check.status==='interrupted')host.textContent='검사 중단 · 보정·모드·연결이 바뀌었습니다';
    else if(check.status==='timeout')host.textContent='준비 시간 초과 · 연결과 보정 상태를 확인하세요';
    else if(check.status==='cancelled')host.textContent='검사를 취소했습니다';
    else host.textContent='원위치에 정지시킨 뒤 검사하세요. 목표: 모든 유효값 −5 ~ +5 mm';
  }
  $('staticTestBtn').onclick=()=>{
    if(['waiting','running'].includes(check.status))check.cancel();
    else {check.start(Date.now());if(latest)check.push(latest,Date.now());}
    render();
  };
  $('staticZeroBtn').onclick=()=>root.document.getElementById('zeroBtn').click();
  root.addEventListener('uwb-snapshot',e=>{
    latest=e.detail.sensors?.find(x=>x.node_id==='TAG-01')||e.detail.sensors?.find(x=>x.role==='tag');
    if(latest){check.push(latest,Date.now());render();}
  });
  root.addEventListener('uwb-disconnected',()=>{if(check.status==='running')check.status='interrupted';render();});
  setInterval(()=>{check.tick(Date.now());render();},500);
  $('staticDownloadBtn').onclick=()=>{
    const quote=x=>`"${String(x??'').replaceAll('"','""')}"`;
    const lines=[['elapsed_ms','node_id','boot_id','seq','calibration_id','mode','valid','displacement_mm','raw_geometry_mm','within_5mm','reference_only','anchor_id','raw_range_m','bias_m','filtered_range_m'].join(',')];
    for(const {elapsed_ms,raw:r} of check.rows)for(const a of r.ranges?.length?r.ranges:[{}]){
      lines.push([elapsed_ms,r.node_id,r.boot_id,r.seq,r.uwb_calibration?.id,r.uwb_filter?.mode,r.measurement_valid,r.dz_mm,r.raw_dz_mm,
        r.measurement_valid&&r.dz_mm!=null?Math.abs(r.dz_mm)<=5:null,r.reference_only,a.anchor_id,a.m,a.bias_m,a.corrected_m].map(quote).join(','));
    }
    const url=URL.createObjectURL(new Blob(['\ufeff'+lines.join('\r\n')],{type:'text/csv;charset=utf-8'}));
    const a=root.document.createElement('a');a.href=url;a.download=`SOC_STATIC_${Date.now()}.csv`;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
  };
  render();
})(typeof window==='object'?window:null);
