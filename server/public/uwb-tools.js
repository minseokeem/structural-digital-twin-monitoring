(() => {
  const $=id=>document.getElementById(id);
  let recording=false,rows=[],started=0,truth=null,name='',seen=new Map(),layout=null;
  const finite=x=>x!=null && Number.isFinite(Number(x));
  const labels={stale_data:'새 거리 데이터 없음',no_range:'UWB 거리 없음',coordinates_required:'실제 좌표와 단위 확인 필요',
    filter_warmup:'거리 필터 준비 중',calibrating_metric_pairs:'정지 상태 보정 중',insufficient_geometry:'유효 Anchor 부족',
    weak_vertical_geometry:'법선 감도가 부족합니다. Tag의 Surface normal과 Anchor 배치를 확인하세요.',
    weak_normal_geometry:'법선 감도가 부족합니다. Tag의 Surface normal과 Anchor 배치를 확인하세요.',
    inconsistent_ranges:'거리들이 서로 일치하지 않아 측정을 보류합니다.'};
  window.addEventListener('uwb-disconnected',()=>{
    $('uwbQuality').textContent='서버 연결 끊김 · 현재 변위를 확인할 수 없습니다. 3D는 마지막 유효 자세입니다.';
    $('uwbQuality').dataset.level='warn';
  });
  window.addEventListener('uwb-snapshot',e=>{
    const tags=(e.detail.sensors||[]).filter(x=>x.role==='tag').sort((a,b)=>String(a.node_id).localeCompare(String(b.node_id)));
    if(!tags.length)return;
    const primary=tags.find(x=>x.node_id==='TAG-01')||tags[0];
    if(primary.uwb_filter)showMode(primary.uwb_filter);
    const onlineCount=tags.filter(x=>x.online).length,validCount=tags.filter(x=>x.online&&x.measurement_valid).length;
    const calibrating=tags.find(x=>x.dz_method==='calibrating_metric_pairs');
    $('uwbQuality').dataset.level=validCount<tags.length?'warn':'ok';
    $('uwbQuality').textContent=calibrating?
      `전체 Tag 기록 · 보정 중 ${calibrating.node_id} ${Math.round(100*(calibrating.uwb_calibration?.progress||0))}%`:
      `전체 Tag 기록 · 온라인 ${onlineCount}/${tags.length} · 유효 변위 ${validCount}/${tags.length}`;
    if(!recording)return;
    for(const raw of tags){
      const key=`${raw.boot_id||''}:${raw.seq}:${raw.last_seen_ms}:${raw.last_measurement_ms||''}`;
      if(seen.get(raw.node_id)===key)continue;
      seen.set(raw.node_id,key);
      rows.push({t:(Date.now()-started)/1000,raw:structuredClone(raw)});
    }
    $('uwbRecordStatus').textContent=`전체 Tag ${rows.length}개 packet 기록 · ${((Date.now()-started)/1000).toFixed(1)}초`;
    if(Date.now()-started>=600000||rows.length>=30000){recording=false;$('uwbRecord').disabled=false;$('uwbRecordStatus').textContent+=' · 기록 완료, CSV를 저장하세요';}
  });
  function showMode(config){
    const win=Math.round(Number(config?.window_ms||1200));
    $('liveFilterSummary').textContent=`Unified Adaptive · 자동 · 최대 ${win} ms window`;
    $('uwbFilterNote').textContent='단일 estimator 상태를 계속 유지합니다. IMU/UWB motion score가 커질수록 시간창과 process uncertainty가 연속적으로 바뀌며, 필요한 Anchor 수는 posterior uncertainty로 자동 결정됩니다.';
  }
  fetch('/api/sensors/filter').then(r=>{if(!r.ok)throw Error('계측 설정 조회 실패');return r.json()}).then(showMode).catch(()=>{});
  $('uwbRecord').onclick=()=>{
    if(rows.length && !confirm('이전 기록을 새 기록으로 바꿀까요? 저장하지 않았다면 먼저 CSV를 저장하세요.'))return;
    rows=[];seen.clear();started=Date.now();recording=true;
    name=$('uwbExperiment').value||'UWB';truth=$('uwbTruth').value.trim()===''?null:Number($('uwbTruth').value);
    $('uwbRecord').disabled=true;$('uwbDownload').disabled=false;$('uwbRecordStats').textContent='';
  };
  $('uwbDownload').onclick=()=>{
    recording=false;$('uwbRecord').disabled=false;
    if(!rows.length){$('uwbRecordStatus').textContent='수신된 기록이 없습니다.';return;}
    const quote=x=>`"${String(x??'').replaceAll('"','""')}"`;
    const header=[
      'experiment','elapsed_s','server_ms','node_id','boot_id','seq','truth_mm','online','valid','method',
      'delta_n_mm','raw_geometry_mm','posterior_sigma_mm','residual_rms_mm','normal_sensitivity',
      'velocity_mm_s','motion_score','zero_velocity_active','quiet_duration_ms','velocity_leak',
      'verification_only','residual_baseline_frozen','displacement_consensus_confirmed','displacement_consensus_count','displacement_consensus_mm','displacement_consensus_spread_mm','residual_baseline_mm','quiet_residual_limit_mm','consensus_rejected',
      'inlier_count','outlier_count','inlier_anchors','outlier_anchors','quarantined_anchors','reacquired','reacquire_target_mm',
      'window_samples','window_span_ms','effective_window_ms','process_sigma_d_mm','process_sigma_v_mm_s',
      'tilt_deg','vibration','dynamic_g','ax_g','ay_g','az_g','imu_ok','wifi_rssi',
      'range_count','requested_links','anchor_mask','adaptive_reason','adaptive_event',
      'anchor_id','scheduled','ok','clock_offset_ppm','raw_range_m','corrected_range_m','instant_corrected_m','bias_m','noise_m',
      'measurement_sigma_m','anchor_residual_m','information_h','information_gain','robust_weight','age_weight',
      'repetition_weight','total_weight','hard_rejected','quarantined','quarantine_until_ms','candidate_step_m','innovation_m','outlier','calibrated','success_rate','attempt_success_rate','attempts','successes',
      'nlos_score','nlos_suspect','fp_amp','peak_amp','fp_peak_gap','fp_ratio','diag_fresh','reference_only','quality_warning'
    ];
    const lines=[header.join(',')];
    for(const {t,raw:r} of rows){
      const ranges=(r.ranges?.length?r.ranges:[{}]);
      for(const a of ranges){
        const detail=(r.uwb_link_details||[]).find(x=>x.anchor_id===a.anchor_id)||{};
        const sigma=finite(a.sigma_m)?Number(a.sigma_m):(finite(detail.sigma_m)?Number(detail.sigma_m):null);
        const h=finite(a.information_h)?Number(a.information_h):(finite(detail.information_h)?Number(detail.information_h):null);
        const info=finite(a.information_gain)?Number(a.information_gain):(sigma>0&&finite(h)?h*h/(sigma*sigma):null);
        lines.push([
          name,t,r.last_seen_ms,r.node_id,r.boot_id,r.seq,truth,r.online,r.measurement_valid,r.dz_method,
          r.dz_mm,r.raw_dz_mm,r.uwb_uncertainty_mm,finite(r.uwb_residual_rms_m)?r.uwb_residual_rms_m*1000:null,
          (r.normal_sensitivity??r.vertical_sensitivity),r.uwb_velocity_mm_s,r.uwb_motion_score,r.uwb_zero_velocity_active,
          r.uwb_quiet_duration_ms,r.uwb_velocity_leak,r.uwb_verification_only,r.uwb_residual_baseline_frozen,r.uwb_displacement_consensus_confirmed,r.uwb_displacement_consensus_count,r.uwb_displacement_consensus_mm,r.uwb_displacement_consensus_spread_mm,r.uwb_residual_baseline_mm,r.uwb_quiet_residual_limit_mm,r.uwb_consensus_rejected,
          r.uwb_inlier_count,r.uwb_outlier_count,(r.uwb_inlier_anchors||[]).join('|'),(r.uwb_outlier_anchors||[]).join('|'),(r.uwb_quarantined_anchors||[]).join('|'),r.uwb_reacquired,r.uwb_reacquire_target_mm,
          r.uwb_window_samples,r.uwb_window_span_ms,r.uwb_effective_window_ms,
          r.uwb_process_sigma_d_mm,r.uwb_process_sigma_v_mm_s,r.tilt_deg,r.vibration,r.accel_g,r.ax_g,r.ay_g,r.az_g,r.imu_ok,r.wifi_rssi,
          r.range_count,r.requested_links,r.anchor_mask,r.adaptive_reason,r.adaptive_event,
          a.anchor_id,a.scheduled,a.ok,a.clock_offset_ppm,a.m,a.corrected_m,a.instant_corrected_m,a.bias_m,a.noise_m,sigma,
          (a.residual_m??detail.residual_m),h,info,(a.robust_weight??detail.robust_weight),(a.age_weight??detail.age_weight),
          (a.repetition_weight??detail.repetition_weight),(a.total_weight??detail.total_weight),(a.hard_rejected??detail.hard_rejected),(a.quarantined??detail.quarantined),(a.quarantine_until_ms??detail.quarantine_until_ms),(a.candidate_step_m??detail.candidate_step_m),(a.innovation_m??detail.innovation_m),a.outlier,a.calibrated,a.success_rate,
          a.attempt_success_rate,a.attempts,a.successes,a.nlos_score,a.nlos_suspect,a.fp_amp,a.peak_amp,a.fp_peak_gap,a.fp_ratio,a.diag_fresh,
          r.reference_only,r.quality_warning
        ].map(quote).join(','));
      }
    }
    const byTag=new Map();
    for(const {raw:r} of rows){if(!r.measurement_valid||!finite(r.dz_mm))continue;const a=byTag.get(r.node_id)||[];a.push(Number(r.dz_mm));byTag.set(r.node_id,a);}
    const stats=[];
    for(const [tag,values] of [...byTag.entries()].sort()){
      const mean=values.reduce((a,b)=>a+b,0)/values.length;
      const sigma=Math.sqrt(values.reduce((a,b)=>a+(b-mean)**2,0)/values.length);
      const rmse=truth==null?null:Math.sqrt(values.reduce((a,b)=>a+(b-truth)**2,0)/values.length);
      stats.push(`${tag}: n=${values.length}, 평균 ${mean.toFixed(1)} mm, σ ${sigma.toFixed(1)} mm, 범위 ${(Math.max(...values)-Math.min(...values)).toFixed(1)} mm${rmse==null?'':`, RMSE ${rmse.toFixed(1)} mm`}`);
    }
    $('uwbRecordStats').textContent=stats.length?stats.join(' · '):'유효 변위 없음 · CSV의 계측 상태와 원시 거리를 확인하세요.';
    const url=URL.createObjectURL(new Blob(['\ufeff'+lines.join('\r\n')],{type:'text/csv;charset=utf-8'}));
    const a=document.createElement('a');a.href=url;a.download=`SOC_UWB_ALL_TAGS_${new Date().toISOString().replaceAll(':','-')}.csv`;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
    $('uwbRecordStatus').textContent=`${rows.length}개 Tag packet 기록 종료 · 전체 Tag CSV 다운로드`;
  };
  $('uwbEditLayout').onclick=async()=>{
    try{
      const response=await fetch('/api/sensors/layout');if(!response.ok)throw Error('좌표 조회 실패');
      layout=await response.json();
      const points=[...layout.tags,...layout.anchors];
      if(!points.length)throw Error('먼저 추천 배치와 모듈 매핑을 완료하세요.');
      const table=document.createElement('table');
      table.innerHTML='<thead><tr><th>ID</th><th>X (m)</th><th>Y (m)</th><th>Z (m)</th></tr></thead>';
      const body=document.createElement('tbody');
      points.forEach((p,i)=>{const row=document.createElement('tr'),id=document.createElement('td');id.textContent=p.node_id;row.append(id);
        ['x','y','z'].forEach(k=>{const td=document.createElement('td'),input=document.createElement('input');input.type='number';input.step='0.0001';input.required=true;input.min='-1000';input.max='1000';input.value=Number(p[k]).toFixed(4);input.dataset.index=i;input.dataset.axis=k;input.setAttribute('aria-label',`${p.node_id} ${k}`);td.append(input);row.append(td);});body.append(row);});
      table.append(body);$('uwbLayoutRows').replaceChildren(table);$('uwbLayoutError').textContent='';$('uwbMetricConfirm').checked=false;$('uwbLayoutDialog').showModal();
    }catch(e){$('uwbRecordStatus').textContent=e.message;}
  };
  $('uwbLayoutCancel').onclick=()=>$('uwbLayoutDialog').close();
  $('uwbLayoutForm').onsubmit=async e=>{
    e.preventDefault();
    try{
      const next=structuredClone(layout),points=[...next.tags,...next.anchors];
      $('uwbLayoutRows').querySelectorAll('input').forEach(input=>points[Number(input.dataset.index)][input.dataset.axis]=Number(input.value));next.metric=true;
      const response=await fetch('/api/sensors/layout',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(next)});
      const data=await response.json();if(!response.ok)throw Error(data.message||'좌표 적용 실패');
      // Explicit confirmation is also a fresh static baseline even when coordinates are unchanged.
      const zero=await fetch('/api/sensors/zero',{method:'POST'});if(!zero.ok)throw Error('보정 재시작 실패');
      $('uwbLayoutDialog').close();$('uwbRecordStatus').textContent='좌표 적용 완료 · 구조물을 고정하고 보정 완료를 기다리세요.';
    }catch(e){$('uwbLayoutError').textContent=e.message;}
  };
})();
