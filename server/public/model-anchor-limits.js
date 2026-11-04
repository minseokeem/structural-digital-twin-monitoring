// Shared browser/server rule. Coordinates are metres in the viewer's world frame.
(function(root){
  const normalize=name=>String(name||'').split(/[\\/]/).pop().trim().toLowerCase()
    .replace(/\s*\(2\)(?=\.glb$)/,'');
  function forFiles(names){
    if(!Array.isArray(names)||names.length!==1)return null;
    const name=normalize(names[0]);
    if(name==='assembly_model_raised_35mm.glb')
      return {minX:-0.70,maxX:0.70,minZ:-0.50,maxZ:0.50,maxY:0.60};
    if(name==='assembly_model.glb')
      return {minX:-0.70,maxX:0.70,minZ:-0.45,maxZ:0.45};
    return null;
  }
  function contains(point,limits){
    if(!limits)return true;
    if(!Number.isFinite(point?.x)||!Number.isFinite(point?.z))return false;
    if(point.x<limits.minX||point.x>limits.maxX||point.z<limits.minZ||point.z>limits.maxZ)return false;
    if(Number.isFinite(limits.minY)&&(!Number.isFinite(point?.y)||point.y<limits.minY))return false;
    if(Number.isFinite(limits.maxY)&&(!Number.isFinite(point?.y)||point.y>limits.maxY))return false;
    return true;
  }
  const api={forFiles,contains};
  if(typeof module!=='undefined'&&module.exports)module.exports=api;
  else root.ModelAnchorLimits=api;
})(typeof globalThis!=='undefined'?globalThis:this);
