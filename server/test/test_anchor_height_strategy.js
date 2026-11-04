const fs=require('fs'),vm=require('vm'),assert=require('assert/strict'),path=require('path');
const file=path.join(__dirname,'..','server.js');
const src=fs.readFileSync(file,'utf8');
const a=src.indexOf('function nearest('),b=src.indexOf('function extractJson',a);
const c={console,Math,Number,Object,Array,String,Boolean,JSON,Set};
vm.createContext(c);vm.runInContext(src.slice(a,b),c);
const tags=[{id:'T001',x:0,y:1,z:0,u:.5,zone:'midspan'}];
const anchors=[];let id=1;
for(const y of [.05,.4,.8,1.2,1.6,2.0]){
  for(const [x,z,q] of [[-2,-1,'N'],[0,-1.5,'N'],[2,-1,'E'],[2,1,'E'],[0,1.5,'S'],[-2,1,'W']]){
    anchors.push({id:`A${String(id++).padStart(3,'0')}`,x,y,z,quadrant:q,heightRatio:y});
  }
}
const r=c.demoResult({profile:{geometryHypothesis:{type:'girder_bridge'}},candidates:{tags,anchors},tagCount:1,anchorCount:4,stage:'girder'});
assert.equal(r.final_placement.anchor_candidate_ids.length,4);
const selected=r.final_placement.anchor_candidate_ids.map(x=>anchors.find(a=>a.id===x));
assert(selected.some(a=>a.y<1),'expected at least one Anchor below Tag');
assert(selected.some(a=>a.y>1),'expected at least one Anchor above Tag');
assert(new Set(selected.map(a=>a.id)).size===4,'duplicate Anchor selected');
console.log('PASS: elevated Anchor strategy mixes heights and returns unique valid IDs.');
console.log(JSON.stringify(selected,null,2));
