const fs=require('fs'),vm=require('vm'),assert=require('assert/strict'),path=require('path');
const file=path.join(__dirname,'..','server.js');
const src=fs.readFileSync(file,'utf8');
const a=src.indexOf('function nearest('),b=src.indexOf('function extractJson',a);
const c={console,Math,Number,Object,Array,String,Boolean,JSON,Set};
vm.createContext(c);vm.runInContext(src.slice(a,b),c);
const tags=[
  {id:'T001',x:-.5,y:1,z:0,nx:0,ny:1,nz:0,u:.2,structuralRole:'primary_superstructure',bridgePriorityScore:3},
  {id:'T002',x:0,y:1,z:0,nx:0,ny:1,nz:0,u:.5,structuralRole:'primary_superstructure',bridgePriorityScore:3},
  {id:'T003',x:.5,y:1,z:0,nx:0,ny:1,nz:0,u:.8,structuralRole:'primary_superstructure',bridgePriorityScore:3}
];
const anchors=[];let id=1;
const columns=[[-2,-1,'NW'],[2,-1,'NE'],[2,1,'SE'],[-2,1,'SW'],[0,-2,'N'],[0,2,'S']];
for(const y of [.05,.5,1,1.5,2]) for(const [x,z,q] of columns)
  anchors.push({id:`A${String(id++).padStart(3,'0')}`,x,y,z,quadrant:q,heightRatio:y});
const r=c.demoResult({profile:{geometryHypothesis:{type:'girder_bridge',category:'bridge'}},candidates:{tags,anchors},tagCount:3,anchorCount:4,stage:'bridge'});
const selected=r.final_placement.anchor_candidate_ids.map(x=>anchors.find(a=>a.id===x));
assert.equal(selected.length,4);
const keys=selected.map(a=>`${a.x.toFixed(4)}:${a.z.toFixed(4)}`);
assert.equal(new Set(keys).size,4,'same X/Z column selected more than once');
console.log('PASS: all selected Anchors use different X/Z positions.');
console.log(JSON.stringify(selected,null,2));
