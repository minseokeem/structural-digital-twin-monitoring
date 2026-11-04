const fs=require('fs'),vm=require('vm'),assert=require('assert/strict'),path=require('path');
const file=path.join(__dirname,'..','server.js');
const src=fs.readFileSync(file,'utf8');
const a=src.indexOf('function nearest('),b=src.indexOf('function extractJson',a);
const c={console,Math,Number,Object,Array,String,Boolean,JSON,Set};
vm.createContext(c);vm.runInContext(src.slice(a,b),c);
const tags=[
  {id:'T001',u:.24,x:-1,y:1,z:0,surfaceClass:'upper',surfacePatch:'left',structuralRole:'primary_superstructure',bridgePriorityScore:3.4,sourceMeshName:'Left blue span',accessibility_m:1},
  {id:'T002',u:.50,x:0,y:1,z:0,surfaceClass:'upper',surfacePatch:'center',structuralRole:'primary_superstructure',bridgePriorityScore:3.5,sourceMeshName:'Center blue span',accessibility_m:1},
  {id:'T003',u:.76,x:1,y:1,z:0,surfaceClass:'upper',surfacePatch:'right',structuralRole:'primary_superstructure',bridgePriorityScore:3.4,sourceMeshName:'Right blue span',accessibility_m:1},
  {id:'T004',u:.04,x:-1.3,y:.5,z:0,surfaceClass:'upper',surfacePatch:'pierL',structuralRole:'support_reference',bridgePriorityScore:-3,sourceMeshName:'Left pedestal',accessibility_m:2},
  {id:'T005',u:.96,x:1.3,y:.5,z:0,surfaceClass:'upper',surfacePatch:'pierR',structuralRole:'support_reference',bridgePriorityScore:-3,sourceMeshName:'Right pedestal',accessibility_m:2}
];
const anchors=[
  {id:'A001',x:-2,y:0,z:-2,quadrant:'N'},
  {id:'A002',x:2,y:2,z:-2,quadrant:'E'},
  {id:'A003',x:2,y:0,z:2,quadrant:'S'},
  {id:'A004',x:-2,y:2,z:2,quadrant:'W'}
];
const r=c.demoResult({profile:{geometryHypothesis:{category:'bridge',type:'elongated_bridge_or_viaduct'}},candidates:{tags,anchors},tagCount:3,anchorCount:4});
assert.deepEqual([...r.final_placement.tag_candidate_ids].sort(),['T001','T002','T003']);
console.log('PASS: bridge Tag strategy prioritizes left/center/right superstructure and rejects pedestal/support Tags.');
console.log(r.final_placement.tag_candidate_ids);
