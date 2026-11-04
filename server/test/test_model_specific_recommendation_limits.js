'use strict';
const assert=require('assert');
const limitsApi=require('../public/model-anchor-limits');

const raised=limitsApi.forFiles(['assembly_model_raised_35mm.glb']);
assert.deepStrictEqual(raised,{minX:-0.70,maxX:0.70,minZ:-0.50,maxZ:0.50,maxY:0.60});
assert(limitsApi.contains({x:-0.70,y:0.60,z:-0.50},raised));
assert(limitsApi.contains({x:0.70,y:0.60,z:0.50},raised));
assert(!limitsApi.contains({x:0.701,y:0.20,z:0},raised));
assert(!limitsApi.contains({x:0,y:0.20,z:0.501},raised));
assert(!limitsApi.contains({x:0,y:0.601,z:0},raised));

const legacy=limitsApi.forFiles(['assembly_model.glb']);
assert.deepStrictEqual(legacy,{minX:-0.70,maxX:0.70,minZ:-0.45,maxZ:0.45});
assert(limitsApi.contains({x:0,y:9,z:0.45},legacy));
assert(!limitsApi.contains({x:0,y:0,z:0.451},legacy));
assert.strictEqual(limitsApi.forFiles(['other.glb']),null);
console.log('model-specific recommendation limits: ok');
