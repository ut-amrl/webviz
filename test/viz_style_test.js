const assert = require('node:assert/strict');
const { decodeColor, stroke } = require('../viz_style.js');
assert.equal(decodeColor(0x990000ff), 'rgba(0,0,255,0.6)');
assert.equal(decodeColor(0xff00ffff), 'rgba(0,255,255,1)');
assert.equal(decodeColor(0x00ff0000), 'rgb(255,0,0)');
for (const color of ['rgb(0,0,255)', 'rgba(0,0,255,0.6)', '#0000ff']) {
  assert.deepEqual(stroke(color), { color: 'rgba(0,0,255,0.6)', width: 0.6 });
}
assert.deepEqual(stroke(decodeColor(0x00ffff)), { color: 'rgb(0,255,255)', width: 1.5 });
assert.deepEqual(stroke(decodeColor(0xff0000)), { color: 'rgb(255,0,0)', width: 1.5 });
assert.equal(stroke('rgb(200,200,200)').width, 1);
console.log('Rollout width/opacity, unsigned alpha, and distinct prediction/tracking colors passed');
