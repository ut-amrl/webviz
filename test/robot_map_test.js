const assert = require('node:assert/strict');
const {localRouteLatLng, tabShortcut} = require('../robot_map.js');

const close = (actual, expected) => assert.ok(Math.abs(actual - expected) < 1e-9);
const lat = 30.28, lng = -97.73;
const degree = 180 / (Math.PI * 6378137);
const east = localRouteLatLng([lat, lng, 0], [[8, 0], [0, 0], [0, 8]]);
close(east[0][0], lat);
close(east[0][1], lng + 8 * degree / Math.cos(lat * Math.PI / 180));
assert.deepEqual(east[1], [lat, lng]);
close(east[2][0], lat + 8 * degree);
close(east[2][1], lng);
const north = localRouteLatLng([lat, lng, Math.PI / 2], [[8, 0], [0, 8]]);
close(north[0][0], lat + 8 * degree);
close(north[0][1], lng);
close(north[1][0], lat);
close(north[1][1], lng - 8 * degree / Math.cos(lat * Math.PI / 180));
for (const invalid of [null, [], [NaN, lng, 0], [90, lng, 0]]) {
  assert.deepEqual(localRouteLatLng(invalid, [[8, 0]]), []);
}
assert.deepEqual(localRouteLatLng([lat, lng, 0], [[NaN, 0], null, [0, 0]]), [[lat, lng]]);

for (const [key, tab] of [['s', 'health'], ['m', 'map'], ['r', 'robot'], ['R', 'robot']]) {
  assert.equal(tabShortcut({key}), tab);
  for (const modifier of ['ctrlKey', 'metaKey', 'altKey', 'isComposing', 'defaultPrevented']) {
    assert.equal(tabShortcut({key, [modifier]: true}), null);
  }
  assert.equal(tabShortcut({key, target: {isContentEditable: true}}), null);
  assert.equal(tabShortcut({key, target: {closest: () => ({})}}), null);
}
assert.equal(tabShortcut({key: 'Enter'}), null);
console.log('Geographic route projection, ENU heading, invalid fixes, and safe tab shortcuts passed');
