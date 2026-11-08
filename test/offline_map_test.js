// Run with node src/webviz/test/offline_map_test.js; no browser dependencies.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const root = path.resolve(__dirname, '..');
const bounds = [-98.05, 30.05, -97.45, 30.60];

function setup({statusCode = 200, missingLibrary = false} = {}) {
  const callbacks = {};
  const requests = [];
  const status = {textContent: '', title: ''};
  const map = {maxZoom: 0, setMaxZoom(value) { this.maxZoom = value; }};
  const renderer = {on(name, callback) { callbacks[name] = callback; }};
  const layer = {addTo() {}, getMaplibreMap() { return renderer; }};
  const context = vm.createContext({
    URL, AbortSignal, console: {error() {}},
    L: {maplibreGL: missingLibrary ? undefined : options => {
      const urls = options.style.sources.osm.tiles.concat(options.style.glyphs);
      assert(urls.every(url => url.startsWith('http://robot:10273/tiles/')));
      assert.deepEqual(Array.from(options.style.sources.osm.bounds), bounds);
      return layer;
    }},
    fetch: async url => {
      requests.push(url);
      assert.equal(url, 'http://robot:10273/tiles/austin/metadata.json');
      return {ok: statusCode === 200, status: statusCode,
        headers: {get() { return 'test-build'; }},
        json: async () => ({format: 'pbf', vector_layers: [], bounds, maxzoom: 14})};
    },
  });
  vm.runInContext(fs.readFileSync(path.join(root, 'map_style.js'), 'utf8'), context);
  vm.runInContext(fs.readFileSync(path.join(root, 'offline_map.js'), 'utf8'), context);
  return {context, status, map, callbacks, layer, requests};
}

(async () => {
  const ok = setup();
  assert.equal(await ok.context.loadOfflineGpsMap(ok.map, ok.status, 'http://robot:10273'), ok.layer);
  ok.callbacks.load();
  assert.equal(ok.status.textContent, 'Offline map');
  assert.equal(ok.map.maxZoom, 22);
  ok.callbacks.error({error: new Error('missing label glyph')});
  assert.match(ok.status.textContent, /missing label glyph/);
  assert.equal(ok.requests.length, 1);  // errors do not fetch an online replacement

  for (const options of [{statusCode: 404}, {missingLibrary: true}]) {
    const failed = setup(options);
    assert.equal(await failed.context.loadOfflineGpsMap(
      failed.map, failed.status, 'http://robot:10273'), null);
    assert.match(failed.status.textContent, /Offline map unavailable/);
    assert(failed.requests.length <= 1);
  }
  const file = setup();
  assert.equal(await file.context.loadOfflineGpsMap(file.map, file.status, 'null'), null);
  assert.equal(file.requests.length, 0);

  const html = fs.readFileSync(path.join(root, 'webviz.html'), 'utf8');
  assert(!html.includes('tile.openstreetmap.org'));
  assert(html.includes('offline_map.js'));
  const oldMetadata = vm.runInContext(
    'webvizMapStyle("http://robot:10273", "austin", "1", {bounds:[0,0,0,0]})', ok.context);
  assert.equal(oldMetadata.sources.osm.bounds, undefined);
  console.log('Offline map success, error, missing-data, coverage, and no-public-fallback checks passed');
})().catch(error => { console.error(error); process.exitCode = 1; });
