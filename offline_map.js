// Load only the locally built basemap. A renderer or tile failure must never
// redirect the dashboard to public tile servers.
async function loadOfflineGpsMap(map, status, base) {
  let layer;
  let failed = false;
  const reportError = (reason) => {
    failed = true;
    const message = (reason && reason.message) || String(reason);
    console.error('Offline map unavailable:', message);
    status.textContent = `Offline map unavailable: ${message}`;
    status.title = 'Serve the dashboard with scripts/serve_webviz.py; build its data with scripts/build_map_tiles.sh. No public tile server is used.';
  };
  try {
    if (!/^https?:$/.test(new URL(base).protocol)) {
      throw new Error('open the dashboard through its HTTP server');
    }
    if (typeof L.maplibreGL !== 'function' || typeof webvizMapStyle !== 'function') {
      throw new Error('the local vector map renderer failed to load');
    }
    status.textContent = 'Loading offline map...';
    const response = await fetch(new URL('/tiles/austin/metadata.json', base).href, {
      cache: 'no-store', signal: AbortSignal.timeout(3000),
    });
    if (!response.ok) throw new Error(`tile metadata: HTTP ${response.status}`);
    const metadata = await response.json();
    if (metadata.format !== 'pbf' || !Array.isArray(metadata.vector_layers)) {
      throw new Error('invalid offline vector tile metadata');
    }
    const version = response.headers.get('Last-Modified') || '1';
    layer = L.maplibreGL({
      style: webvizMapStyle(base, 'austin', version, metadata), maxZoom: 22,
    });
    layer.addTo(map);
    const renderer = layer.getMaplibreMap();
    renderer.on('load', () => {
      if (!failed) {
        status.textContent = 'Offline map';
        status.title = 'Local Austin vector tiles; no public tile server is used.';
      }
    });
    // A missing glyph or an out-of-coverage tile must not discard the rest of
    // the map or start downloading an online replacement.
    renderer.on('error', (event) => reportError(event.error || event));
    map.setMaxZoom(22);
    return layer;
  } catch (error) {
    reportError(error);
    return null;
  }
}
