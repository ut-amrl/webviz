// Offline MapLibre style for the Map tab: OpenMapTiles-schema vector tiles
// built by memo_ws/scripts/build_map_tiles.sh and served (with the glyphs) by
// memo_ws/scripts/serve_webviz.py under <base>/tiles/. Footways and other
// paths are drawn prominently since the robot navigates as a pedestrian.
function webvizMapStyle(base, tileset, version, metadata = {}) {
  const query = version ? `?v=${encodeURIComponent(version)}` : '';
  const name = ['coalesce', ['get', 'name:latin'], ['get', 'name']];
  const road = (id, classes, color, widths, extra) => Object.assign({
    id: id,
    type: 'line',
    source: 'osm',
    'source-layer': 'transportation',
    filter: ['in', ['get', 'class'], ['literal', classes]],
    layout: { 'line-cap': 'round', 'line-join': 'round' },
    paint: {
      'line-color': color,
      'line-width': ['interpolate', ['exponential', 1.6], ['zoom'], 12, widths[0], 16, widths[1], 20, widths[2]],
    },
  }, extra || {});

  return {
    version: 8,
    glyphs: `${base}/tiles/fonts/{fontstack}/{range}.pbf${query}`,
    sources: {
      osm: {
        type: 'vector',
        tiles: [`${base}/tiles/${tileset}/{z}/{x}/{y}.pbf${query}`],
        minzoom: metadata.minzoom ?? 0,
        maxzoom: metadata.maxzoom ?? 14,
        ...(Array.isArray(metadata.bounds) && metadata.bounds.length === 4 &&
            metadata.bounds.every(Number.isFinite) && metadata.bounds[0] < metadata.bounds[2] &&
            metadata.bounds[1] < metadata.bounds[3] ? { bounds: metadata.bounds } : {}),
        attribution: '&copy; OpenStreetMap contributors',
      },
    },
    layers: [
      { id: 'background', type: 'background', paint: { 'background-color': '#f2efe9' } },
      {
        id: 'landuse', type: 'fill', source: 'osm', 'source-layer': 'landuse',
        paint: {
          'fill-color': ['match', ['get', 'class'],
            ['residential', 'suburb', 'neighbourhood'], '#ebe7e0',
            ['commercial', 'retail'], '#f2e4e2',
            ['industrial', 'railway'], '#e8e2ea',
            ['school', 'university', 'college', 'hospital'], '#f0ebd8',
            ['cemetery'], '#d8e6d2',
            '#ece8e1'],
        },
      },
      {
        id: 'landcover', type: 'fill', source: 'osm', 'source-layer': 'landcover',
        paint: {
          'fill-color': ['match', ['get', 'class'], 'wood', '#cfe3c0', 'sand', '#f3e9c6', '#d9ebc9'],
          'fill-opacity': 0.8,
        },
      },
      { id: 'park', type: 'fill', source: 'osm', 'source-layer': 'park', paint: { 'fill-color': '#cfe8bd', 'fill-opacity': 0.8 } },
      { id: 'water', type: 'fill', source: 'osm', 'source-layer': 'water', paint: { 'fill-color': '#aad3df' } },
      {
        id: 'waterway', type: 'line', source: 'osm', 'source-layer': 'waterway',
        paint: { 'line-color': '#aad3df', 'line-width': ['interpolate', ['linear'], ['zoom'], 10, 0.5, 16, 3] },
      },
      {
        id: 'building', type: 'fill', source: 'osm', 'source-layer': 'building', minzoom: 14,
        paint: { 'fill-color': '#d9d0c9', 'fill-outline-color': '#c4b8ad' },
      },
      road('road-service', ['service', 'track'], '#ffffff', [0.5, 3, 10], { minzoom: 14 }),
      road('road-minor', ['minor'], '#ffffff', [0.8, 5, 18]),
      road('road-tertiary', ['tertiary'], '#ffffff', [1, 7, 22]),
      road('road-secondary', ['secondary'], '#f7f3c8', [1.2, 8, 26]),
      road('road-primary', ['primary', 'trunk'], '#fcd6a4', [1.5, 9, 30]),
      road('road-motorway', ['motorway'], '#e892a2', [1.8, 10, 34]),
      road('rail', ['rail', 'transit'], '#9a9a9a', [0.6, 1.5, 3], { paint: { 'line-color': '#9a9a9a', 'line-width': 1.5, 'line-dasharray': [3, 2] } }),
      // Pedestrian network: sidewalks, footways, paths, steps (tiles carry
      // these from z14; MapLibre over-zooms them to street level).
      road('path', ['path'], '#e8604c', [0.6, 1.6, 4], {
        minzoom: 14,
        paint: {
          'line-color': '#e8604c',
          'line-width': ['interpolate', ['linear'], ['zoom'], 14, 0.8, 18, 2.5, 21, 4],
          'line-dasharray': [2, 1],
        },
      }),
      {
        id: 'road-names', type: 'symbol', source: 'osm', 'source-layer': 'transportation_name', minzoom: 14,
        layout: {
          'symbol-placement': 'line',
          'text-field': name,
          'text-font': ['Noto Sans Regular'],
          'text-size': ['interpolate', ['linear'], ['zoom'], 14, 10, 19, 14],
        },
        paint: { 'text-color': '#444', 'text-halo-color': '#fff', 'text-halo-width': 1.5 },
      },
      {
        id: 'poi', type: 'symbol', source: 'osm', 'source-layer': 'poi', minzoom: 16,
        filter: ['<=', ['get', 'rank'], 30],
        layout: { 'text-field': name, 'text-font': ['Noto Sans Regular'], 'text-size': 11, 'text-max-width': 8 },
        paint: { 'text-color': '#6b4f3a', 'text-halo-color': '#fff', 'text-halo-width': 1.2 },
      },
      {
        id: 'place', type: 'symbol', source: 'osm', 'source-layer': 'place',
        layout: {
          'text-field': name,
          'text-font': ['Noto Sans Regular'],
          'text-size': ['match', ['get', 'class'], 'city', 16, 'town', 14, 12],
        },
        paint: { 'text-color': '#333', 'text-halo-color': '#fff', 'text-halo-width': 1.5 },
      },
    ],
  };
}
