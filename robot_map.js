// Policy geolocation is [latitude, longitude, ENU yaw in radians]. Route XY is
// metres in the footprint frame: x forward, y left, matching training.
function localRouteLatLng(geolocation, points) {
  if (!Array.isArray(geolocation) || geolocation.length < 3 ||
      !geolocation.every(Number.isFinite)) return [];
  const [lat, lng, yaw] = geolocation;
  if (Math.abs(lat) >= 90 || Math.abs(lng) > 180) return [];
  const degreesPerMetre = 180 / (Math.PI * 6378137);
  return (points || []).filter(p => Array.isArray(p) && p.length >= 2 &&
    Number.isFinite(p[0]) && Number.isFinite(p[1])).map(([x, y]) => {
    const east = Math.cos(yaw) * x - Math.sin(yaw) * y;
    const north = Math.sin(yaw) * x + Math.cos(yaw) * y;
    return [lat + north * degreesPerMetre,
      lng + east * degreesPerMetre / Math.cos(lat * Math.PI / 180)];
  });
}

function tabShortcut(event) {
  if (event.defaultPrevented || event.ctrlKey || event.metaKey || event.altKey ||
      event.isComposing || event.target?.isContentEditable ||
      event.target?.closest?.('input, textarea, select, [contenteditable]')) return null;
  return ({s: 'health', m: 'map', r: 'robot'})[event.key.toLowerCase()] || null;
}

class RobotRouteMap {
  constructor(id, status) {
    this.status = status;
    this.map = L.map(id, {minZoom: 15, maxZoom: 22}).setView([30.2849, -97.7341], 18);
    const satelliteOptions = {maxZoom: 22, attribution: 'Satellite imagery &copy; Esri'};
    const satellite = L.layerGroup([
      L.tileLayer('/satellite/17/{x}/{y}.jpg', {...satelliteOptions, minNativeZoom: 17, maxNativeZoom: 17}),
      L.tileLayer('/satellite/19/{x}/{y}.jpg', {...satelliteOptions, minZoom: 18,
        minNativeZoom: 19, maxNativeZoom: 19, bounds: [[30.235, -97.8], [30.35, -97.68]]}),
    ]).addTo(this.map);
    const streets = L.layerGroup();
    let streetLoading = false;
    this.map.on('baselayerchange', async event => {
      if (event.layer === streets && !streetLoading) {
        streetLoading = true;
        const layer = await loadOfflineGpsMap(this.map, status, window.location.origin);
        if (layer) {
          streets.addLayer(layer);
          if (!this.map.hasLayer(streets)) layer.remove();
        }
      }
    });
    satellite.eachLayer(layer => layer.on('tileerror', () => {
      status.textContent = 'Satellite tile unavailable · try Map layer';
    }));
    L.control.layers({'Satellite': satellite, 'Map': streets}).addTo(this.map);
    L.control.scale({imperial: false}).addTo(this.map);
    this.route = L.polyline([], {color: '#ff2020', weight: 4, opacity: 1}).addTo(this.map);
    this.local = L.layerGroup().addTo(this.map);
    this.follow = true;
    this.map.on('dragstart', () => { this.follow = false; });
    status.textContent = 'Waiting for robot location';
  }

  setRoute(points) { this.route.setLatLngs(points); }

  setPosition(latlng) {
    if (!latlng?.every(Number.isFinite)) return;
    if (!this.robot) {
      this.robot = L.circleMarker(latlng, {radius: 6, color: '#17212b', weight: 2,
        fillColor: '#4ba3e3', fillOpacity: 1}).addTo(this.map);
    } else this.robot.setLatLng(latlng);
    this.robot.bringToFront();
    if (this.follow) this.map.panTo(latlng, {animate: false});
    this.status.textContent = `${latlng[0].toFixed(6)}, ${latlng[1].toFixed(6)}`;
  }

  setFrame(frame) {
    this.local.clearLayers();
    const points = localRouteLatLng(frame.geolocation, frame.goal_xy);
    if (!points.length) {
      this.status.textContent = 'Policy geolocation unavailable';
      return;
    }
    points.forEach(point => L.circleMarker(point, {radius: 4, color: '#fff', weight: 2,
      fillColor: '#0d6efd', fillOpacity: 1}).addTo(this.local));
    this.setPosition(frame.geolocation.slice(0, 2));
  }

  recenter() {
    this.follow = true;
    if (this.robot) this.map.panTo(this.robot.getLatLng());
  }
}

if (typeof module !== 'undefined') module.exports = {localRouteLatLng, tabShortcut};
