(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.MemoVizStyle = api;
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  function decodeColor(value) {
    const alpha = (value >>> 24) & 255;
    const red = (value >>> 16) & 255;
    const green = (value >>> 8) & 255;
    const blue = value & 255;
    return alpha ? `rgba(${red},${green},${blue},${alpha / 255})` : `rgb(${red},${green},${blue})`;
  }
  function stroke(color) {
    const compact = color.toLowerCase().replace(/\s+/g, '');
    if (/^rgba?\(0,0,255(?:,[\d.]+)?\)$/.test(compact) || compact === '#0000ff')
      return { color: 'rgba(0,0,255,0.6)', width: 0.6 };
    const highlighted = /^rgba?\((?:0,255,255|255,0,0)(?:,[\d.]+)?\)$/.test(compact);
    return { color, width: highlighted ? 1.5 : 1 };
  }
  return { decodeColor, stroke };
});
