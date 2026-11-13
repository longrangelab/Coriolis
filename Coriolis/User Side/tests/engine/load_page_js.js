// Loads the phone-side scripts straight out of their C++ headers.
const fs = require('fs'), path = require('path'), vm = require('vm');
function extract(file, tag) {
  const h = fs.readFileSync(path.join(__dirname, '../../lib/WebUI', file), 'utf8');
  const m = h.match(new RegExp('R"' + tag + '\\(([\\s\\S]*?)\\)' + tag + '"'));
  if (!m) throw new Error(tag + ' raw string not found in ' + file);
  return m[1];
}
// A window-like sandbox with an in-memory localStorage.
function makeWindow(store) {
  store = store || {};
  const localStorage = {
    getItem: k => (k in store ? store[k] : null), setItem: (k, v) => { store[k] = String(v); },
    removeItem: k => { delete store[k]; },
  };
  const w = { localStorage, crypto: require('crypto').webcrypto, console, setTimeout, clearTimeout, Math, JSON, Date };
  w.window = w;
  return { w, store };
}
module.exports = function load(store, extraGlobals) {
  const { w } = makeWindow(store);
  w.fetch = () => Promise.reject(new Error('no base station in this test'));
  Object.assign(w, extraGlobals || {});
  const ctx = vm.createContext(w);
  vm.runInContext(extract('EngineJs.h', 'ENGINEJS'), ctx, { filename: 'engine.js' });
  vm.runInContext(extract('GearJs.h', 'GEARJS'), ctx, { filename: 'gear.js' });
  return w;
};
module.exports.extract = extract;
