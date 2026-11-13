// Loads the phone engine straight out of lib/WebUI/EngineJs.h (the only copy).
const fs = require('fs'), path = require('path'), vm = require('vm');
module.exports = function loadEngine() {
  const h = fs.readFileSync(path.join(__dirname, '../../lib/WebUI/EngineJs.h'), 'utf8');
  const m = h.match(/R"ENGINEJS\(([\s\S]*?)\)ENGINEJS"/);
  if (!m) throw new Error('ENGINE_JS raw string not found');
  const sandbox = { window: {} };
  vm.runInNewContext(m[1], sandbox, { filename: 'engine.js' });
  return sandbox.window.SkyEngine;
};
