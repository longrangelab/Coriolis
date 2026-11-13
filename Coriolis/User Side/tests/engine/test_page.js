// Headless browser test of the phone page against a mock base station.
// Serves the page, /engine.js and /gear.js straight out of the C++ headers,
// fakes /data.json and implements /gear the way WebUI.h does (version check,
// 409 on conflict, gzip passthrough). Needs Playwright + Chromium.
// Run: node test_page.js
const http = require('http'), fs = require('fs'), path = require('path'), zlib = require('zlib');
const { chromium } = require('playwright');
const { extract } = require('./load_page_js.js');

const PAGE = extract('WebUI.h', 'HTMLPAGE');
const ENGINE = extract('EngineJs.h', 'ENGINEJS');
const GEAR = extract('GearJs.h', 'GEARJS');

let fails = 0;
function check(name, cond, extra) { if (!cond) { fails++; console.log('FAIL', name, extra === undefined ? '' : extra); } else console.log('ok  ', name); }

// ---- mock base station ----
const gear = { ver: 0, gz: false, body: Buffer.alloc(0), posts: 0, conflicts: 0 };
const base = { lat: 45.0, lon: -93.0 };
// Firing azimuth 0 (north): nodes placed 100 / 400 / 700 yd north of the base.
const YD_PER_DEG_LAT = 111319.4908 * 1.0936133;
function node(id, downYd, speed, dir) {
  return { id, speed, speedSd: 0.5, dir, dirSd: 5, gust: speed + 2, tempF: 50, presInHg: 29.5, lux: -1, battV: 4, battPct: 80,
    lat: base.lat + downYd / YD_PER_DEG_LAT, lon: base.lon, gps: true, distYd: downYd, rssi: -80, age: 1,
    windOk: true, windStale: false, sleeping: false, sleepCmd: false };
}
function dataJson() {
  return JSON.stringify({
    base: { lat: base.lat, lon: base.lon, tempF: 50, presInHg: 29.5, battPct: 90, battV: 4.1, gps: true },
    avg: { count: 3, seen: 3, speed: 8, speedSd: 0.5, dir: 270, agree: 1, dirSd: 5, tempF: 50, presInHg: 29.5 },
    nodes: [node(1, 100, 8, 270), node(2, 400, 8, 270), node(3, 700, 8, 270)],
    stability: { cur: { spdSd: 0.5, dirSd: 5 }, w10: { spdSd: 0.5, dirSd: 5 }, w30: { spdSd: 0.5, dirSd: 5 }, w60: { spdSd: 0.5, dirSd: 5 } },
    sleep: { mode: false, waking: false, wakeRemain: 0, acked: 0, nodeMask: 0 },
    aim: { present: false, moving: false, clamped: false },
  });
}
function multipartFile(buf, ctype) {
  const b = '--' + /boundary=([^;]+)/.exec(ctype)[1];
  const start = buf.indexOf('\r\n\r\n') + 4;
  const end = buf.indexOf(Buffer.from('\r\n' + b), start);
  return buf.subarray(start, end);
}
const legacyCalls = [];
const server = http.createServer((req, res) => {
  const u = new URL(req.url, 'http://x');
  const send = (code, type, body, hdr) => { res.writeHead(code, Object.assign({ 'Content-Type': type }, hdr || {})); res.end(body); };
  if (u.pathname === '/') return send(200, 'text/html', PAGE);
  if (u.pathname === '/engine.js') return send(200, 'application/javascript', ENGINE);
  if (u.pathname === '/gear.js') return send(200, 'application/javascript', GEAR);
  if (u.pathname === '/data.json') return send(200, 'application/json', dataJson());
  if (u.pathname === '/gear/info') return send(200, 'application/json', JSON.stringify({ ok: true, ver: gear.ver, bytes: gear.body.length, gz: gear.gz, max: 393216 }));
  if (u.pathname === '/gear' && req.method === 'GET') {
    if (!gear.body.length) return send(204, 'application/json', '');
    return send(200, 'application/json', gear.body, Object.assign({ 'X-Gear-Ver': String(gear.ver) }, gear.gz ? { 'Content-Encoding': 'gzip' } : {}));
  }
  if (u.pathname === '/gear' && req.method === 'POST') {
    const chunks = []; req.on('data', c => chunks.push(c));
    req.on('end', () => {
      if (+u.searchParams.get('base') !== gear.ver) { gear.conflicts++; return send(409, 'application/json', JSON.stringify({ ok: false, err: 'conflict', ver: gear.ver })); }
      gear.body = Buffer.from(multipartFile(Buffer.concat(chunks), req.headers['content-type']));
      gear.gz = u.searchParams.get('gz') === '1'; gear.ver++; gear.posts++;
      send(200, 'application/json', JSON.stringify({ ok: true, ver: gear.ver }));
    });
    return;
  }
  if (u.pathname === '/solve' || u.pathname === '/sensitivity') { legacyCalls.push(u.pathname); return send(200, 'application/json', '{"ok":false}'); }
  if (u.pathname === '/cmd' || u.pathname === '/aim') return send(200, 'application/json', '{"ok":true}');
  send(404, 'text/plain', 'nf');
});

const oldProfiles = {
  '6.5 CM': { mv: '2710', bc: '0.315', model: '7', wgt: '140', cal: '0.264', twist: '8', blen: '1.41', twistDir: '1', sh: '1.75', zero: '100',
    zeroElevMag: '0', zeroElevDir: 'high', zeroWindMag: '0', zeroWindDir: 'right', range: '600' },
  '308 G1': { mv: 2650, bc: 0.505, model: 1, wgt: 175, cal: 0.308, twist: 10, blen: '', twistDir: -1, sh: 1.5, zero: 100 },
};
const T = Date.now() - 1000;
const skyFile = JSON.stringify({
  format: 'skyward-library', version: 1, kind: 'master', app: 'Skyward Ballistics',
  rifles: [{ id: 'r1', name: 'Tikka CTR', twistIn: 8, twistDirection: 'RIGHT', sightHeightIn: 1.9, sightOffsetIn: 0, updatedAt: T }],
  loads: [{ id: 'l1', rifleId: 'r1', rifleName: 'Tikka CTR', name: 'ELD-M 140', bulletId: 'b1', muzzleVelocityFps: 2705, zeroRangeYd: 100, updatedAt: T }],
  bullets: [
    { id: 'b1', maker: 'Hornady', name: '6.5mm 140 ELD-M', diameterIn: 0.264, weightGr: 140, lengthIn: 1.42, dragModel: 'G7',
      bcPoints: [{ bc: 0.326, minVelFps: 2800 }, { bc: 0.315, minVelFps: 2200 }, { bc: 0.305, minVelFps: 0 }], dragCurve: [], source: 'user', updatedAt: T },
    { id: 'b2', maker: 'Berger', name: '6.5mm 140 Hybrid', diameterIn: 0.264, weightGr: 140, lengthIn: 1.4, dragModel: 'G7',
      bcPoints: [{ bc: 0.311, minVelFps: 0 }], dragCurve: [], source: 'library', updatedAt: T },
  ],
  deleted: [], cards: [], shots: [],
});

(async () => {
  await new Promise(r => server.listen(0, '127.0.0.1', r));
  const url = 'http://127.0.0.1:' + server.address().port + '/';
  const browser = await chromium.launch();
  const errors = [];
  async function phone(seed) {
    const ctx = await browser.newContext({ viewport: { width: 400, height: 860 } });
    if (seed) await ctx.addInitScript(s => { if (!localStorage.getItem('__seeded')) { for (const k in s) localStorage.setItem(k, s[k]); localStorage.setItem('__seeded', '1'); } }, seed);
    const pg = await ctx.newPage();
    pg.on('pageerror', e => errors.push('pageerror: ' + e.message));
    pg.on('console', m => { if (m.type() === 'error') errors.push('console: ' + m.text()); });
    pg.on('dialog', d => { errors.push('dialog: ' + d.message()); d.dismiss(); });
    await pg.goto(url);
    await pg.waitForFunction(() => window.Gear && window.SkyEngine);
    return pg;
  }

  // ---- phone A: old profiles migrate and get pushed to the base station ----
  const A = await phone({ profiles: JSON.stringify(oldProfiles), activeProfile: '6.5 CM', opt_spin: '1', opt_jump: '1', live_wind: '1' });
  await A.waitForFunction(() => Gear.meta().status === 'synced with base station', null, { timeout: 8000 });
  check('A: migrated old profiles', await A.evaluate(() => Gear.loads().length) === 2);
  check('A: active profile carried over', await A.evaluate(() => activeName()) === '6.5 CM');
  check('A: gear pushed to base station', gear.ver >= 1 && gear.body.length > 0, gear);

  // ---- solver ----
  await A.evaluate(() => { setView('solver'); setv('range', 600); });
  await A.evaluate(() => doSolve());
  const res = await A.textContent('#results');
  check('A: solve shows elevation and windage', /Elevation/.test(res) && /Windage/.test(res) && /MOA/.test(res), res);
  const s600 = await A.evaluate(() => Gear.solve(solverParams()));
  check('A: 600 yd elevation is sane (12-16 MOA)', s600.ok && s600.elevMOA > 12 && s600.elevMOA < 16, s600.elevMOA);
  await A.evaluate(() => { document.getElementById('tableBox').open = true; renderTable(); });
  const rows = await A.evaluate(() => document.querySelectorAll('#tableOut tr').length);
  check('A: come-up table has 6 rows + header', rows === 7, rows);

  // ---- WIND HOLD (Basic view, live wind) ----
  await A.evaluate(() => { setView('avg'); });
  await A.evaluate(() => updateWindHold(true));
  const wh = await A.textContent('#windHold');
  check('A: WIND HOLD shows a hold', /MOA/.test(wh) && /mil/.test(wh), wh);

  // ---- auto weights from ballistics (phone-side sensitivity) ----
  await A.evaluate(() => autoWeights());
  const w = await A.evaluate(() => ({ w: nodeWeights(), s: lastSens }));
  const sum = (w.w[1] || 0) + (w.w[2] || 0) + (w.w[3] || 0);
  check('A: auto weights sum to 100', Math.abs(sum - 100) < 0.5, w);
  check('A: muzzle meter weighs most', w.w[1] > w.w[2] && w.w[2] > w.w[3], w.w);
  check('A: positions from GPS', w.s[1] && w.s[1].src === 'gps' && Math.abs(w.s[2].downYd - 400) < 2, w.s);

  // ---- Skyward import + bullet picker ----
  await A.evaluate(() => setView('solver'));
  await A.setInputFiles('#importFile', { name: 'skyward_master.json', mimeType: 'application/json', buffer: Buffer.from(skyFile) });
  await A.waitForFunction(() => Gear.loads().length === 3);
  check('A: Skyward load imported', await A.evaluate(() => Gear.loads().some(l => l.name === 'ELD-M 140')));
  const opts = await A.evaluate(() => Array.from(document.querySelectorAll('#bulletSel option')).map(o => o.textContent));
  check('A: bullet picker lists the 6.5 bullets', opts.some(t => /ELD-M/.test(t)) && opts.some(t => /Hybrid/.test(t)), opts);
  const sky = await A.evaluate(() => { const id = Gear.loads().find(l => l.name === 'ELD-M 140').id; loadProfile(id); return { name: activeName(), bands: getv('bcBands', ''), s: Gear.solve(solverParams()) }; });
  check('A: multi-BC load shows its bands', /0\.326@2800/.test(sky.bands), sky.bands);
  check('A: multi-BC load solves', sky.s.ok, sky.s);
  await A.evaluate(() => Gear.sync());
  const stored = () => JSON.parse((gear.gz ? zlib.gunzipSync(gear.body) : gear.body).toString());
  check('A: import pushed (stored ' + (gear.gz ? 'gzipped' : 'plain') + ')', stored().loads.length === 3);

  // ---- sandbox true curve ----
  await A.evaluate(() => setSection('sandbox'));
  const sb = await A.evaluate(() => {
    const id = Gear.loads().find(l => l.name === '6.5 CM').id;
    const sel = document.getElementById('sbProfileSel'); sel.value = id; sbLoadProfile(id); sbPopulateCurve();
    return { src: sbCoefSrc, n: sbRealCoef && sbRealCoef.length, lbl: document.getElementById('sbCurveSrc').textContent };
  });
  await new Promise(r => setTimeout(r, 300));
  const sb2 = await A.evaluate(() => ({ src: sbCoefSrc, n: sbRealCoef && sbRealCoef.length, lbl: document.getElementById('sbCurveSrc').textContent }));
  check('A: sandbox gets the true curve from the phone engine', sb2.src === 'device' && sb2.n > 4 && /true curve/.test(sb2.lbl), sb2);
  await A.evaluate(() => setSection('basic'));

  // ---- phone B: empty phone pulls gear from the base station ----
  const B = await phone(null);
  await B.waitForFunction(() => Gear.loads().length === 3, null, { timeout: 8000 });
  check('B: pulled all 3 loads from base station', true);
  // B edits a load; A sees it after its next sync.
  await B.evaluate(() => { const id = Gear.loads().find(l => l.name === '308 G1').id; setActiveId(id); loadProfile(id); setv('mv', 2600); saveActive(); return Gear.sync(); });
  await new Promise(r => setTimeout(r, 1200));
  await B.evaluate(() => Gear.sync());
  await A.evaluate(() => Gear.sync());
  const mvA = await A.evaluate(() => Gear.flat(Gear.loads().find(l => l.name === '308 G1').id).mv);
  check('A: sees B\'s edit after sync', +mvA === 2600, mvA);

  check('no request fell back to the ESP32 solver', legacyCalls.length === 0, legacyCalls);
  check('no JS errors or dialogs', errors.length === 0, errors);
  if (process.env.SHOT_DIR) {
    await A.evaluate(() => { setView('solver'); doSolve(); });
    await new Promise(r => setTimeout(r, 300));
    await A.screenshot({ path: path.join(process.env.SHOT_DIR, 'page_solver.png'), fullPage: true });
  }
  await browser.close(); server.close();
  console.log(fails ? fails + ' FAILED' : 'ALL PASS');
  process.exit(fails ? 1 : 0);
})().catch(e => { console.error(e); process.exit(1); });
