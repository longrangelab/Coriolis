// Headless browser test of the phone page against a mock base station.
// Serves the page, /engine.js, /gear.js and /sky.js straight out of the C++ headers,
// fakes /data.json and implements /gear the way WebUI.h does (version check,
// 409 on conflict, gzip passthrough). Needs Playwright + Chromium.
// Run: node test_page.js
const http = require('http'), fs = require('fs'), path = require('path'), zlib = require('zlib');
const { chromium } = require('playwright');
const { extract } = require('./load_page_js.js');

const PAGE = extract('WebUI.h', 'HTMLPAGE');
const ENGINE = extract('EngineJs.h', 'ENGINEJS');
const GEAR = extract('GearJs.h', 'GEARJS');
const SKY = extract('SkyUiJs.h', 'SKYJS');

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
  if (u.pathname === '/sky.js') return send(200, 'application/javascript', SKY);
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
  await A.evaluate(() => { openClassicSolver(); setv('range', 600); });
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
  await A.evaluate(() => openClassicSolver());
  await A.setInputFiles('#importFile', { name: 'skyward_master.json', mimeType: 'application/json', buffer: Buffer.from(skyFile) });
  await A.waitForFunction(() => Gear.loads().length === 3);
  check('A: Skyward load imported', await A.evaluate(() => Gear.loads().some(l => l.name === 'ELD-M 140')));
  const opts = await A.evaluate(() => Array.from(document.querySelectorAll('#bulletSel option')).map(o => o.textContent));
  check('A: bullet picker lists the 6.5 bullets', opts.some(t => /ELD-M/.test(t)) && opts.some(t => /Hybrid/.test(t)), opts);
  const sky = await A.evaluate(() => { const id = Gear.loads().find(l => l.name === 'ELD-M 140').id; loadProfile(id); return { name: activeName(), bands: getv('bcBands', ''), s: Gear.solve(solverParams()) }; });
  check('A: multi-BC load shows its bands', /0\.326@2800/.test(sky.bands), sky.bands);
  check('A: multi-BC load solves', sky.s.ok, sky.s);
  await A.evaluate(() => Gear.sync());
  // typing a drag curve switches the model to Custom and the solver uses it
  const cur = await A.evaluate(() => {
    renderSolver();
    const ta = document.querySelector('#loadFields textarea');
    ta.value = '0.5:0.120, 0.9:0.125, 1.0:0.190, 1.2:0.180, 2.0:0.150, 3.0:0.130';
    ta.dispatchEvent(new Event('change'));
    const s = Gear.solve(solverParams());
    return { model: getv('model', 7), ok: s.ok, e: s.elevMil, bad: (() => { const t = document.querySelector('#loadFields textarea'); t.value = '0.5'; t.dispatchEvent(new Event('change')); return getv('dragCurve', '').length > 5; })() };
  });
  check('A: typed drag curve sets Custom and solves', cur.model === 'C' && cur.ok && cur.e > 0, cur);
  check('A: bad curve text is refused and the old curve kept', cur.bad, cur);
  await A.evaluate(() => { saveActive(); });
  check('A: saved curve is on the bullet', await A.evaluate(() => (Gear.bullet(Gear.load(activeLoadId()).bulletId).dragCurve || []).length === 6));
  const stored = () => JSON.parse((gear.gz ? zlib.gunzipSync(gear.body) : gear.body).toString());
  for (let i = 0; i < 40 && !(gear.body.length && stored().bullets.some(b => (b.dragCurve || []).length === 6)); i++) {
    await A.evaluate(() => Gear.sync()); await new Promise(r => setTimeout(r, 100));
  }
  check('A: import pushed (stored ' + (gear.gz ? 'gzipped' : 'plain') + ')', stored().loads.length === 3 && stored().bullets.some(b => (b.dragCurve || []).length === 6));

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

  // ---- Skyward-style solver (sky.js) on the Solver tab ----
  await A.evaluate(() => {
    const id = Gear.loads().find(l => l.name === '6.5 CM').id; loadProfile(id); setv('range', 600);
    window.__statusLog = [];
    new MutationObserver(() => window.__statusLog.push(document.getElementById('status').textContent))
      .observe(document.getElementById('status'), { childList: true, characterData: true, subtree: true });
    localStorage.setItem('skyTab', 'calc'); setView('avg'); setView('solver');
  });
  check('Sky: Solver tab opens the Skyward UI', await A.evaluate(() => SkyUI.isOpen() && !document.getElementById('sky').classList.contains('hide') && view !== 'solver'));
  await new Promise(r => setTimeout(r, 1300));   // launch splash
  await A.waitForFunction(() => /\d/.test(document.querySelector('#sky .sk-cell .v').textContent));
  const cmp = await A.evaluate(() => {
    const sol = SkyUI._solveNow(), r = sol.at(+getv('range', 0)), g = Gear.solve(solverParams());
    return { sky: r.comeUpMil * SkyEngine.MOA_PER_MIL, gear: g.elevMOA, skyW: r.windageMil * SkyEngine.MOA_PER_MIL, gearW: g.windMOA,
      mil: r.comeUpMil, shown: document.querySelector('#sky .sk-cell .v').textContent };
  });
  check('Sky: elevation matches the Wind Lab solver', Math.abs(cmp.sky - cmp.gear) < 0.01, cmp);
  check('Sky: windage matches the Wind Lab solver', Math.abs(cmp.skyW - cmp.gearW) < 0.01, cmp);
  check('Sky: result bar shows the elevation', Math.abs(parseFloat(cmp.shown.replace(/^[^0-9.-]+/, '')) - cmp.mil) < 0.051, cmp);
  // the live wind follows the meters
  const lw = await A.evaluate(() => ({ live: liveOn('live_wind'), speed: document.querySelectorAll('#sky .sk-steprow input')[1].value, avg: avgWind().speed }));
  check('Sky: live wind speed comes from the meters', lw.live && Math.abs(parseFloat(lw.speed) - lw.avg) < 0.06, lw);
  // phone compass: a heading from the device sets the direction of fire
  const az0 = await A.evaluate(() => fireAz);
  await A.click('#sky .sk-pill:has-text("Use phone compass")');
  await A.waitForSelector('#sky button:has-text("Waiting for the compass")');
  await A.evaluate(() => { for (let i = 0; i < 3; i++) window.dispatchEvent(Object.assign(new Event('deviceorientationabsolute'), { alpha: 90, absolute: true })); });
  await A.click('#sky button:has-text("Use 270°")');
  await new Promise(r => setTimeout(r, 150));
  const cz = await A.evaluate(() => ({ az: fireAz, stored: localStorage.getItem('fireAz'), field: document.getElementById('az').value }));
  check('Sky: phone compass sets the direction of fire', cz.az === 270 && cz.stored === '270' && cz.field === '270', cz);
  await A.click('#sky .sk-pill:has-text("Use phone compass")');
  await A.waitForSelector('#sky button:has-text("Compass not available")', { timeout: 4000 });
  check('Sky: no compass readings says so', true);
  await A.evaluate(() => document.querySelector('#sky .sk-sheet').parentNode.click());
  check('Sky: compass sheet closes', await A.evaluate(() => !document.querySelector('#sky .sk-sheet')));
  await A.evaluate(v => { fireAz = v; localStorage.setItem('fireAz', v); document.getElementById('az').value = v; }, az0);

  // full solution: WIND HOLD is the same number as Wind Lab's
  await A.click('#sky .sk-res .sk-btn');
  await A.waitForSelector('#sky .sk-big');
  await A.evaluate(() => updateWindHold(true));
  const holds = await A.evaluate(() => {
    const row = Array.from(document.querySelectorAll('#sky .sk-row')).find(r => /WIND HOLD/.test(r.textContent));
    return { sky: row && row.querySelector('.sk-val').textContent, lab: document.getElementById('windHold').textContent };
  });
  const moa = t => { const m = /([\d.]+) ([RL]) MOA/.exec(t || ''); return m ? m[1] + m[2] : null; };
  check('Sky: WIND HOLD matches Wind Lab', moa(holds.sky) && moa(holds.sky) === moa(holds.lab), holds);
  const tabClick = async label => { await A.click(`#sky .sk-nav button:has-text("${label}")`); await new Promise(r => setTimeout(r, 80)); };
  await tabClick('Table');
  check('Sky: trajectory table', await A.evaluate(() => document.querySelectorAll('#sky table.sk-tbl tr').length) > 10);
  await tabClick('Target Card');
  check('Sky: target card screen', await A.evaluate(() => /card/i.test(document.querySelector('#sky .sk-main').textContent)));
  await tabClick('Calibrate');
  check('Sky: calibrate screen', await A.evaluate(() => /muzzle velocity/i.test(document.querySelector('#sky .sk-main').textContent)));
  await A.goBack(); await new Promise(r => setTimeout(r, 150));
  check('Sky: phone back returns to the calculator', await A.evaluate(() => SkyUI.isOpen() && SkyUI.screen() === 'calc'));

  // My gear: add a firearm and ammo through the screens
  await tabClick('My gear');
  check('Sky: a card per firearm', await A.evaluate(() => document.querySelectorAll('#sky .sk-card').length === Gear.rifles().length));
  await A.click('#sky .sk-bwrap .sk-btn');
  await A.fill('#sky input[placeholder="Firearm name"]', 'Test Rifle');
  await A.click('#sky .sk-bar button[aria-label="Save"]');
  await new Promise(r => setTimeout(r, 150));
  check('Sky: firearm saved', await A.evaluate(() => Gear.rifles().some(r => r.name === 'Test Rifle' && r.twistIn === 8)));
  await A.click('#sky .sk-act[aria-label="Add ammo for Test Rifle"]');
  await A.fill('#sky input[placeholder="Ammo Name"]', 'Test Ammo');
  await A.fill('#sky input[placeholder="BC"]', '0.3');
  await A.click('#sky .sk-bar button[aria-label="Save"]');
  await new Promise(r => setTimeout(r, 150));
  const ta = await A.evaluate(() => { const l = Gear.loads().find(x => x.name === 'Test Ammo'); const b = l && Gear.bullet(l.bulletId); return { l, b }; });
  check('Sky: ammo saved with its own bullet', ta.l && ta.b && ta.b.bcPoints.length === 1 && ta.b.bcPoints[0].bc === 0.3 && ta.l.muzzleVelocityFps === 2800, ta);
  await A.evaluate(() => {
    const row = Array.from(document.querySelectorAll('#sky .sk-item')).find(r => r.querySelector('.t').textContent === 'Test Ammo');
    row.querySelector('.mo').click();
  });
  await A.click('#sky .sk-menu div:has-text("Use on the Calculator")');
  await new Promise(r => setTimeout(r, 200));
  const used = await A.evaluate(() => ({ name: activeName(), mv: +getv('mv', 0), bc: +getv('bc', 0), tab: SkyUI.screen(), log: window.__statusLog.slice() }));
  check('Sky: "Use on the Calculator" sets the Wind Lab profile', /Test Ammo/.test(used.name) && used.mv === 2800 && used.bc === 0.3 && used.tab === 'calc', used);
  check('Sky: own edits are not called "updated on another device"', !used.log.some(t => /another device/.test(t)), used.log);
  await A.waitForFunction(() => /\d/.test(document.querySelector('#sky .sk-cell .v').textContent));

  // Analysis: hit probability
  await tabClick('Analysis');
  await A.click('#sky .sk-row:has-text("Hit probability")');
  await A.waitForFunction(() => Array.from(document.querySelectorAll('#sky div')).some(d => /^\d+%$/.test(d.textContent) && d.style.fontSize === '44px'), null, { timeout: 8000 });
  const hp = await A.evaluate(() => Array.from(document.querySelectorAll('#sky div')).find(d => /^\d+%$/.test(d.textContent) && d.style.fontSize === '44px').textContent);
  check('Sky: hit probability works', parseInt(hp) >= 0 && parseInt(hp) <= 100, hp);
  await A.goBack(); await new Promise(r => setTimeout(r, 150));

  // More -> classic solver, and the toolbar back arrow
  await tabClick('More');
  await A.click('#sky .sk-row:has-text("Classic Coriolis solver")');
  await new Promise(r => setTimeout(r, 150));
  check('Sky: More opens the classic solver', await A.evaluate(() => !SkyUI.isOpen() && view === 'solver' && document.getElementById('solveView').className !== 'hide'));
  await A.evaluate(() => { setView('avg'); localStorage.setItem('skyTab', 'calc'); setView('solver'); });
  await A.click('#sky .sk-bar button[aria-label="Back to Wind Lab"]');
  await new Promise(r => setTimeout(r, 150));
  check('Sky: back arrow returns to Wind Lab', await A.evaluate(() => !SkyUI.isOpen() && document.getElementById('sky').classList.contains('hide')));
  if (process.env.SHOT_DIR) {
    const shot = async (name) => { await new Promise(r => setTimeout(r, 250)); await A.screenshot({ path: path.join(process.env.SHOT_DIR, name) }); };
    await A.evaluate(() => setView('solver')); await shot('sky_calc.png');
    await A.click('#sky .sk-res .sk-btn'); await shot('sky_solution.png');
    await A.goBack(); await tabClick('My gear'); await shot('sky_gear.png');
    await A.evaluate(() => SkyUI.close());
  }

  check('no request fell back to the ESP32 solver', legacyCalls.length === 0, legacyCalls);
  check('no JS errors or dialogs', errors.length === 0, errors);
  if (process.env.SHOT_DIR) {
    await A.evaluate(() => { openClassicSolver(); doSolve(); });
    await new Promise(r => setTimeout(r, 300));
    await A.screenshot({ path: path.join(process.env.SHOT_DIR, 'page_solver.png'), fullPage: true });
  }
  await browser.close(); server.close();
  console.log(fails ? fails + ' FAILED' : 'ALL PASS');
  process.exit(fails ? 1 : 0);
})().catch(e => { console.error(e); process.exit(1); });
