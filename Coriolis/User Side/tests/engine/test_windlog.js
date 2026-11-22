// Headless browser test of the Basic/Forecast wind call and the wind log,
// against a mock base station that serves /windsec the way main.cpp does:
// readings on the base's millis() clock, only those a packet has delivered
// (each packet repeats the last 3 seconds), filtered by ?since=.
// Needs Playwright + Chromium. Run: node test_windlog.js
const http = require('http'), fs = require('fs');
const { chromium } = require('playwright');
const { extract } = require('./load_page_js.js');

const PAGE = extract('WebUI.h', 'HTMLPAGE');
const ENGINE = extract('EngineJs.h', 'ENGINEJS');
const GEAR = extract('GearJs.h', 'GEARJS');
const SKY = extract('SkyUiJs.h', 'SKYJS');

let fails = 0;
function check(name, cond, extra) { if (!cond) { fails++; console.log('FAIL', name, extra === undefined ? '' : JSON.stringify(extra).slice(0, 600)); } else console.log('ok  ', name); }
const sleep = ms => new Promise(r => setTimeout(r, ms));

// ---- mock base: three meters, 1 s readings at their own sub-second phase ----
const base = { bootAt: Date.now() - 200000, aim: false, utc: true, windsecCalls: 0 };
const METERS = [{ id: 1, phase: 130, pkt: 0 }, { id: 2, phase: 470, pkt: 900 }, { id: 3, phase: 820, pkt: 1800 }];
function baseNow() { return Date.now() - base.bootAt; }
function speedOf(id, t) { return 6 + id + 2 * Math.sin(t / 7000 + id); }
function dirOf(id, t) { return (260 + 10 * id + 5 * Math.sin(t / 5000)) % 360; }
// A meter's packets go out every 3 s (at its pkt phase) carrying its last 3
// readings, so a reading becomes visible only once a packet has carried it.
function visibleUpTo(m, now) { const lastPkt = Math.floor((now - m.pkt) / 3000) * 3000 + m.pkt; return lastPkt; }
function windsec(since) {
  const now = baseNow(), nodes = {};
  METERS.forEach(m => {
    if (m.dead) return;
    const upto = visibleUpTo(m, now), r = [];
    // ring of the last 300 readings (WSEC_CAP)
    const lastK = Math.floor((upto - m.phase) / 1000), firstK = Math.max(0, lastK - 299);
    for (let k = firstK; k <= lastK; k++) {
      const t = k * 1000 + m.phase;
      if (since !== null && t - since <= 0) continue;
      r.push([t, +speedOf(m.id, t).toFixed(2), +dirOf(m.id, t).toFixed(1)]);
    }
    nodes[m.id] = { src: 'beacon', period: 1000, r };
  });
  return { now, utc: base.utc ? Date.now() : null, avg_ms: 2000, nodes };
}
function node(id, speed, dir) {
  return { id, speed, speedSd: 0.5, dir, dirSd: 5, gust: speed + 2, tempF: 50, presInHg: 29.5, lux: -1, battV: 4, battPct: 80,
    lat: 0, lon: 0, gps: false, distYd: -1, rssi: -80, age: 1, windOk: true, windStale: false, sleeping: false, sleepCmd: false };
}
function dataJson() {
  const t = baseNow();
  return JSON.stringify({
    base: { lat: 0, lon: 0, tempF: 50, presInHg: 29.5, battPct: 90, battV: 4.1, gps: false },
    avg: { count: 3, seen: 3, speed: 8, speedSd: 0.5, dir: 270, agree: 1, dirSd: 5, tempF: 50, presInHg: 29.5 },
    nodes: METERS.map(m => node(m.id, speedOf(m.id, t), dirOf(m.id, t))),
    stability: { cur: { spdSd: 0.5, dirSd: 5 }, w10: { spdSd: 0.5, dirSd: 5 }, w30: { spdSd: 0.5, dirSd: 5 }, w60: { spdSd: 0.5, dirSd: 5 } },
    sleep: { mode: false, waking: false, wakeRemain: 0, acked: 0, nodeMask: 0 },
    aim: { present: base.aim, moving: false, clamped: false },
  });
}
const T = Date.now() - 1000;
const lib = JSON.stringify({
  format: 'skyward-library', version: 1, kind: 'master', app: 'Skyward Ballistics',
  rifles: [{ id: 'r1', name: 'Tikka CTR', twistIn: 8, twistDirection: 'RIGHT', sightHeightIn: 1.9, sightOffsetIn: 0,
    windageUnit: 'MOA', windageClick: 0.25, elevationUnit: 'MOA', elevationClick: 0.25, updatedAt: T }],
  loads: [{ id: 'l1', rifleId: 'r1', rifleName: 'Tikka CTR', name: 'ELD-M 140', bulletId: 'b1', muzzleVelocityFps: 2705, zeroRangeYd: 100, updatedAt: T }],
  bullets: [{ id: 'b1', maker: 'Hornady', name: '6.5mm 140 ELD-M', diameterIn: 0.264, weightGr: 140, lengthIn: 1.42, dragModel: 'G7',
    bcPoints: [{ bc: 0.315, minVelFps: 0 }], dragCurve: [], source: 'user', updatedAt: T }],
  deleted: [], cards: [], shots: [],
});
const gear = { ver: 1, body: Buffer.from(lib) };
const server = http.createServer((req, res) => {
  const u = new URL(req.url, 'http://x');
  const send = (code, type, body, hdr) => { res.writeHead(code, Object.assign({ 'Content-Type': type }, hdr || {})); res.end(body); };
  if (u.pathname === '/') return send(200, 'text/html', PAGE);
  if (u.pathname === '/engine.js') return send(200, 'application/javascript', ENGINE);
  if (u.pathname === '/gear.js') return send(200, 'application/javascript', GEAR);
  if (u.pathname === '/sky.js') return send(200, 'application/javascript', SKY);
  if (u.pathname === '/data.json') return send(200, 'application/json', dataJson());
  if (u.pathname === '/windsec') { base.windsecCalls++; const s = u.searchParams.get('since'); return send(200, 'application/json', JSON.stringify(windsec(s === null ? null : +s))); }
  if (u.pathname === '/gear/info') return send(200, 'application/json', JSON.stringify({ ok: true, ver: gear.ver, bytes: gear.body.length, gz: false, max: 393216 }));
  if (u.pathname === '/gear' && req.method === 'GET') return send(200, 'application/json', gear.body, { 'X-Gear-Ver': String(gear.ver) });
  if (u.pathname === '/gear' && req.method === 'POST') { req.resume(); return req.on('end', () => send(409, 'application/json', JSON.stringify({ ok: false, err: 'conflict', ver: gear.ver }))); }
  if (u.pathname === '/favicon.ico') return send(204, 'image/x-icon', '');   // the real base answers every other path with the page
  if (u.pathname === '/cmd' || u.pathname === '/aim') return send(200, 'application/json', '{"ok":true}');
  send(404, 'text/plain', 'nf');
});

(async () => {
  await new Promise(r => server.listen(0, '127.0.0.1', r));
  const url = 'http://127.0.0.1:' + server.address().port + '/';
  const browser = await chromium.launch();
  const errors = [];
  const ctx = await browser.newContext({ viewport: { width: 400, height: 860 }, acceptDownloads: true });
  await ctx.addInitScript(() => { if (!localStorage.getItem('__seeded')) { localStorage.setItem('activeLoadId', 'l1'); localStorage.setItem('sol_range', '600');
    localStorage.setItem('section', 'basic'); localStorage.setItem('__seeded', '1'); } });
  async function tab() {
    const pg = await ctx.newPage();
    pg.on('pageerror', e => errors.push('pageerror: ' + e.message));
    pg.on('console', m => { if (m.type() === 'error') errors.push('console: ' + m.text()); });
    pg.on('dialog', d => d.accept());
    await pg.goto(url);
    await pg.waitForFunction(() => window.Gear && window.SkyEngine && Gear.load('l1') && window.data);
    return pg;
  }
  const A = await tab();
  await A.evaluate(() => { followActiveLoad(); setSection('basic'); });

  // ---- Basic (AVG): no SD cards, big hold in the rifle's unit, distance ----
  await A.waitForFunction(() => /MOA/.test(document.querySelector('#windHold .callBig') ? document.querySelector('#windHold .callBig').textContent : ''), null, { timeout: 5000 });
  const basic = await A.evaluate(() => ({
    sd: getComputedStyle(document.getElementById('sdRow')).display, call: getComputedStyle(document.getElementById('callCard')).display,
    big: document.querySelector('#windHold .callBig').textContent, sub: document.querySelector('#windHold .callSub').textContent,
    dist: document.getElementById('distIn').value, unit: document.getElementById('distU').textContent,
    hero: getComputedStyle(document.querySelector('.hero')).display, sleep: !!document.getElementById('sleepBar'),
    order: (() => { const a = document.querySelector('.hero'), b = document.getElementById('callCard'); return !!(a.compareDocumentPosition(b) & Node.DOCUMENT_POSITION_FOLLOWING); })(),
  }));
  check('Basic: Speed SD / Dir SD / Crosswind cards are gone', basic.sd === 'none', basic);
  check('Basic: weighted wind window still there', basic.hero !== 'none', basic);
  check('Basic: wind hold window below it', basic.call !== 'none' && basic.order, basic);
  check('Basic: hold in the rifle\'s unit (MOA) with clicks', /^[RL]\s?[\d.]+MOA$/.test(basic.big.replace(/ /g, ' ')) && /clicks/.test(basic.sub), basic);
  check('Basic: distance shows the Solver\'s distance', basic.dist === '600' && basic.unit === 'yd', basic);
  // the number follows the solver exactly
  const same = await A.evaluate(() => { const s = Gear.solve(solverParams()); return { s: s.windMOA, txt: document.querySelector('#windHold .callSub').textContent }; });
  check('Basic: hold equals the solver\'s windage', new RegExp(Math.abs(same.s).toFixed(1).replace('.', '\\.') + ' [RL] MOA').test(same.txt), same);
  // + / - and typing change the Solver's distance; the hold follows
  await A.click('#callCard .distBtn[aria-label="Farther"]');
  const d1 = await A.evaluate(() => ({ r: getv('range', 0), v: document.getElementById('distIn').value }));
  check('Basic: + steps the distance and the Solver gets it', +d1.r === 625 && d1.v === '625', d1);
  await A.fill('#distIn', '800'); await A.press('#distIn', 'Enter'); await A.evaluate(() => document.getElementById('distIn').blur());
  const d2 = await A.evaluate(() => ({ r: getv('range', 0), sky: SkyUI._params().range }));
  check('Basic: typed distance goes to the Solver', +d2.r === 800 && +d2.sky === 800, d2);
  // the Solver changes the distance: Basic follows
  await A.evaluate(() => { setv('range', 450); render(); });
  check('Basic: follows a distance set in the Solver', await A.evaluate(() => document.getElementById('distIn').value) === '450');
  // metric distance setting in the Solver
  const met = await A.evaluate(() => { localStorage.setItem('skySettings', JSON.stringify({ distanceUnit: 'm' })); renderCallCard(true);
    const r = { v: document.getElementById('distIn').value, u: document.getElementById('distU').textContent };
    localStorage.removeItem('skySettings'); renderCallCard(true); return r; });
  check('Basic: metres when the Solver is set to metres', met.v === '411' && met.u === 'm', met);
  // MIL rifle
  const mil = await A.evaluate(() => { const r = Gear.rifle('r1'); const save = r.windageUnit; r.windageUnit = 'MIL'; r.windageClick = 0.1;
    const f = holdFmt(0.42); r.windageUnit = save; r.windageClick = 0.25; return f; });
  check('Basic: MIL rifle shows MIL', mil.unit === 'MIL' && /0\.4/.test(mil.text), mil);
  base.aim = true; await sleep(800);
  check('Basic: aimbot buttons stay off the Basic tab (even with a stepper)', await A.evaluate(() => getComputedStyle(document.getElementById('aimRow')).display === 'none'));
  base.aim = false;

  // ---- Forecast ----
  base.aim = true;
  await A.evaluate(() => setSection('forecast'));
  await sleep(800);
  const fc = await A.evaluate(() => {
    const vis = id => getComputedStyle(document.getElementById(id)).display !== 'none';
    const pos = (a, b) => !!(a.compareDocumentPosition(b) & Node.DOCUMENT_POSITION_FOLLOWING);
    return { hero: getComputedStyle(document.querySelector('.hero')).display !== 'none', call: vis('callCard'), sd: vis('sdRow'),
      aim: vis('aimRow'), bottom: vis('predBottom'), logInStab: !!document.querySelector('#stabView #btnStart'),
      logUnderWheel: pos(document.getElementById('wheelBlock'), document.getElementById('btnStart')),
      swLast: pos(document.getElementById('btnStart'), document.getElementById('aimSw')), mark: !!document.getElementById('btnMark'),
      big: (document.querySelector('#windHold .callBig') || {}).textContent };
  });
  check('Forecast: live wind window + wind hold window', fc.hero && fc.call && /MOA/.test(fc.big || ''), fc);
  check('Forecast: no SD cards on the field view', !fc.sd, fc);
  check('Forecast: log is its own window under the wind wheel', fc.bottom && !fc.logInStab && fc.logUnderWheel, fc);
  check('Forecast: aimbot switch at the bottom', fc.swLast, fc);
  check('Forecast: no MARK (shot) button', !fc.mark, fc);
  check('Forecast: aimbot shows with a stepper and the switch on', fc.aim, fc);
  await A.click('#aimSw', { force: true });
  check('Forecast: switch off hides AIMBOT / ZERO', await A.evaluate(() => getComputedStyle(document.getElementById('aimRow')).display === 'none' && localStorage.getItem('aimOn') === '0'));
  await A.click('#aimSw', { force: true });
  check('Forecast: switch on shows them again', await A.evaluate(() => getComputedStyle(document.getElementById('aimRow')).display === 'flex'));
  base.aim = false;

  // ---- wind log ----
  await A.evaluate(() => setv('range', 600));
  await A.click('#btnStart');
  await A.waitForFunction(() => wlMeta() && wlMeta().rec);
  // a second tab of the same page must not record the same data again
  const B = await tab();
  await sleep(9000);
  const st1 = await B.evaluate(() => ({ other: wlOtherTabRecording(), html: document.getElementById('logStat') ? wlStatusHtml() : '' }));
  check('Log: a second open tab leaves the recording to the first', st1.other, st1);
  // base restarts mid-log (its clock starts again)
  base.bootAt = Date.now() - 500;
  await sleep(7000);
  // a meter drops out, then the page reloads mid-log (it carries on)
  METERS[2].dead = true;
  await A.reload(); await A.waitForFunction(() => window.Gear && window.data);
  await sleep(7000);
  METERS[2].dead = false;
  await sleep(5000);
  await A.evaluate(() => setSection('log'));
  await A.click('#wlBtnStop');
  const meta = await A.evaluate(() => wlMeta());
  check('Log: stopped', !meta.rec && meta.stoppedAt > meta.startedAt, meta);
  check('Log: saw the base restart', meta.boots === 1, meta);
  check('Log: all three meters logged', meta.n['1'] > 10 && meta.n['2'] > 10 && meta.n['3'] > 5, meta.n);
  check('Log: weighted rows logged', meta.n.w > 15, meta.n);

  // export
  const [dl] = await Promise.all([A.waitForEvent('download'), A.click('button:has-text("Download CSV")')]);
  const csv = fs.readFileSync(await dl.path(), 'utf8').trim().split(/\r?\n/);
  const head = csv[0].split(','), rows = csv.slice(1).map(l => l.split(','));
  check('CSV: header', head.join(',') === 'time_utc,unix_ms,meter,speed_mph,dir_deg,crosswind_mph,hold_mil,hold_moa,distance_yd,fire_az_deg,meters_used', head);
  check('CSV: file name', /^windlog_\d{4}-\d\d-\d\d_\d{4}\.csv$/.test(dl.suggestedFilename()), dl.suggestedFilename());
  const keys = new Set(), byMeter = {};
  let dup = 0, unsorted = 0, prev = 0, badVal = 0;
  rows.forEach(r => {
    const k = r[2] + '|' + r[1]; if (keys.has(k)) dup++; keys.add(k);
    if (+r[1] < prev) unsorted++; prev = +r[1];
    (byMeter[r[2]] = byMeter[r[2]] || []).push(r);
    if (r[2] !== 'weighted') { const id = +r[2]; if (!(Math.abs(+r[3] - speedOf(id, 0) ) < 10)) badVal++; }
  });
  check('CSV: no duplicate rows', dup === 0, dup);
  check('CSV: oldest first', unsorted === 0, unsorted);
  check('CSV: meter rows match what the base stored', +meta.n['1'] === byMeter['1'].length && +meta.n['2'] === byMeter['2'].length && +meta.n['3'] === byMeter['3'].length,
    { meta: meta.n, csv: Object.keys(byMeter).map(k => k + ':' + byMeter[k].length) });
  // each meter: one row per second (its readings 1 s apart), apart from the restart and the dropout
  const gaps = m => { const t = byMeter[m].map(r => +r[1]); let g = 0; for (let i = 1; i < t.length; i++) if (t[i] - t[i - 1] !== 1000) g++; return g; };
  check('CSV: meter 1 is second by second (one break: base restart)', gaps('1') <= 1, gaps('1'));
  check('CSV: meter 3 has the dropout and restart breaks only', gaps('3') <= 2, gaps('3'));
  // backfill: meter rows exist for seconds whose packet arrived later (every
  // second is there, not only the packet times)
  check('CSV: readings between packets were kept (backfill)', byMeter['2'].length >= 0.8 * (meta.stoppedAt - meta.startedAt) / 1000 - 12, byMeter['2'].length);
  const w = byMeter.weighted;
  const wsec = new Set(w.map(r => +r[1]));
  check('CSV: weighted rows on whole seconds, once each', w.every(r => +r[1] % 1000 === 0) && wsec.size === w.length, w.slice(0, 3));
  check('CSV: weighted rows carry the hold, distance and meters used', w.every(r => r[6] !== '' && r[7] !== '' && r[8] === '600' && +r[10] >= 1), w.slice(0, 2));
  check('CSV: hold MOA = mil x 3.4377', w.every(r => Math.abs(+r[7] - +r[6] * 3.43774677) < 0.011), w[0]);
  const w2 = w.find(r => +r[10] === 2);
  check('CSV: weighted wind uses the meters that reported (2 during the dropout)', !!w2, w.map(r => r[10]).join(''));
  check('CSV: meter rows leave the weighted columns empty', byMeter['1'].every(r => r[5] === '' && r[6] === ''));
  check('CSV: meter values are the base\'s readings', badVal === 0, badVal);
  // the weighted hold equals the solver's hold for that wind
  const chk = await A.evaluate(r => { const p = solverParams(); p.windmph = +r[3]; p.windrel = relAngle(+r[4]); return Gear.solve(p).windMil; }, w[w.length - 1]);
  check('CSV: weighted hold matches the solver for that second\'s wind', Math.abs(chk - +w[w.length - 1][6]) < 0.002, { chk, row: w[w.length - 1] });

  // Log tab shows meters and charts
  const lv = await A.evaluate(() => ({ rows: document.querySelectorAll('#wlTable tbody tr').length, svgs: document.querySelectorAll('#wlCharts svg').length,
    shots: /shot|miss/i.test(document.getElementById('logView').textContent) }));
  check('Log tab: a row per meter + weighted, three charts', lv.rows === 4 && lv.svgs === 3, lv);
  check('Log tab: nothing about shots or misses', !lv.shots, lv);
  // old-format log offered for download, not silently deleted
  await A.evaluate(() => { localStorage.setItem('windlog', JSON.stringify({ samples: [{ t: 1 }], marks: [] })); renderLog(); });
  check('Log tab: an older log is offered, not deleted', await A.evaluate(() => /older log/i.test(document.getElementById('wlOld').textContent)));
  // gear export is no longer blocked by an empty log
  await A.evaluate(() => { localStorage.removeItem('windlog'); });

  check('Log tab: fleet sleep bar stays off', await A.evaluate(() => getComputedStyle(document.getElementById('sleepBar')).display === 'none'));
  // delete
  await A.click('button:has-text("Delete log")');
  check('Log: delete clears it', await A.evaluate(() => wlMeta() === null && /Nothing logged/.test(document.getElementById('wlTable').textContent)));

  // ---- regressions on the classic views ----
  await A.evaluate(() => setSection('basic'));
  await A.evaluate(() => setView(2));
  const nodeTab = await A.evaluate(() => ({ sd: getComputedStyle(document.getElementById('sdRow')).display, call: document.getElementById('callCard').className,
    fleet: getComputedStyle(document.getElementById('sleepBar')).display }));
  check('Node tab: fleet sleep bar stays off (it is for Basic/Forecast)', nodeTab.fleet === 'none', nodeTab);
  check('Node tab: keeps its SD cards, no wind call card', nodeTab.sd !== 'none' && /hide/.test(nodeTab.call), nodeTab);
  await A.evaluate(() => setView('avg'));

  check('no JS errors', errors.length === 0, errors);
  await browser.close(); server.close();
  console.log(fails ? fails + ' FAILED' : 'ALL PASS');
  process.exit(fails ? 1 : 0);
})().catch(e => { console.error(e); process.exit(1); });
