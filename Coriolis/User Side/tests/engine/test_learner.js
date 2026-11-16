// Tests the wind learner (/wind-learner.js, LearnJs.h) and the Learn page.
//   node test_learner.js          learner maths only (no browser)
//   node test_learner.js --page   also runs the page headless (needs playwright)
const assert = require('assert'), vm = require('vm'), http = require('http');
const { extract } = require('./load_page_js.js');

const LIB = extract('LearnJs.h', 'LEARNLIB');
const PAGE = extract('LearnJs.h', 'LEARNHTML');
const ctx = vm.createContext({ Math, JSON, Array, Object });
ctx.globalThis = ctx;
vm.runInContext(LIB, ctx, { filename: 'wind-learner.js' });
const WL = ctx.WindLearner;
let pass = 0;
function ok(name, fn) { fn(); pass++; console.log('ok  ' + name); }
function near(a, b, tol, msg) { assert(Math.abs(a - b) <= tol, (msg || '') + ' got ' + a + ' want ' + b + ' +/- ' + tol); }

// ---- geometry: line of fire due north, wind from the west (moves east, to the right)
const shooter = { x: 0, y: 0 }, target = { x: 0, y: 800 }, west = { x: 10, y: 0 };
const g = WL.geometry([
  { id: 'line', x: 5, y: 300 }, { id: 'up', x: -100, y: 500 }, { id: 'down', x: 120, y: 400 },
  { id: 'past', x: 0, y: 950 }, { id: 'behind', x: 0, y: -100 }
], shooter, target, west, {});
const row = id => g.rows.find(r => r.id === id);
ok('line of fire bearing and length', () => { near(g.lof.R, 800, 1e-9); near(g.lof.bearing, 0, 1e-9); near(g.cross, 10, 1e-9); });
ok('meter on the line is used', () => { assert(row('line').included); assert.strictEqual(row('line').side, 'on line'); });
ok('upwind meter is used with a lead time', () => {
  const r = row('up'); assert(r.included); assert.strictEqual(r.side, 'upwind');
  near(r.lead, 100 / (10 * WL.YD_PER_S_PER_MPH), 1e-6, 'lead');
});
ok('far downwind meter is ignored', () => { assert(!row('down').included); assert.strictEqual(row('down').side, 'downwind'); });
ok('meters past the target or behind the shooter are ignored', () => { assert(!row('past').included); assert(!row('behind').included); });
ok('wind from the east flips upwind and downwind', () => {
  const e = WL.geometry([{ id: 'up', x: -100, y: 500 }], shooter, target, { x: -10, y: 0 }, {});
  assert.strictEqual(e.rows[0].side, 'downwind');
});

// ---- physics weights: each meter owns the nearest stretch, muzzle counts more
const kernel = (a, b) => { const f = s => 2 * s / 800 - s * s / 640000; return f(b) - f(a); }; // 2:0 muzzle-to-target slope
ok('physics weights sum to 1 and favour the muzzle', () => {
  const w = WL.physicsWeights([{ id: 'a', s: 100, included: true }, { id: 'b', s: 400, included: true }, { id: 'c', s: 700, included: true }], 800, kernel);
  near(w.a + w.b + w.c, 1, 1e-12); assert(w.a > w.b && w.b > w.c);
});
ok('ignored meters get no weight', () => {
  const w = WL.physicsWeights([{ id: 'a', s: 100, included: true }, { id: 'x', s: 400, included: false }], 800, kernel);
  near(w.a, 1, 1e-12); assert(!('x' in w));
});

// ---- learner: recovers a meter that reads low, keeps the call in mph
function rng(seed) { return () => { seed = (seed * 1664525 + 1013904223) >>> 0; return seed / 4294967296; }; }
const r = rng(42), gauss = () => { const u = 1 - r(), v = r(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); };
ok('learns a sheltered meter (reads 0.6x) from 300 shots', () => {
  const L = new WL.Learner();
  for (let i = 0; i < 300; i++) {
    const a = 10 + 4 * gauss(), b = 10 + 4 * gauss();       // what each stretch of the path really had
    const readA = a + 0.3 * gauss(), readB = 0.6 * b + 0.3 * gauss();
    const sector = i % 2 ? 6 : 2;                            // wind from W and from E
    L.update([{ lkey: 'A#0', x: 0.5 * readA }, { lkey: 'B#0', x: 0.5 * readB }], sector, 0.5 * a + 0.5 * b + 0.7 * gauss(), 0.8);
  }
  near(L.meter('A#0', 6).mult, 1, 0.12, 'A');
  near(L.meter('B#0', 6).mult, 1 / 0.6, 0.2, 'B');
  assert(L.meter('B#0', 6).sd < 0.12, 'B should be confident by now');
  assert.strictEqual(L.n, 300);
});
ok('a fresh learner predicts the plain physics call', () => {
  const L = new WL.Learner(), p = L.predict([{ lkey: 'A#0', x: 3 }, { lkey: 'B#0', x: 4 }], 0);
  near(p.mean, 7, 1e-12); assert(p.sd > 0);
});
ok('saved state round-trips', () => {
  const L = new WL.Learner();
  L.update([{ lkey: 'A#0', x: 5 }], 1, 6, 1);
  const M = new WL.Learner(JSON.parse(JSON.stringify(L.toJSON())));
  near(M.predict([{ lkey: 'A#0', x: 5 }], 1).mean, L.predict([{ lkey: 'A#0', x: 5 }], 1).mean, 1e-12);
});
ok('meters that always agree are reported as tied', () => {
  const L = new WL.Learner();
  for (let i = 0; i < 200; i++) { const w = 10 + 2 * gauss(); L.update([{ lkey: 'A#0', x: 0.5 * w }, { lkey: 'B#0', x: 0.5 * w }], 6, 0.9 * w + 0.5 * gauss(), 0.6); }
  assert(L.corr('A#0', 'B#0', 6) < -0.45, 'corr ' + L.corr('A#0', 'B#0', 6));
});
ok('sector names and lookup', () => { assert.strictEqual(WL.SECTOR_NAMES[WL.sectorOf(270)], 'W'); assert.strictEqual(WL.sectorOf(359), 0); near(WL.fromDeg({ x: 10, y: 0 }), 270, 1e-9); });

// ---- the page: served like the base serves it, no internet, no console errors
async function pageTest() {
  const { chromium } = require('playwright');
  const server = http.createServer((req, res) => {
    if (req.url === '/learn') { res.writeHead(200, { 'content-type': 'text/html' }); res.end(PAGE); }
    else if (req.url === '/wind-learner.js') { res.writeHead(200, { 'content-type': 'application/javascript' }); res.end(LIB); }
    else { res.writeHead(404); res.end(); }
  });
  await new Promise(ok => server.listen(0, ok));
  const port = server.address().port, b = await chromium.launch(), p = await b.newPage({ viewport: { width: 400, height: 860 } });
  const errs = []; p.on('pageerror', e => errs.push(e.message)); p.on('console', m => { if (m.type() === 'error') errs.push(m.text()); });
  await p.route(/^https?:\/\/(?!127\.0\.0\.1)/, rt => { errs.push('page tried to reach ' + rt.request().url()); rt.abort(); });
  await p.goto('http://127.0.0.1:' + port + '/learn');
  await p.waitForFunction(() => /[LR0]/.test(document.getElementById('callLearn').textContent), null, { timeout: 15000 });
  await p.click('#burstL');
  const n = await p.evaluate(() => ST.shots.filter(s => s.plotted).length);
  assert.strictEqual(n, 25, 'Fire 25 plots 25 shots');
  assert(await p.evaluate(() => RT.learner.n === 25), 'learner saw 25 shots');
  assert(await p.evaluate(() => !!document.querySelector('a[href="/"]')), 'back link to Wind Lab');
  const wide = await p.evaluate(() => document.documentElement.scrollWidth > innerWidth + 1);
  assert(!wide, 'page scrolls sideways at phone width');
  await b.close(); server.close();
  assert.deepStrictEqual(errs, []);
  pass++; console.log('ok  Learn page runs offline at phone width, Fire 25 trains the model');
}
(process.argv.includes('--page') ? pageTest() : Promise.resolve()).then(() => console.log(pass + ' passed'), e => { console.error(e); process.exit(1); });
