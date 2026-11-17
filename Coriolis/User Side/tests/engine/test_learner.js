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

// ---- relationships between meters
function relData(seconds, opts) {
  opts = opts || {};
  const R = new WL.Relations(), rr = rng(opts.seed || 7), gg = () => { const u = 1 - rr(), v = rr(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); };
  const G = []; let x = 0;
  for (let t = 0; t < seconds + 40; t++) { x = 0.9 * x + 0.9 * gg(); G.push(x); }
  const dir = opts.dir == null ? 270 : opts.dir;
  for (let t = 20; t < seconds + 20; t++) {
    const rd = { A: { spd: 10 + 2 * G[t] + 0.3 * gg(), dir: dir + 3 * gg() }, B: { spd: 0.8 * (10 + 2 * G[t - 12]) + 0.3 * gg(), dir: dir + 10 + 5 * gg() }, C: { spd: 10 + 2 * gg(), dir: dir } };
    if (t % 50 === 0) delete rd.C;            // a meter that drops a reading now and then
    R.ingest(t, rd);
  }
  return R;
}
const REL = relData(3000);
ok('relations: finds who leads, by how long, and the speed ratio', () => {
  const p = REL.pair('B', 'A', 6);
  assert.strictEqual(p.lead, 'A'); assert.strictEqual(p.follow, 'B'); assert.strictEqual(p.lagS, 12);
  assert(p.r > 0.85 && p.significant, 'r ' + p.r);   // this test air keeps a fixed 12 s whatever the speed; learning is by yards, so a little blur
  near(p.ratio, 0.8, 0.03, 'ratio'); near(p.gain, 0.8, 0.05, 'gain'); near(p.dirOff, 10, 1.5, 'dir offset');
  assert(p.skill > 0.5, 'beats persistence ' + p.skill);
  assert.strictEqual(WL.relationKind(p), 'leads');
});
ok('relations: an unrelated meter reads as different air', () => {
  const p = REL.pair('A', 'C', 6);
  assert(!p.significant || p.r < 0.2, 'r ' + p.r); assert.strictEqual(WL.relationKind(p), 'independent');
});
ok('relations: best predictor and Sentinel suggestion', () => {
  const S = REL.summary(6);
  assert.strictEqual(S.predictors.B.by, 'A'); assert.strictEqual(S.predictors.B.lagS, 12);
  const sg = REL.suggestSentinels(6);
  assert.strictEqual(sg.length, 1);
  assert.deepStrictEqual([sg[0].sentId, sg[0].nodeId, sg[0].lagS], ['A', 'B', 12]);
  assert(sg[0].spdPct >= 8 && sg[0].spdPct <= 50 && sg[0].dirDeg >= 8 && sg[0].dirDeg <= 45);
  near(sg[0].ratio, 0.8, 0.03);
});
ok('relations: each wind direction is learned on its own', () => {
  assert.strictEqual(REL.pair('A', 'B', 2), null);
  assert(REL.sectorSecs[6] > 2900 && REL.sectorSecs[2] === 0);
  near(REL.pair('A', 'B', 'all').lagS, 12, 0);
});
ok('relations: a wind on a sector edge counts for both sectors', () => {
  const R = relData(400, { dir: 250 });       // 2.5 deg inside W, next to SW
  assert(R.sectorSecs[6] > 300 && R.sectorSecs[5] > 300, R.sectorSecs.join(','));
  const C = relData(400, { dir: 270 });
  assert.strictEqual(C.sectorSecs[5], 0);
});
ok('relations: saved state round-trips', () => {
  const R2 = new WL.Relations(JSON.parse(JSON.stringify(REL.toJSON())));
  const a = REL.pair('A', 'B', 6), b = R2.pair('A', 'B', 6);
  assert.strictEqual(b.lagS, a.lagS); near(b.r, a.r, 1e-4); near(b.ratio, a.ratio, 1e-4);
});
ok('relations: frozen-air lag from positions', () => {
  const e = WL.expectedLag({ x: -100, y: 0 }, { x: 0, y: 0 }, { x: 10, y: 0 });
  near(e.lagS, 100 / (10 * WL.YD_PER_S_PER_MPH), 1e-9); near(e.crossYd, 0, 1e-9);
  assert(WL.upwindOk({ x: -100, y: 0 }, { x: 0, y: 0 }, { x: 10, y: 0 }));
  assert(!WL.upwindOk({ x: 100, y: 0 }, { x: 0, y: 0 }, { x: 10, y: 0 }));
});
ok('second aligner: buckets readings by second, in order, once', () => {
  const al = new WL.SecondAligner(5), out = [];
  al.add('1', 10.2, 5, 270); al.add('2', 9.8, 6, 270); al.add('1', 11.1, 7, 270); al.add('1', 9.6, 4, 260);
  al.release(16, (t, r) => out.push([t, r]));
  assert.deepStrictEqual(out.map(o => o[0]), [10, 11]);
  near(out[0][1]['1'].spd, 4.5, 1e-9); near(out[0][1]['2'].spd, 6, 1e-9);
  al.add('1', 10.4, 9, 270);                  // too late: second 10 already went out
  al.release(30, (t, r) => out.push([t, r]));
  assert.strictEqual(out.length, 2);
});
ok('live model: one row per wind sector for Wind Lab', () => {
  const L = new WL.Learner();
  for (let i = 0; i < 300; i++) { const a = 10 + 4 * gauss(), c = 10 + 4 * gauss(); L.update([{ lkey: 'P1#0', x: 0.6 * a }, { lkey: 'P2#0', x: 0.4 * 0.7 * c }], 6, 0.6 * a + 0.4 * c + 0.3 * gauss(), 0.6); }
  const meters = [
    { id: 'P1', nodeId: 1, lkey: 'P1#0', x: 0, y: 100 }, { id: 'P2', nodeId: 2, lkey: 'P2#0', x: 10, y: 500 },
    { id: 'S', nodeId: 5, lkey: 'S#0', x: -250, y: 450, role: 'sentinel' }];
  const R = new WL.Relations();             // S leads P2 by 8 s; also a fake P2-leads-S pairing must be dropped by geometry
  const rr = rng(3), gg = () => { const u = 1 - rr(), v = rr(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); };
  const G = []; let x = 0; for (let t = 0; t < 2100; t++) { x = 0.9 * x + gg(); G.push(x); }
  for (let t = 20; t < 2000; t++) R.ingest(t, { S: { spd: 10 + 2 * G[t], dir: 270 }, P2: { spd: 9 + 2 * G[t - 8], dir: 270 }, P1: { spd: 10 + 2 * gg(), dir: 270 } });
  const lm = WL.buildLiveModel({ name: 'T', meters, shooter: { x: 0, y: 0 }, target: { x: 0, y: 800 }, kernel, learner: L, relations: R, windMph: 10 });
  assert.strictEqual(lm.kind, 'coriolis-live-model'); assert.strictEqual(lm.sectors.length, 8);
  const W = lm.sectors[6];
  near(W.wPct[1] + W.wPct[2], 100, 0.2); assert.strictEqual(W.wPct[5], 0, 'a sentinel takes no path weight');
  near(W.mult[2], 1 / 0.7, 0.15, 'learned multiplier');
  assert.strictEqual(W.sent.length, 1); assert.deepStrictEqual([W.sent[0].sentId, W.sent[0].nodeId, W.sent[0].lagS], [5, 2, 8]);
  // wind from the east: S is downwind, so it may not warn anyone
  assert.strictEqual(lm.sectors[2].sent.length, 0);
});

// ---- forecasting each meter's wind a few seconds ahead
function fcRun(useLeader) {
  const rr = rng(21), gg = () => { const u = 1 - rr(), v = rr(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); };
  const F = new WL.Forecaster(null, { useLeader, leaderFn: id => id === 'B' ? { by: 'A', lagS: 15, ratio: 0.9 } : null });
  const G = []; let x = 0; for (let t = 0; t < 1300; t++) { x = 0.95 * x + 0.6 * gg(); G.push(x); }
  for (let t = 20; t < 1220; t++) { F.add('A', t, 10 + 3 * G[t] + 0.2 * gg(), 270); F.add('B', t, 0.9 * (10 + 3 * G[t - 15]) + 0.2 * gg(), 270); }
  return F;
}
ok('forecaster: an upwind early warning makes the next 10 s predictable', () => {
  const S = fcRun(true).stats('B'), N = fcRun(false).stats('B');
  const s10 = S.find(s => s.h === 10), n10 = N.find(s => s.h === 10);
  assert(s10.skill > 0.8, 'with the leader, 10 s skill ' + s10.skill);
  assert(n10.skill < 0.3, 'without it, gusts are mostly a surprise ' + n10.skill);
  assert(s10.wLeader > 0.4 && N.find(s => s.h === 10).wLeader < 0.1, 'leans on the leader (u carries all of a west wind) ' + s10.wLeader);
  assert(S.find(s => s.h === 20).skill < s10.skill, 'past the 15 s warning the forecast fades');
});
ok('forecaster: predict interpolates, reports doubt and round-trips', () => {
  const F = fcRun(true), t = F.latest('B');
  const p0 = F.predict('B', t), p7 = F.predict('B', t + 7);
  assert(p0.ready && p0.sd === 0 && p7.ready && p7.sd > 0 && p7.sd < 1, JSON.stringify(p7));
  near(p7.h, 7, 1e-9);
  const F2 = new WL.Forecaster(JSON.parse(JSON.stringify(F.toJSON())));
  near(F2.stats('B')[2].rmse, F.stats('B')[2].rmse, 1e-4);
});
// ---- the wind model's self-improving loop (no shots)
function loopRun(evolve, seconds) {
  const rr = rng(3), gg = () => { const u = 1 - rr(), v = rr(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); };
  const G = []; let x = 0; for (let t = 0; t < seconds + 100; t++) { x = 0.95 * x + 0.6 * gg(); G.push(x); }
  const L = new WL.ForecastLoop(null, { seed: 7, evolve, regimes: false });          // no hints: it has to find the pair itself
  for (let t = 50; t < seconds + 50; t++) { L.add('S', t, 10 + 3 * G[t] + 0.2 * gg(), 270); L.add('N', t, 0.9 * (10 + 3 * G[t - 22]) + 0.2 * gg(), 270 + 3 * gg()); L.add('X', t, 10 + 2 * gg(), 270); }
  return L;
}
const LOOP = loopRun(true, 2700);
ok('wind loop: finds a hidden Sentinel pair by trying, and proves it', () => {
  const best = LOOP.pool('N', 10).find(c => c.champ);
  assert(best.leaders.some(l => l.by === 'S' && Math.abs(l.lag - 22) <= 2), 'best model leans on S about 22 s back: ' + best.desc);
  const pp = LOOP.provenPairs().filter(q => q.nodeId === 'N');
  assert(pp.length && pp[0].sentId === 'S' && pp[0].gain > 0.5, JSON.stringify(pp));
  assert(!LOOP.provenPairs().some(q => q.nodeId === 'X'), 'unrelated air proves nothing');
  const s10 = LOOP.stats('N').find(q => q.h === 10);
  assert(s10.skill > 0.6, 'checked 10 s skill ' + s10.skill);   // 0.3-0.8 across seeds, it depends on when the pair turns up
});
ok('wind loop: skill climbs by generation; without evolving it stays on own history', () => {
  const H = LOOP.history('N').map(q => q.skill[10]).filter(v => v != null);
  const early = H.slice(0, 3).reduce((a, b) => a + b, 0) / 3, late = H.slice(-5).reduce((a, b) => a + b, 0) / 5;
  assert(late > early + 0.2, 'early ' + early + ' late ' + late);
  const F = loopRun(false, 1200);
  assert(F.pool('N', 10).every(c => !c.leaders.length) && F.stats('N')[2].skill < 0.3, 'fixed pool');
  assert(LOOP.log('N').some(q => /Retired/.test(q.txt)) && LOOP.log('N').some(q => /Trying/.test(q.txt)));
});
ok('wind loop: every prediction is checked only after its wind arrives, and the loop round-trips', () => {
  const I = LOOP.info('N'), S = LOOP.stats('N');
  S.forEach(q => assert(q.n <= 2700 - q.h && q.n >= 2700 - q.h - 5, 'h ' + q.h + ' n ' + q.n));
  assert(I.checked === S.reduce((a, q) => a + q.n, 0));
  const L2 = new WL.ForecastLoop(JSON.parse(JSON.stringify(LOOP.toJSON())));
  assert.strictEqual(L2.info('N').champ, I.champ); near(L2.stats('N')[2].rmse, S[2].rmse, 1e-4);
  const p = LOOP.predict('N', LOOP.latest('N') + 10);
  assert(p.ready && p.sd > 0 && p.sd < 1.5, JSON.stringify(p));
});
ok('wind loop: a base restart (clock back to zero) does not stop the checking', () => {
  const rr = rng(4), gg = () => { const u = 1 - rr(), v = rr(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); };
  const L = new WL.ForecastLoop(null, { seed: 3, regimes: false });
  for (let t = 2000; t < 3200; t++) { L.addEnv(t, { pressure: 1000 + gg() }); L.add('A', t, 10 + 2 * gg(), 270); }
  const before = L.info('A').checked, gen = L.info('A').generation;
  for (let t = 5; t < 400; t++) { L.addEnv(t, { pressure: 1000 + gg() }); L.add('A', t, 10 + 2 * gg(), 270); }
  assert(L.info('A').checked > before + 1000, 'checks carry on after the restart: ' + before + ' -> ' + L.info('A').checked);
  assert(L.info('A').generation > gen, 'generations carry on');
  assert(L.latest('A') === 399 && Object.keys(L.env.s).every(k => +k < 400), 'old seconds dropped');
});
ok('wind loop: a new neural net gets 30 min before it can be retired (unless worse than "stays"), two at a time at most', () => {
  const rr = rng(8), gg = () => { const u = 1 - rr(), v = rr(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); };
  const G = []; let x = 0; for (let t = 0; t < 14500; t++) { x = 0.95 * x + 0.6 * gg(); G.push(x); }
  const L = new WL.ForecastLoop(null, { seed: 5, maxCandidates: 10, regimes: false }), born = {}, ages = [], scores = {};
  let young = 0;
  const orig = L._generation.bind(L);
  L._generation = function (id, p, sec) {
    const ids = new Set(p.list.map(c => c.id));
    p.list.forEach(c => { scores[id + ':' + c.id] = this._score(p, c); });   // what this generation judges by
    orig(id, p, sec);
    p.list.forEach(c => { if (c.kind === 'mlp' && born[id + ':' + c.id] == null) born[id + ':' + c.id] = c.born; });
    ids.forEach(i => { if (!p.list.some(c => c.id === i) && born[id + ':' + i] != null) ages.push([p.g - born[id + ':' + i], scores[id + ':' + i]]); });
    young = Math.max(young, p.list.filter(c => c.kind === 'mlp' && p.g - c.born < 20 && c.born > 0).length);
  };
  for (let t = 50; t < 14450; t++) { L.add('S', t, 10 + 3 * G[t] + 0.2 * gg(), 270); L.add('N', t, 10 + 3 * G[t - 12] + 0.3 * Math.max(0, G[t - 12]) ** 2 + 0.2 * gg(), 270); }
  assert(Object.keys(born).length >= 2, 'nets get tried: ' + JSON.stringify(born));
  assert(ages.every(a => a[0] >= 20 || a[1] > 1.05), 'no net retired before 20 generations unless worse than "stays": ' + JSON.stringify(ages));
  assert(young <= 2, 'at most two young nets at once: ' + young);
});
// two clusters: in a west wind A1 is 20 s upwind of A2, in a north wind B1 is 25 s upwind of it
function clusterRun(regimes, segS, nSeg) {
  const rr = rng(11), gg = () => { const u = 1 - rr(), v = rr(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); };
  const T = segS * nSeg + 200, GA = [], GB = []; let a = 0, b = 0;
  for (let t = 0; t < T; t++) { a = 0.95 * a + 0.6 * gg(); b = 0.95 * b + 0.6 * gg(); GA.push(a); GB.push(b); }
  const L = new WL.ForecastLoop(null, { seed: 4, regimes, speedBands: false }), pend = {}; let se = 0, n = 0;
  for (let t = 100; t < T - 50; t++) {
    const seg = Math.floor((t - 100) / segS), west = seg % 2 === 0, dir = west ? 270 : 0;
    const R = { A1: 10 + 3 * GA[t], B1: 10 + 3 * GB[t], A2: west ? 10 + 3 * GA[t - 20] : 10 + 3 * GB[t - 25] };
    for (const id in R) L.add(id, t, R[id] + 0.3 * gg(), dir + 4 * gg());
    const d = dir * Math.PI / 180, last = L.rt.A2.s[t], q = pend[t];
    if (q && seg >= 2) { se += ((q.x - last.u) ** 2 + (q.y - last.v) ** 2) / 2; n++; }
    const f = L.predict('A2', t + 10); if (f) pend[t + 10] = f;
  }
  return { L, rmse: Math.sqrt(se / n) };
}
ok('wind loop: each wind direction keeps what it learned, and gets it back when the wind returns', () => {
  const M = clusterRun(true, 1800, 8), A = clusterRun(false, 1800, 8);
  const names = M.L.regimeList().map(r => r.name);
  assert(names.includes('From W') && names.includes('From N'), names.join(','));
  assert(M.L.log('A2').some(q => /brought back/.test(q.txt)), 'models come back when the wind does');
  assert(M.rmse < 0.9 * A.rmse, 'remembering beats relearning: ' + M.rmse.toFixed(3) + ' vs ' + A.rmse.toFixed(3));
  const W = M.L.regimeList().find(r => r.name === 'From W'), N = M.L.regimeList().find(r => r.name === 'From N');
  assert(W.meters.A2.leaders.some(l => l.by === 'A1' && Math.abs(l.lag - 20) <= 2) || N.meters.A2.leaders.some(l => l.by === 'B1' && Math.abs(l.lag - 25) <= 2), 'the right Sentinel per wind: ' + JSON.stringify([W.meters.A2, N.meters.A2]));
  const L2 = new WL.ForecastLoop(JSON.parse(JSON.stringify(M.L.toJSON())));
  assert.strictEqual(L2.regimeList().length, M.L.regimeList().length, 'what it knows per wind is saved');
  assert.strictEqual(L2.regime(), M.L.regime());
});
ok('relations: learned in yards of air travel, so two wind speeds agree', () => {
  const R = new WL.Relations(), rr = rng(21), gg = () => { const u = 1 - rr(), v = rr(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); };
  // B is 60 yd downwind of A: 12.3 s at 10 mph, 6.1 s at 20 mph
  const G = []; let x = 0; for (let t = 0; t < 4000; t++) { x = 0.9 * x + 0.9 * gg(); G.push(x); }
  for (let t = 20; t < 3600; t++) {
    const mph = t < 1800 ? 10 : 20, lag = 60 / (mph * WL.YD_PER_S_PER_MPH), k = Math.floor(lag), w = lag - k;
    const gB = (1 - w) * G[t - k] + w * G[t - k - 1];
    R.ingest(t, { A: { spd: mph + 2 * G[t], dir: 270 }, B: { spd: mph + 2 * gB, dir: 270 } });
  }
  const p = R.pair('A', 'B', 6);
  assert(p.lead === 'A' && Math.abs(p.alongYd - 60) <= 6 && p.r > 0.8, JSON.stringify(p));
  near(p.lagF, 60 / (p.atMph * WL.YD_PER_S_PER_MPH), 1.5, 'seconds at the speed it learned in');
});
ok('wind loop: other sensors are optional inputs that must earn their place', () => {
  const rr = rng(13), gg = () => { const u = 1 - rr(), v = rr(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); };
  const G = []; let x = 0; for (let t = 0; t < 3200; t++) { x = 0.97 * x + 0.4 * gg(); G.push(x); }
  const L = new WL.ForecastLoop(null, { seed: 5, regimes: false });
  // a base pressure reading that moves 20 s before the wind does; a meter light reading that means nothing
  for (let t = 50; t < 3050; t++) { L.addEnv(t, { pressure: 1000 + G[t + 20] }); L.add('N', t, 10 + 3 * G[t] + 0.2 * gg(), 270, { light: 500 + 50 * gg() }); }
  assert.deepStrictEqual(L.channels(), ['light', 'pressure']);
  const best = L.pool('N', 10).find(c => c.champ);
  assert(best.inputs.some(q => q.k === 'pressure' && q.lag >= 10), 'best leans on pressure, read some seconds back: ' + best.desc);
  assert(L.stats('N')[3].skill > 0.5, '20 s skill ' + L.stats('N')[3].skill);
  assert(!L.pool('N', 10).filter(c => c.n >= 300).some(c => c.inputs.length === 1 && c.inputs[0].k === 'light' && c.skill > 0.1), 'the meaningless sensor earns nothing');
  const L2 = new WL.ForecastLoop(JSON.parse(JSON.stringify(L.toJSON())));
  assert.strictEqual(JSON.stringify(L2.pool('N', 10).find(c => c.champ).inputs), JSON.stringify(best.inputs));
});
function chRun(adapt, change) {
  const rr = rng(5), gg = () => { const u = 1 - rr(), v = rr(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); };
  const L = new WL.Learner(null, { adapt }), errs = [];
  for (let i = 0; i < 300; i++) {
    const a = 10 + 4 * gg(), c = 10 + 4 * gg(), cal = change && i >= 150 ? 0.6 : 1;   // P1 gets sheltered at shot 150
    const f = [{ lkey: 'P1#0', x: 0.5 * a * cal }, { lkey: 'P2#0', x: 0.5 * c }], truth = 0.5 * a + 0.5 * c;
    errs.push(L.predict(f, 6).mean - truth); L.update(f, 6, truth + 0.5 * gg(), 0.6);
  }
  const r = (a, b) => Math.sqrt(errs.slice(a, b).reduce((s, v) => s + v * v, 0) / (b - a));
  return { ev: L.events, mid: r(165, 200), late: r(250, 300) };
}
ok('change watch: a meter that suddenly reads low is caught and relearned fast', () => {
  assert.strictEqual(chRun(true, false).ev.length, 0, 'no false alarm in steady conditions');
  const A = chRun(true, true), N = chRun(false, true);
  assert(A.ev.length >= 1 && A.ev[0].n > 150 && A.ev[0].n <= 165, JSON.stringify(A.ev));
  assert(A.mid < 0.75 * N.mid, 'relearns faster ' + A.mid + ' vs ' + N.mid);
});
ok('noise scale: the stated doubt learns its real size', () => {
  const rr = rng(9), gg = () => { const u = 1 - rr(), v = rr(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); };
  const L = new WL.Learner(), H = new WL.Learner();
  for (let i = 0; i < 120; i++) { const a = 10 + 3 * gg(), f = [{ lkey: 'A#0', x: a }]; L.update(f, 6, a + 1.2 * gg(), 0.6); H.update(f, 6, a + 0.6 * gg(), 0.6); }
  near(L.noiseScale(), 2, 0.5, 'surprises twice the stated noise'); near(H.noiseScale(), 1, 0.3, 'honest noise');
  assert(L.events.length === 0, 'a noisier rifle is not a change ' + JSON.stringify(L.events));
  near(new WL.Learner(JSON.parse(JSON.stringify(L.toJSON()))).noiseScale(), L.noiseScale(), 1e-9);
});
ok('hit chance: plate size, spread and offset', () => {
  near(WL.hitChance({ windSd: 0, rifleSd: 0.1, halfW: 10, halfH: 10 }), 1, 1e-6);
  const c = WL.hitChance({ windSd: 0.3, rifleSd: 0.2, halfW: 0.3, halfH: 0.4 });
  assert(c > 0.45 && c < 0.65, c);
  assert(WL.hitChance({ windSd: 0.3, rifleSd: 0.2, halfW: 0.3, halfH: 0.4, offset: 0.3 }) < c);
  assert(WL.hitChance({ windSd: 0, rifleSd: 0.2, halfW: 0.3, halfH: 0.3, round: true }) < WL.hitChance({ windSd: 0, rifleSd: 0.2, halfW: 0.3, halfH: 0.3 }));
});

ok('layout pairs: each meter gets the meter straight upwind, from GPS spots alone', () => {
  const M = [{ id: 'P1', x: 0, y: 100 }, { id: 'P2', x: 10, y: 500 }, { id: 'S', x: -250, y: 480, role: 'sentinel' }, { id: 'F', x: -200, y: 120 }];
  const W = WL.geometryPairs(M, { x: 10, y: 0 });                     // wind from the west
  const by = {}; W.forEach(q => { by[q.nodeId] = q; });
  assert.strictEqual(by.P2.sentId, 'S'); near(by.P2.lagS, 260 / (10 * WL.YD_PER_S_PER_MPH), 1); assert.strictEqual(by.P2.src, 'layout');
  assert.strictEqual(by.P1.sentId, 'F', 'F is 200 yd upwind of P1: ' + JSON.stringify(by.P1));
  assert(!by.S, 'a Sentinel-only meter warns, it is not warned');
  assert.strictEqual(WL.geometryPairs(M, { x: -10, y: 0 }).filter(q => q.sentId === 'S').length, 0, 'east wind: S is downwind');
  assert.strictEqual(WL.geometryPairs(M, { x: 0, y: 0 }).length, 0, 'no wind, no pairs');
});

// ---- the page: served like the base serves it, no internet, no console errors
async function pageTest() {
  const { chromium } = require('playwright');
  // a mock base station: three meters on GPS north of the base; meter 2's
  // gusts reach meter 1 six seconds later; /windsec keeps two minutes
  const t0 = Date.now() - 120000, YDD = 111319.4908 * 1.0936133;
  const gust = t => 8 + 2 * Math.sin(2 * Math.PI * t / 37) + 1.3 * Math.sin(2 * Math.PI * t / 13 + 1);
  const spd = { 1: t => gust(t - 6), 2: t => gust(t), 3: t => 8 + 1.5 * Math.sin(2 * Math.PI * t / 23 + 2) };
  // meter sensors: 1 has none (old fields read 0 / -1), 2 sends the env object
  // newer firmware adds, 3 only the older tempF / presInHg / lux fields
  const sens = { 1: { tempF: 32, presInHg: 0, lux: -1 }, 2: { tempF: 55.2, presInHg: 29.89, lux: 20000, env: { pressure: 1012.34, temp: 55.2, light: 20000 } }, 3: { tempF: 54, presInHg: 29.9, lux: 300 } };
  const node = (id, yd) => Object.assign({ id, speed: spd[id]((Date.now() - t0) / 1000), speedSd: 1, dir: 270, dirSd: 5, lat: 45 + yd / YDD, lon: -93, gps: true, windOk: true, age: 1 }, sens[id]);
  const server = http.createServer((req, res) => {
    const u = new URL(req.url, 'http://x'), json = o => { res.writeHead(200, { 'content-type': 'application/json' }); res.end(JSON.stringify(o)); };
    if (u.pathname === '/learn') { res.writeHead(200, { 'content-type': 'text/html' }); res.end(PAGE); }
    else if (u.pathname === '/wind-learner.js') { res.writeHead(200, { 'content-type': 'application/javascript' }); res.end(LIB); }
    else if (u.pathname === '/data.json') json({ base: { lat: 45, lon: -93, gps: true, env: { pressure: 1013 + (Date.now() % 7000) / 7000 } }, nodes: [node(1, 120), node(2, 400), node(3, 650)] });
    else if (u.pathname === '/windsec') {
      const now = Date.now() - t0, since = u.searchParams.has('since') ? +u.searchParams.get('since') : -1, nodes = {};
      for (const id of [1, 2, 3]) {
        const r = [];
        for (let t = Math.max(0, now - 120000) - (now % 1000) + id * 7; t <= now; t += 1000) if (t > since) r.push(id === 1 ? [t, +spd[id](t / 1000).toFixed(2), 270, { temp: 60 + t / 60000 }] : [t, +spd[id](t / 1000).toFixed(2), 270]);
        nodes[id] = { src: 'beacon', period: 1000, r };
      }
      json({ now, utc: null, avg_ms: 2000, nodes });
    }
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
  assert(await p.evaluate(() => ST.shots.every(s => s.callSd > 0 && s.hitP > 0 && s.hitP <= 1)), 'each shot keeps the stated doubt and hit chance');
  await p.waitForFunction(() => document.querySelectorAll('#outlook > div').length === 4, null, { timeout: 5000 });
  assert(/hit chance/.test(await p.textContent('#callLearnS')), 'learned call shows its hit chance');
  await p.click('#tab-learn');
  assert(await p.evaluate(() => /×/.test(document.getElementById('kCal').textContent) && document.querySelectorAll('#fcTbl tbody tr').length >= 3 && !!document.querySelector('#chart polyline[stroke-dasharray]')), 'Learning tab: doubt check, forecast table, doubt line');
  await p.click('#tab-pred'); await p.click('#pTrain');
  await p.waitForFunction(() => !document.getElementById('pTrain').disabled && RT.fc.info('M3').generation >= 20, null, { timeout: 60000 });
  await p.click('#tab-shoot');
  assert(await p.evaluate(() => !!document.querySelector('a[href="/"]')), 'back link to Wind Lab');
  const wide = await p.evaluate(() => document.documentElement.scrollWidth > innerWidth + 1);
  assert(!wide, 'page scrolls sideways at phone width');
  // relationships learned in practice, and the model sent to Wind Lab
  await p.evaluate(() => advance(600));
  await p.click('#tab-rel');
  assert(await p.evaluate(() => RT.rel.secs > 500 && document.querySelectorAll('#relTbl tbody tr').length === 10), 'relations table for 5 meters');
  assert(await p.evaluate(() => RT.pairsNow.some(q => q.sentId === 'M5' && q.nodeId === 'M3') && /M5 → M3/.test(document.getElementById('relSent').textContent)), 'M5 warns M3 (layout or data), shown in the Sentinel table');
  await p.click('#relTbl tbody tr'); assert(await p.evaluate(() => document.querySelector('#relChart polyline') !== null), 'lag chart drawn');
  await p.click('#tab-model'); await p.click('#pushWL');
  const lm = await p.evaluate(() => JSON.parse(localStorage.getItem('learnedModel')));
  assert(lm && lm.sectors.length === 8 && lm.shots === 25 && await p.evaluate(() => localStorage.getItem("learnedOn") === "1"), 'Wind Lab model written');
  await p.fill('#modelName', 'Test range'); await p.click('#modelSave');
  assert(await p.evaluate(() => !!JSON.parse(localStorage.getItem('windLearningRange.models'))['Test range']), 'model saved');
  await p.click('#stopWL'); assert(await p.evaluate(() => localStorage.getItem('learnedOn') === '0'), 'Stop turns Wind Lab back to its own settings');
  // practice meters carry pressure / temp / light; pressure reads the air
  // ~15 s ahead, so the loop should pick it up on its own
  await p.click('#tab-pred'); await p.click('#pTrain'); await p.waitForFunction(() => !document.getElementById('pTrain').disabled, null, { timeout: 60000 });
  const sc = await p.evaluate(() => ({ ch: RT.fc.channels().join(','), top: RT.fc.meters().map(id => RT.fc.pool(id, 10).filter(c => c.n >= 120).slice(0, 3).map(c => c.inputs)), champs: RT.fc.meters().map(id => (RT.fc.pool(id, 10).find(c => c.champ) || { inputs: [] }).inputs),
    note: RT.fc.meters().filter(id => (RT.fc.pool(id, 10).find(c => c.champ) || { inputs: [] }).inputs.some(q => q.k === 'pressure')).map(id => sensorVerdict(RT.fc, id)).join('') }));
  assert.strictEqual(sc.ch, 'light,pressure,temp', 'practice: simulated sensors reach the wind model');
  assert(sc.top.some(L => L.some(I => I.some(q => q.k === 'pressure'))), 'practice: the leading pressure is found on its own ' + JSON.stringify(sc.top));
  assert(!sc.note || /10 s ahead than its best without|no sensor helps yet|checked enough/.test(sc.note), 'Predict tab says what the sensors are worth: ' + sc.note);
  assert(await p.evaluate(() => { RT.regAt = 0; renderRegimes(RT.fc); return document.querySelectorAll('#pRegs tbody tr').length >= 1 && /\(now\)/.test(document.getElementById('pRegs').textContent); }), 'Predict tab lists what it knows, wind by wind');
  assert(await p.evaluate(() => document.querySelectorAll('#pPool tbody tr').length >= 4 && !!document.querySelector('#pHist polyline') && !!document.querySelector('#pTrace polyline') && +document.getElementById('kPc').textContent.replace(/,/g, '') > 10000), 'Predict tab: pool, skill by generation, predicted vs actual');
  // live mode against the mock base
  await p.click('#modeLive');
  await p.waitForFunction(() => ST.mode === 'live' && ST.meters.length === 3 && RT.calls, null, { timeout: 15000 });
  const geo = await p.evaluate(() => ST.meters.map(m => [m.id, m.x, m.y, m.src]));
  assert(Math.abs(geo[1][2] - 400) < 1 && Math.abs(geo[1][1]) < 1 && geo[1][3] === 'GPS', 'meter 2 placed 400 yd north from GPS ' + JSON.stringify(geo));
  await p.click('#tab-shoot'); await p.click('#shoot');
  assert(await p.evaluate(() => !!RT.awaiting && RT.awaiting.live), 'live SHOT waits for a plot');
  const box = await p.locator('#tgt').boundingBox();
  await p.locator('#tgt').click({ position: { x: box.width * 0.6, y: box.height * 0.46 } });
  const shot = await p.evaluate(() => ({ n: RT.learner.n, s: ST.shots[0] }));
  assert(shot.n === 1 && shot.s.plotted && Math.abs(shot.s.perfect - (shot.s.dial - shot.s.impact.x)) < 1e-9, 'live shot plotted and learned ' + JSON.stringify(shot));
  await p.waitForFunction(() => RT.rel.secs > 112, null, { timeout: 20000 });
  const lp = await p.evaluate(() => RT.rel.pair('1', '2', 6, { minN: 80 }));
  assert(lp && lp.lead === '2' && lp.lagS === 6 && lp.r > 0.9, 'live: meter 2 warns meter 1 six seconds ahead ' + JSON.stringify(lp));
  assert(await p.evaluate(() => RT.fc.stats('1')[0].n > 60), 'live: the forecaster trains on the 1 s readings');
  assert.strictEqual(await p.evaluate(() => RT.fc.channels().join(',')), 'light,pressure,temp', 'live: extra sensors from the base and the meters reach the wind model');
  const ne = await p.evaluate(() => RT.nodeEnv);
  assert(!ne['1'] && ne['2'].x.pressure === 1012.34 && Math.abs(ne['2'].x.light - Math.log10(20001)) < 1e-3 && Math.abs(ne['3'].x.pressure - 29.9 * 33.86389) < 0.01 && ne['3'].x.temp === 54,
    'live: meter sensors from the env object and from the older fields, none for a meter without them ' + JSON.stringify(ne));
  // reload: it comes back in live mode with the shot
  await p.evaluate(() => autosave());
  await p.reload();
  await p.waitForFunction(() => ST.mode === 'live' && ST.shots.length === 1, null, { timeout: 10000 });
  await p.click('#modeSim');
  assert(await p.evaluate(() => ST.mode === 'sim' && ST.shots.length === 25), 'practice range comes back with its shots');
  // a live poll still in flight when you switch to Practice must not land there
  await p.click('#modeLive');
  await p.waitForFunction(() => ST.mode === 'live' && ST.meters.length === 3, null, { timeout: 15000 });
  await p.route('**/data.json', async rt => { await new Promise(r => setTimeout(r, 1500)); rt.continue().catch(() => {}); });
  await p.waitForTimeout(2100);                 // the 2 s poll is now waiting on the slow base
  await p.click('#modeSim'); await p.waitForTimeout(2500);
  const pm = await p.evaluate(() => ({ mode: ST.mode, ids: ST.meters.map(m => m.id).join(','), shots: ST.shots.length }));
  assert(pm.mode === 'sim' && /^M1,M2/.test(pm.ids) && pm.shots === 25, 'Live -> Practice mid-poll keeps the practice range ' + JSON.stringify(pm));
  await p.unroute('**/data.json');
  // a fresh range keeps "Keep Wind Lab updated"
  await p.evaluate(() => { $('autoPush').checked = true; $('autoPush').onchange(); $('sessNew').click(); });
  assert(await p.evaluate(() => ST.autoPush === true && $('autoPush').checked && ST.shots.length === 0), 'fresh range keeps Keep Wind Lab updated');
  await b.close(); server.close();
  assert.deepStrictEqual(errs, []);
  pass++; console.log('ok  Learn page: offline at phone width, Fire 25 trains, relations, model to Wind Lab, live mode on a mock base');
}
(process.argv.includes('--page') ? pageTest() : Promise.resolve()).then(() => console.log(pass + ' passed'), e => { console.error(e); process.exit(1); });
