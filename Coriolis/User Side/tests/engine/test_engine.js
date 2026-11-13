// Checks the phone engine (lib/WebUI/EngineJs.h) against the ESP32 solver
// (golden_ballistics_h.json, made by make_golden.cpp from Ballistics.h).
// Run: node test_engine.js   (exit code 1 on any failure)
const E = require('./load_engine.js')();
const golden = require('./golden_ballistics_h.json');

// Coriolis flat profile -> Skyward rifle / load / bullet / shot
function gear(p) {
  return {
    rifle: { twistIn: p.twist, twistDirection: p.twistDir < 0 ? 'LEFT' : 'RIGHT', sightHeightIn: p.sh },
    load: { muzzleVelocityFps: p.mv, zeroRangeYd: p.zero },
    bullet: { dragModel: p.model === 1 ? 'G1' : 'G7', bcPoints: [{ bc: p.bc, minVelFps: 0 }],
              weightGr: p.wgt, diameterIn: p.cal, lengthIn: p.blen },
    shot: { distanceYd: p.range, windSpeed1Mph: p.windmph, windAngleDeg: p.windrel, aeroJump: true, spinDrift: true,
            spinDriftModel: 'LITZ_CLASSIC', coriolis: !!p.earth, latitudeDeg: p.lat, azimuthDeg: p.az,
            atmosphere: { temperatureF: p.tempF, pressureInHg: p.pres, pressureIsAbsolute: true, humidityPct: 0 } },
  };
}

// Tolerances. The engines differ only in how they stop at the target (the ESP32
// takes the first 0.5 ms step past it, the phone interpolates) and in float vs
// double inputs, so these are tight; a physics change blows straight through them.
const TOL = { elevMil: 0.03, windMil: 0.01, vRemain: 2.0, tof: 0.002, spinDriftIn: 0.05, aeroJumpMOA: 0.005, sg: 0.01 };
let fails = 0, worst = {};
for (const c of golden.solve) {
  const g = gear(c.in);
  const o = E.solveOne(g.rifle, g.load, g.bullet, g.shot);
  if (!o.ok) { fails++; console.log('NOT OK', c.name, c.in.range, o.err); continue; }
  for (const k of Object.keys(TOL)) {
    const d = Math.abs(o[k] - c.out[k]);
    if (!(d <= TOL[k])) { fails++; console.log(`FAIL ${c.name} ${c.in.range} yd ${k}: phone ${o[k].toFixed(4)} esp32 ${c.out[k]}`); }
    if (!worst[k] || d > worst[k]) worst[k] = d;
  }
}
console.log(`${golden.solve.length} shots vs Ballistics.h, largest gaps:`,
  Object.fromEntries(Object.entries(worst).map(([k, v]) => [k, +v.toFixed(4)])));

// Wind sensitivity curves: same shape as the ESP32 sweep, within 2% of the peak.
for (const c of golden.sensitivity) {
  const name = c.name; const base = golden.solve.find(s => s.name === name).in;
  const g = gear(Object.assign({}, base, { range: c.range }));
  const s = E.windSensitivity(g.rifle, g.load, g.bullet, g.shot.atmosphere, c.range);
  const peak = Math.max(...c.coef);
  let bad = !s.ok || s.coef.length !== c.coef.length;
  if (!bad) s.coef.forEach((v, i) => { if (Math.abs(v - c.coef[i]) > 0.02 * peak) bad = true; });
  if (bad) { fails++; console.log('FAIL sensitivity', name, c.range, s.ok && s.coef.slice(0, 4), c.coef.slice(0, 4)); }
}
console.log(`${golden.sensitivity.length} sensitivity curves checked`);

// Skyward-only features: sanity and invariants (no ESP32 reference exists).
function base() { return gear({ mv: 2710, bc: 0.315, model: 7, wgt: 140, cal: 0.264, twist: 8, twistDir: 1, blen: 1.41, sh: 1.75,
  zero: 100, range: 1000, windmph: 10, windrel: 90, lat: 45, az: 90, earth: 0, tempF: 59, pres: 29.92 }); }
function check(name, cond) { if (!cond) { fails++; console.log('FAIL', name); } }
{ // one band == same BC listed twice
  const a = base(), b = base();
  b.bullet.bcPoints = [{ bc: 0.315, minVelFps: 2000 }, { bc: 0.315, minVelFps: 0 }];
  check('identical bands = single BC', Math.abs(E.solveOne(a.rifle, a.load, a.bullet, a.shot).elevMil - E.solveOne(b.rifle, b.load, b.bullet, b.shot).elevMil) < 1e-9);
  // lower BC below 2000 fps -> more drop at 1000 yd
  b.bullet.bcPoints = [{ bc: 0.315, minVelFps: 2000 }, { bc: 0.29, minVelFps: 0 }];
  check('lower low-speed band = more come-up', E.solveOne(b.rifle, b.load, b.bullet, b.shot).elevMil > E.solveOne(a.rifle, a.load, a.bullet, a.shot).elevMil + 0.05);
}
{ // custom curve = G7 table scaled by i/(BC) reproduces the G7 solve
  const a = base(), b = base();
  const sd = 140 / 7000 / (0.264 * 0.264), i = sd / 0.315, curve = [];
  for (let k = 0; k < E.G7.length; k += 2) curve.push({ mach: E.G7[k], cd: E.G7[k + 1] * i });
  b.bullet.dragModel = 'CUSTOM'; b.bullet.dragCurve = curve;
  const d = Math.abs(E.solveOne(a.rifle, a.load, a.bullet, a.shot).elevMil - E.solveOne(b.rifle, b.load, b.bullet, b.shot).elevMil);
  check('custom curve equal to G7/BC matches G7 (' + d.toFixed(5) + ')', d < 0.002);
}
{ // powder temperature: +10 F at 1.5 fps/F == +15 fps MV
  const a = base(), b = base();
  b.load.velocityPerDegFps = 1.5; b.load.powderTempF = 49;
  a.load.muzzleVelocityFps = 2725;
  check('powder temp sensitivity', Math.abs(E.solveOne(a.rifle, a.load, a.bullet, a.shot).elevMil - E.solveOne(b.rifle, b.load, b.bullet, b.shot).elevMil) < 1e-9);
}
{ // zero height: group 0.5 MOA high at 100 yd -> about 0.5 MOA less come-up everywhere
  const a = base(), b = base();
  b.load.zeroHeightIn = 0.5 * 1.04720 * 1.0;
  const d = (E.solveOne(a.rifle, a.load, a.bullet, a.shot).elevMOA - E.solveOne(b.rifle, b.load, b.bullet, b.shot).elevMOA);
  check('zero height 0.5 MOA high -> 0.5 MOA less come-up (' + d.toFixed(3) + ')', Math.abs(d - 0.5) < 0.02);
  const c = base(); c.load.zeroOffsetIn = 0.5 * 1.0472;
  const w = (E.solveOne(c.rifle, c.load, c.bullet, c.shot).windMOA - E.solveOne(a.rifle, a.load, a.bullet, a.shot).windMOA);
  check('zero offset 0.5 MOA right -> hold 0.5 MOA left (' + w.toFixed(3) + ')', Math.abs(w + 0.5) < 0.02);
}
{ // truing recovers a known MV
  const a = base(), t = base();
  t.load.muzzleVelocityFps = 2680;
  const obs = [600, 800, 1000].map(r => ({ rangeYd: r, actualComeUpMil: E.solveOne(t.rifle, t.load, t.bullet, Object.assign({}, t.shot, { distanceYd: r })).elevMil }));
  const res = E.trueMuzzleVelocity(a.rifle, a.load, a.bullet, a.shot, obs);
  check('truing finds MV 2680 (' + res.value.toFixed(1) + ')', Math.abs(res.value - 2680) < 1);
}
{ // refusals instead of confident garbage
  const a = base(); a.load.zeroRangeYd = 0;
  check('zero 0 refused', !E.solveOne(a.rifle, a.load, a.bullet, a.shot).ok);
  const b = base(); b.bullet.bcPoints = [{ bc: 0, minVelFps: 0 }];
  check('BC 0 refused', !E.solveOne(b.rifle, b.load, b.bullet, b.shot).ok);
}
{ // speed
  const a = base(); const t0 = Date.now(); let n = 0;
  while (Date.now() - t0 < 500) { E.solveOne(a.rifle, a.load, a.bullet, a.shot); n++; }
  const t1 = Date.now(); E.windSensitivity(a.rifle, a.load, a.bullet, a.shot.atmosphere, 1000);
  console.log(`speed: ${(500 / n).toFixed(2)} ms per 1000 yd solve, ${Date.now() - t1} ms per 1000 yd sensitivity sweep`);
}
if (fails) { console.log(fails + ' FAILURE(S)'); process.exit(1); }
console.log('ALL PASS');
