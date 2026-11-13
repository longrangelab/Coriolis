// Gear store (lib/WebUI/GearJs.h): Skyward format import/export, old Coriolis
// profile migration, flat <-> records, merge rules, and that solving through
// the gear layer gives the same holds as the ESP32 solver.
// Run: node test_gear.js
const load = require('./load_page_js.js');
let fails = 0;
function check(name, cond, extra) { if (!cond) { fails++; console.log('FAIL', name, extra === undefined ? '' : extra); } else console.log('ok  ', name); }

// --- 1. migration of old Coriolis profiles ---
const oldProfiles = {
  '6.5 CM': { mv: '2710', bc: '0.315', model: '7', wgt: '140', cal: '0.264', twist: '8', blen: '1.41', twistDir: '1', sh: '1.75', zero: '100',
              zeroElevMag: '0.5', zeroElevDir: 'low', zeroWindMag: '0.25', zeroWindDir: 'right', __spin: '1', __jump: '1' },
  '308 G1':  { mv: 2650, bc: 0.505, model: 1, wgt: 175, cal: 0.308, twist: 10, blen: '', twistDir: -1, sh: 1.5, zero: 200,
              zeroElevMag: 0, zeroElevDir: 'high', zeroWindMag: 0, zeroWindDir: 'right' },
};
let store = { profiles: JSON.stringify(oldProfiles), activeProfile: '6.5 CM' };
let w = load(store);
let G = w.Gear;
check('migrated 2 profiles', G.migrated === 2 && G.loads().length === 2);
check('old profiles key kept as backup', store.profiles === JSON.stringify(oldProfiles));
const act = store.activeLoadId;
check('active profile carried over', act && G.load(act) && G.load(act).name === '6.5 CM');
let f = G.flat(act);
check('flat round trip: mv/bc/model', f.mv === 2710 && f.bc === 0.315 && f.model === '7', JSON.stringify(f));
check('flat round trip: group offsets', f.zeroElevMag === 0.5 && f.zeroElevDir === 'low' && f.zeroWindMag === 0.25 && f.zeroWindDir === 'right', JSON.stringify(f));
const f2 = G.flat(G.loads().find(l => l.name === '308 G1').id);
check('flat: G1, left twist, no length', f2.model === '1' && f2.twistDir === -1 && f2.blen === '' && f2.zero === 200);
check('migration is idempotent (key now exists)', load(store).Gear.migrated === 0);

// --- 2. export is a valid Skyward master library ---
const ex = JSON.parse(G.exportText('master'));
check('export header', ex.format === 'skyward-library' && ex.version === 1 && ex.kind === 'master');
const req = { rifles: ['id', 'name'], loads: ['id', 'rifleId', 'name', 'bulletId', 'muzzleVelocityFps'], bullets: ['id', 'name', 'diameterIn', 'weightGr'] };
let missing = [];
for (const k of Object.keys(req)) for (const rec of ex[k]) for (const fld of req[k]) if (rec[fld] === undefined || rec[fld] === null) missing.push(k + '.' + fld);
check('export has every field Skyward requires', missing.length === 0, missing);
check('enums use Skyward names', ex.bullets.every(b => ['G1', 'G7', 'CUSTOM'].includes(b.dragModel)) && ex.rifles.every(r => ['RIGHT', 'LEFT'].includes(r.twistDirection)));
check('loads carry rifleName', ex.loads.every(l => typeof l.rifleName === 'string'));

// --- 3. import a Skyward file (multi-BC bullet, custom curve, override, tombstone) ---
const T = 1762990000000;
const sky = {
  format: 'skyward-library', version: 1, kind: 'master', app: 'Skyward Ballistics',
  rifles: [{ id: 'r1', name: 'Tikka CTR', twistIn: 8, twistDirection: 'RIGHT', sightHeightIn: 1.9, sightOffsetIn: 0, updatedAt: T }],
  loads: [
    { id: 'l1', rifleId: 'r1', rifleName: 'Tikka CTR', name: 'ELD-M 140', bulletId: 'b1', muzzleVelocityFps: 2705, powderTempF: 70, velocityPerDegFps: 1.2,
      zeroRangeYd: 100, zeroHeightIn: 0, zeroOffsetIn: 0, dragScaleFactor: 0.98, updatedAt: T },
    { id: 'l2', rifleId: 'r1', name: 'Curve load', bulletId: 'b2', muzzleVelocityFps: 2650, zeroRangeYd: 100, updatedAt: T },
    { id: 'l3', rifleId: 'r1', name: 'Override load', bulletId: 'b1', muzzleVelocityFps: 2700, zeroRangeYd: 100,
      bcPointsOverride: [{ bc: 0.3, minVelFps: 0 }], updatedAt: T },
  ],
  bullets: [
    { id: 'b1', maker: 'Hornady', name: '6.5mm 140 ELD-M', diameterIn: 0.264, weightGr: 140, lengthIn: 1.42, dragModel: 'G7',
      bcPoints: [{ bc: 0.326, minVelFps: 2800 }, { bc: 0.315, minVelFps: 2200 }, { bc: 0.305, minVelFps: 0 }], dragCurve: [], source: 'user', updatedAt: T },
    { id: 'b2', maker: 'Lab', name: 'Curve bullet', diameterIn: 0.264, weightGr: 140, lengthIn: 1.41, dragModel: 'CUSTOM', bcPoints: [],
      dragCurve: [{ mach: 0, cd: 0.25 }, { mach: 0.9, cd: 0.28 }, { mach: 1.0, cd: 0.55 }, { mach: 1.5, cd: 0.5 }, { mach: 3, cd: 0.38 }], source: 'library', collection: 'Lab', updatedAt: T },
  ],
  deleted: [{ kind: 'load', id: 'gone', deletedAt: T }], cards: [{ id: 'c1', name: 'Card', rifleId: 'r1', loadId: 'l1', targets: [], updatedAt: T }], shots: [],
};
let res = G.importText(JSON.stringify(sky));
check('Skyward import counts', res.loads === 3 && res.bullets === 2 && res.rifles === 1, res);
check('5 loads now', G.loads().length === 5);
check('label shows rifle + load', G.label(G.load('l1')) === 'Tikka CTR · ELD-M 140');
let fl = G.flat('l1');
check('multi-BC shown as bands text', fl.bcBands === '0.326@2800, 0.315@2200, 0.305@0' && fl.bc === 0.326, fl.bcBands);
check('powder temp + drag scale come through', fl.powderTempF === 70 && fl.mvPerF === 1.2 && fl.dragScale === 0.98);
check('custom curve flat model C', G.flat('l2').model === 'C');
check('override BC shown', G.flat('l3').bc === 0.3 && G.flat('l3').bcBands === '');
check('cards preserved for round trip', JSON.parse(G.exportText('master')).cards.length === 1);

// Solving through gear == solving the same bullet directly with the engine
const E = w.SkyEngine;
function P(flat, extra) { return Object.assign({}, flat, { range: 1000, windmph: 10, windrel: 90, tempF: 59, pres: 29.92, lat: 45, az: 0, earth: '0', spin: '1', jump: '1' }, extra || {}); }
let viaGear = G.solve(P(fl, { loadId: 'l1' }));
let direct = E.solveOne({ twistIn: 8, sightHeightIn: 1.9 }, { muzzleVelocityFps: 2705, zeroRangeYd: 100, powderTempF: 70, velocityPerDegFps: 1.2, dragScaleFactor: 0.98 },
  sky.bullets[0], { distanceYd: 1000, windSpeed1Mph: 10, windAngleDeg: 90, spinDriftModel: 'LITZ_CLASSIC', atmosphere: { temperatureF: 59, pressureInHg: 29.92 } });
check('gear solve == direct engine solve (multi-BC, powder temp, drag scale)', viaGear.ok && Math.abs(viaGear.elevMil - direct.elevMil) < 1e-9, [viaGear.elevMil, direct.elevMil]);
let cs = G.solve(P(G.flat('l2'), { loadId: 'l2' }));
check('custom-curve load solves', cs.ok && cs.elevMil > 5, cs);
check('bad BC bands text refused, not guessed', G.solve(P(Object.assign({}, fl, { bcBands: '0.3@abc' }))).ok === false);

// --- 4. editing ---
const nBullets = G.bullets().length;
let id = G.saveFlat('ELD-M 140', Object.assign({}, fl, { mv: 2690 }), 'l1');
check('MV edit keeps the same load and bullet', id === 'l1' && G.load('l1').muzzleVelocityFps === 2690 && G.load('l1').bulletId === 'b1' && G.bullets().length === nBullets);
check('edit bumps updatedAt', G.load('l1').updatedAt > T);
id = G.saveFlat('Curve load', Object.assign({}, G.flat('l2'), { model: '7', bc: 0.31 }), 'l2');
const nb = G.bullet(G.load('l2').bulletId);
check('editing a library bullet makes a user copy', nb.id !== 'b2' && nb.source === 'user' && G.bullet('b2').dragModel === 'CUSTOM', nb);
id = G.saveFlat('Override load', Object.assign({}, G.flat('l3'), { bc: 0.29 }), 'l3');
check('override edit goes to the load, bullet untouched', G.load('l3').bcPointsOverride[0].bc === 0.29 && G.bullet('b1').bcPoints.length === 3);
const asNew = G.saveFlat('ELD-M 140 cold', Object.assign({}, G.flat('l1'), { mv: 2650 }), 'l1', true);
check('Save As makes a new load on the same rifle and bullet', asNew !== 'l1' && G.load(asNew).rifleId === 'r1' && G.load(asNew).bulletId === 'b1' && G.load('l1').muzzleVelocityFps === 2690);
const fresh = G.saveFlat('New rifle', { mv: 2800, bc: 0.25, model: '7', wgt: 77, cal: 0.224, twist: 7, blen: 0.99, twistDir: 1, sh: 2, zero: 100, zeroElevMag: 1, zeroElevDir: 'high', zeroWindMag: 0, zeroWindDir: 'right' }, null);
check('new profile makes rifle + load + bullet', G.load(fresh) && G.rifle(G.load(fresh).rifleId).name === 'New rifle' && G.bullet(G.load(fresh).bulletId).weightGr === 77);
check('group offset stored as inches at zero', Math.abs(G.load(fresh).zeroHeightIn - 1.0472) < 1e-3);

// --- 5. merge rules ---
const N = Date.now() - 1000;   // tombstones older than 180 days are pruned, so use recent times
const A = { rifles: [{ id: 'x', name: 'old', updatedAt: N + 10 }], loads: [], bullets: [], deleted: [], cards: [], shots: [] };
G.mergeInto(A, { rifles: [{ id: 'x', name: 'new', updatedAt: N + 20 }] });
check('newer copy wins', A.rifles[0].name === 'new');
G.mergeInto(A, { rifles: [{ id: 'x', name: 'stale', updatedAt: N + 15 }] });
check('older copy loses', A.rifles[0].name === 'new');
G.mergeInto(A, { deleted: [{ kind: 'rifle', id: 'x', deletedAt: N + 30 }] });
check('tombstone removes', A.rifles.length === 0);
G.mergeInto(A, { rifles: [{ id: 'x', name: 'resurrect?', updatedAt: N + 25 }] });
check('copy saved before the delete stays deleted', A.rifles.length === 0);
G.mergeInto(A, { rifles: [{ id: 'x', name: 'edited after', updatedAt: N + 40 }] });
check('copy edited after the delete comes back', A.rifles.length === 1);

// --- 6. delete ---
G.removeLoad(fresh);
check('delete load removes its own rifle and bullet, leaves a tombstone', !G.load(fresh) && G.lib().deleted.some(d => d.kind === 'load' && d.id === fresh) &&
  !G.lib().rifles.some(r => r.name === 'New rifle'));

// --- 7. re-importing an old Coriolis file does not duplicate ---
const before = G.loads().length;
G.importText(JSON.stringify(oldProfiles));
check('re-import of rifle_profiles.json updates in place', G.loads().length === before);

// --- 8. custom drag curve typed in the page ---
check('curve text: pairs', JSON.stringify(G.parseCurve('0.9:0.15, 0.5:0.142 ,1.0:0.29')) === JSON.stringify([{ mach: 0.5, cd: 0.142 }, { mach: 0.9, cd: 0.15 }, { mach: 1, cd: 0.29 }]));
check('curve text: pasted two-column table with header', G.parseCurve('mach,cd\n0.5,0.142\n1.0\t0.29\n2 0.25').length === 3);
check('curve text: round trip', G.curveToText(G.parseCurve('0.5:0.142, 1:0.29')) === '0.5:0.142, 1:0.29');
let cerr = ''; try { G.parseCurve('0.5:0.142'); } catch (e) { cerr = e.message; }
check('curve text: one point refused', /at least 2/.test(cerr));
// G7 reference curve as a custom curve, Cd = i * Cd_G7 with i = SD/BC: must solve like G7 with that BC
const g7 = w.SkyEngine.G7, bcC = 0.315, sdC = 140 / 7000 / (0.264 * 0.264), ff = sdC / bcC, pts = [];
for (let k = 0; k < g7.length; k += 2) if (g7[k] <= 3) pts.push(g7[k] + ':' + (g7[k + 1] * ff).toFixed(6));
const cFlat = { mv: 2700, bc: 0.315, model: 'C', wgt: 140, cal: 0.264, twist: 8, blen: 1.41, twistDir: 1, sh: 1.75, zero: 100, dragCurve: pts.join(', ') };
const cid = G.saveFlat('Curve typed', cFlat, null, false);
const cb = G.bullet(G.load(cid).bulletId);
check('typed curve saved on the bullet as CUSTOM', cb.dragModel === 'CUSTOM' && cb.dragCurve.length === pts.length);
check('typed curve comes back in flat', G.flat(cid).dragCurve.split(',').length === pts.length && G.flat(cid).model === 'C');
const env = { range: 1000, tempF: 59, pres: 29.92, windmph: 10, windrel: 90, spin: '0', jump: '0' };
const sC = G.solve(Object.assign({}, G.flat(cid), env, { loadId: cid }));
const sG = G.solve(Object.assign({}, cFlat, { model: '7', dragCurve: '' }, env));
check('typed G7-shaped curve solves like G7 (1000 yd)', sC.ok && sG.ok && Math.abs(sC.elevMil - sG.elevMil) < 0.05, [sC.elevMil, sG.elevMil]);
const sNo = G.solve(Object.assign({}, cFlat, { dragCurve: '', bulletId: '' }, env));
check('Custom with no curve gives a clear error', !sNo.ok && /drag curve/i.test(sNo.err), sNo);

// --- 9. errors ---
let threw = '';
try { G.importText(JSON.stringify({ format: 'epv5-user-data' })); } catch (e) { threw = e.message; }
check('Empirical backup gets a helpful message', /Empirical Precision/.test(threw));
try { threw = ''; G.importText('{"format":"skyward-library","version":2,"kind":"master"}'); } catch (e) { threw = e.message; }
check('newer Skyward version refused', /version 2/.test(threw));

if (fails) { console.log(fails + ' FAILURE(S)'); process.exit(1); }
console.log('ALL PASS');
process.exit(0);
