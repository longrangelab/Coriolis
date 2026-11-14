// Gear from the Skyward Ballistics app is view only on this page (lib/WebUI/GearJs.h):
// the page shows the base's file exactly as Skyward sent it, never merges into it or
// writes back, refuses to save gear, and "Edit gear here instead" hands it back.
// Run: node test_gear_skyward.js
const load = require('./load_page_js.js');
let fails = 0;
function check(name, cond, extra) { if (!cond) { fails++; console.log('FAIL', name, extra === undefined ? '' : extra); } else console.log('ok  ', name); }

/** A base station like WebUI.h: GET /gear/info, GET /gear, POST /gear?base=<ver>. */
function fakeBase(initial) {
  const b = { text: initial || '', ver: 5, posts: [] };
  const resp = (status, body, headers) => ({
    status, ok: status >= 200 && status < 300,
    headers: { get: k => (headers || {})[k] || null },
    json: async () => JSON.parse(typeof body === 'string' ? body : JSON.stringify(body)),
    arrayBuffer: async () => new TextEncoder().encode(body).buffer,
  });
  b.fetch = async (url, opt) => {
    const path = url.split('?')[0];
    if (path === '/gear/info') return resp(200, { ok: true, ver: b.ver, bytes: b.text.length, gz: false, max: 384 * 1024 });
    if (path === '/gear' && (!opt || !opt.method || opt.method === 'GET'))
      return b.text ? resp(200, b.text, { 'X-Gear-Ver': String(b.ver) }) : resp(204, '');
    if (path === '/gear' && opt.method === 'POST') {
      const base = +new URL('http://x' + url).searchParams.get('base');
      if (base !== b.ver) return resp(409, { ok: false, err: 'conflict' });
      b.text = await opt.body.get('file').text(); b.ver++; b.posts.push(JSON.parse(b.text));
      return resp(200, { ok: true, ver: b.ver });
    }
    return resp(404, '');
  };
  return b;
}
const globals = b => ({ fetch: b.fetch, TextDecoder, TextEncoder, Blob, FormData, URL });

const T = 1762990000000;
function skyFile(mv, extra) {
  return JSON.stringify(Object.assign({
    format: 'skyward-library', version: 1, kind: 'master', app: 'Skyward Ballistics',
    rifles: [{ id: 'r1', name: 'Tikka CTR', twistIn: 8, twistDirection: 'RIGHT', sightHeightIn: 1.9, updatedAt: T }],
    loads: [{ id: 'l1', rifleId: 'r1', name: 'ELD-M 140', bulletId: 'b1', muzzleVelocityFps: mv, zeroRangeYd: 100, updatedAt: T }],
    bullets: [{ id: 'b1', maker: 'Maker', name: '6.5mm 140', diameterIn: 0.264, weightGr: 140, dragModel: 'G7',
      bcPoints: [{ bc: 0.315, minVelFps: 0 }], dragCurve: [], source: 'user', updatedAt: T }],
    deleted: [], cards: [], shots: [],
  }, extra || {}));
}
function throwsManaged(f) { try { f(); return false; } catch (e) { return !!e.managed; } }

(async () => {
  {
    // a phone that had its own gear joins a base with Skyward's gear
    const b = fakeBase(skyFile(2705));
    const st = {};
    const G = load(st, globals(b)).Gear;
    G.saveRecord('rifle', { id: 'mine', name: 'Phone-only rifle', twistIn: 10, twistDirection: 'RIGHT', sightHeightIn: 1.5 });
    await G.sync();
    check('Skyward file makes the gear view only', G.managed() === true);
    check('page shows exactly what Skyward sent', G.rifles().length === 1 && G.rifles()[0].id === 'r1' && G.load('l1').muzzleVelocityFps === 2705,
      JSON.stringify(G.rifles().map(r => r.id)));
    check('nothing written to the base', b.posts.length === 0);
    check('status says view only', /Skyward/.test(G.meta().status), G.meta().status);
    check('saveRecord refused', throwsManaged(() => G.saveRecord('load', Object.assign({}, G.load('l1'), { muzzleVelocityFps: 2800 }))));
    check('removeRecord refused', throwsManaged(() => G.removeRecord('rifle', 'r1')));
    check('saveFlat refused', throwsManaged(() => G.saveFlat('X', G.flat('l1'), 'l1', false)));
    check('removeLoad refused', throwsManaged(() => G.removeLoad('l1')));
    check('removeBullets refused', throwsManaged(() => G.removeBullets(['b1'])));
    check('import refused', throwsManaged(() => G.importText(skyFile(2600))));
    check('the gear is unchanged after the refusals', G.load('l1').muzzleVelocityFps === 2705 && G.rifles().length === 1);
    check('the message points to Skyward', /Skyward/.test(G.MANAGED_MSG));
    // the solver can still use the gear
    check('solving still works', G.solve(Object.assign(G.flat('l1'), { loadId: 'l1', range: 600, windmph: 10, windrel: 90 })).ok !== false);

    // syncing again with nothing new: still nothing written
    await G.sync();
    check('no write on the next sync either', b.posts.length === 0);

    // Skyward sends a change, including a deletion of something it no longer has
    b.text = skyFile(2690, { rifles: [], loads: [], bullets: [] }); b.ver++;
    await G.sync();
    check('Skyward\'s next send replaces the page copy', G.rifles().length === 0 && G.loads().length === 0);
    check('still nothing written', b.posts.length === 0);
    check('the phone copy is kept in localStorage', JSON.parse(st.gearLib).rifles.length === 0);

    // "Edit gear here instead"
    b.text = skyFile(2705); b.ver++;
    await G.sync();
    await G.takeOver();
    check('take over: editable again', G.managed() === false);
    check('take over: written to the base as this page\'s', b.posts.length === 1 && b.posts[0].app === 'Coriolis', JSON.stringify(b.posts.map(p => p.app)));
    check('take over: same gear', b.posts[0].loads.length === 1 && b.posts[0].loads[0].muzzleVelocityFps === 2705);
    let ok = true; try { G.saveRecord('load', Object.assign({}, G.load('l1'), { muzzleVelocityFps: 2720 })); } catch (e) { ok = false; }
    check('take over: saving works', ok && G.load('l1').muzzleVelocityFps === 2720);
    await G.sync();
    check('take over: the edit reaches the base', b.posts.length === 2 && JSON.parse(b.text).loads[0].muzzleVelocityFps === 2720);
    // Skyward sends again: view only again
    b.text = skyFile(2705); b.ver++;
    await G.sync();
    check('a later Skyward send makes it view only again', G.managed() === true && G.load('l1').muzzleVelocityFps === 2705);
  }
  {
    // a base whose gear was saved on the Coriolis page: editable as before
    const coriolis = JSON.parse(skyFile(2705)); coriolis.app = 'Coriolis';
    const b = fakeBase(JSON.stringify(coriolis));
    const G = load({}, globals(b)).Gear;
    await G.sync();
    check('Coriolis-made gear stays editable', G.managed() === false);
    let ok = true; try { G.saveRecord('load', Object.assign({}, G.load('l1'), { muzzleVelocityFps: 2711 })); } catch (e) { ok = false; }
    check('and saves', ok);
    await G.sync();
    check('and is written to the base', b.posts.length >= 1 && JSON.parse(b.text).loads[0].muzzleVelocityFps === 2711);
  }
  {
    // an empty base: editable
    const b = fakeBase('');
    const G = load({}, globals(b)).Gear;
    await G.sync();
    check('empty base: editable', G.managed() === false);
  }
  {
    // view only survives a page reload (no base station in reach afterwards)
    const b = fakeBase(skyFile(2705));
    const st = {};
    await load(st, globals(b)).Gear.sync();
    const G2 = load(st).Gear;
    check('reloaded page remembers view only', G2.managed() === true && throwsManaged(() => G2.removeRecord('rifle', 'r1')));
  }

  if (fails) { console.log(fails + ' FAILURE(S)'); process.exit(1); }
  console.log('ALL PASS');
  process.exit(0);
})().catch(e => { console.log('CRASH', e && e.stack || e); process.exit(1); });
