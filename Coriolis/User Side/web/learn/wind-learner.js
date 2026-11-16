/*
 * Coriolis wind learner: shared module.
 * Runs in the Wind Learning Range simulator today and is written to drop into
 * Wind Lab later. No DOM, no globals except window.WindLearner.
 *
 * Coordinates: yards on a flat local grid, x = east, y = north, shooter at
 * shooter.{x,y}. Wind vectors are the direction the air MOVES TO, in mph,
 * {x: east, y: north}.
 *
 * Model (per shot):
 *   effective crosswind = b + sum_k (m_k + d_k,sector) * w_k * cross_k
 *   w_k     physics weight of meter k's stretch of the bullet path (geometry)
 *   cross_k meter k's crosswind, read at the moment its air crosses the path
 *   m_k     learned multiplier for meter k at this spot (prior 1)
 *   d_k,s   extra multiplier for wind from sector s (terrain, prior 0)
 *   b       constant offset in mph (zero / spin error, prior 0)
 * Learned with a Kalman filter (Bayesian linear regression updated each shot,
 * with slow forgetting so it can follow changes).
 */
(function (root) {
  'use strict';
  var YD_PER_S_PER_MPH = 0.488889;
  var SECTORS = 8;
  var SECTOR_NAMES = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'];

  function lineOfFire(shooter, target) {
    var dx = target.x - shooter.x, dy = target.y - shooter.y;
    var R = Math.hypot(dx, dy) || 1;
    var u = { x: dx / R, y: dy / R };       // downrange
    var r = { x: u.y, y: -u.x };            // to the shooter's right
    var bearing = (Math.atan2(dx, dy) * 180 / Math.PI + 360) % 360;
    return { R: R, u: u, r: r, bearing: bearing };
  }

  // cross: + = air moving to the right (pushes the bullet right)
  // head:  + = air moving toward the shooter
  function components(v, lof) {
    return { cross: v.x * lof.r.x + v.y * lof.r.y, head: -(v.x * lof.u.x + v.y * lof.u.y) };
  }

  // wind "from" direction in degrees (meteorological) for a moves-to vector
  function fromDeg(v) {
    return (Math.atan2(-v.x, -v.y) * 180 / Math.PI + 360) % 360;
  }
  function sectorOf(deg) {
    return Math.floor((((deg % 360) + 360 + 22.5) % 360) / 45) % SECTORS;
  }

  /*
   * Which meters see the air the bullet will fly through.
   * meters: [{id, x, y}], wind: field-average moves-to vector (mph)
   * opts: upwindReach / downwindReach (yd, how far off the line a meter can
   *       sit and still count), beyond (yd past the target), minCross (mph),
   *       maxLead (s)
   */
  function geometry(meters, shooter, target, wind, opts) {
    opts = opts || {};
    var upReach = opts.upwindReach || 150, downReach = opts.downwindReach || 50;
    var beyond = opts.beyond == null ? 40 : opts.beyond;
    var minCross = opts.minCross || 2, maxLead = opts.maxLead || 30;
    var lof = lineOfFire(shooter, target);
    var c = components(wind, lof).cross;
    var rows = meters.map(function (m) {
      var px = m.x - shooter.x, py = m.y - shooter.y;
      var s = px * lof.u.x + py * lof.u.y;
      var l = px * lof.r.x + py * lof.r.y;       // + = right of the line
      var row = { id: m.id, s: s, l: l, side: 'on line', lead: 0, relevance: 1, included: true, reason: '' };
      if (s < -beyond) { row.included = false; row.relevance = 0; row.side = 'behind'; row.reason = 'Behind the shooter'; return row; }
      if (s > lof.R + beyond) { row.included = false; row.relevance = 0; row.side = 'past target'; row.reason = 'Past the target'; return row; }
      var off = Math.abs(l);
      var speed = Math.max(Math.abs(c), minCross) * YD_PER_S_PER_MPH;
      if (off >= 15) {
        var upwind = (l * c) < 0;                  // wind blowing right: left side is upwind
        row.side = upwind ? 'upwind' : 'downwind';
        row.lead = (upwind ? 1 : -1) * off / speed; // s until (or since) its air crosses the line
        var reach = upwind ? upReach : downReach;
        row.relevance = Math.exp(-Math.pow(off / reach, 2));
        if (upwind && row.lead > maxLead) row.relevance *= 0.5;
      }
      if (row.relevance < 0.3) { row.included = false; row.reason = row.side === 'downwind' ? 'Downwind: its air already crossed the path' : 'Too far off the line'; }
      return row;
    });
    return { lof: lof, cross: c, rows: rows };
  }

  /*
   * Physics weights: each meter owns the stretch of the path closest to it
   * (split halfway between meters), weighted by how much wind on that stretch
   * moves the bullet. kernel(a, b) -> fraction of total deflection from wind
   * between a and b yards.
   */
  function physicsWeights(rows, R, kernel, filter) {
    var use = rows.filter(function (r) { return filter ? filter(r) : r.included; })
      .map(function (r) { return { id: r.id, s: Math.max(0, Math.min(R, r.s)) }; })
      .sort(function (a, b) { return a.s - b.s; });
    var w = {};
    if (!use.length) return w;
    var tot = 0;
    for (var i = 0; i < use.length; i++) {
      var a = i === 0 ? 0 : (use[i - 1].s + use[i].s) / 2;
      var b = i === use.length - 1 ? R : (use[i].s + use[i + 1].s) / 2;
      w[use[i].id] = Math.max(0, kernel(a, b));
      tot += w[use[i].id];
    }
    if (tot > 0) for (var k in w) w[k] /= tot;
    return w;
  }

  // ---------------------------------------------------------------- learner
  function Learner(state, cfg) {
    cfg = cfg || {};
    this.priorMsd = cfg.priorMsd || 0.25;
    this.priorDsd = cfg.priorDsd || 0.15;
    this.priorBsd = cfg.priorBsd || 0.15;
    this.q = cfg.q == null ? 2e-4 : cfg.q;
    this.keys = []; this.idx = {}; this.mean = []; this.cov = []; this.n = 0;
    this.anchor = {};
    if (state) {
      this.keys = state.keys.slice(); this.mean = state.mean.slice();
      this.cov = state.cov.map(function (r) { return r.slice(); });
      this.n = state.n || 0; this.anchor = state.anchor || {};
      for (var i = 0; i < this.keys.length; i++) this.idx[this.keys[i]] = i;
    }
    if (this.idx.b == null) this._add('b', 0, this.priorBsd);
  }
  Learner.prototype._add = function (key, mu, sd) {
    var n = this.keys.length;
    this.keys.push(key); this.idx[key] = n; this.mean.push(mu);
    for (var i = 0; i < n; i++) this.cov[i].push(0);
    var row = new Array(n + 1).fill(0); row[n] = sd * sd; this.cov.push(row);
  };
  // lkey identifies a meter AT a spot; a meter that moves gets a new lkey
  Learner.prototype.ensure = function (lkey) {
    if (this.idx['m:' + lkey] != null) return;
    this._add('m:' + lkey, 1, this.priorMsd);
    for (var s = 0; s < SECTORS; s++) this._add('d:' + lkey + ':' + s, 0, this.priorDsd);
  };
  Learner.prototype._h = function (feats, sector) {
    var h = new Array(this.keys.length).fill(0);
    h[this.idx.b] = 1;
    for (var i = 0; i < feats.length; i++) {
      var f = feats[i]; this.ensure(f.lkey);
      if (h.length < this.keys.length) h = h.concat(new Array(this.keys.length - h.length).fill(0));
      h[this.idx['m:' + f.lkey]] += f.x;
      h[this.idx['d:' + f.lkey + ':' + sector]] += f.x;
    }
    if (h.length < this.keys.length) h = h.concat(new Array(this.keys.length - h.length).fill(0));
    return h;
  };
  // feats: [{lkey, x}] with x = physics weight * crosswind (mph)
  Learner.prototype.predict = function (feats, sector) {
    var h = this._h(feats, sector), n = h.length, mu = 0, v = 0;
    for (var i = 0; i < n; i++) if (h[i]) mu += h[i] * this.mean[i];
    for (i = 0; i < n; i++) if (h[i]) for (var j = 0; j < n; j++) if (h[j]) v += h[i] * this.cov[i][j] * h[j];
    return { mean: mu, sd: Math.sqrt(Math.max(v, 0)) };
  };
  // y: crosswind the impact says the bullet felt (mph); sigma: its noise (mph)
  Learner.prototype.update = function (feats, sector, y, sigma) {
    var h = this._h(feats, sector), n = h.length, i, j;
    for (i = 0; i < n; i++) this.cov[i][i] += this.q;          // slow forgetting
    var Ph = new Array(n).fill(0);
    for (i = 0; i < n; i++) { var s = 0; for (j = 0; j < n; j++) if (h[j]) s += this.cov[i][j] * h[j]; Ph[i] = s; }
    var S = sigma * sigma, pred = 0;
    for (i = 0; i < n; i++) { S += h[i] * Ph[i]; pred += h[i] * this.mean[i]; }
    var innov = y - pred;
    for (i = 0; i < n; i++) this.mean[i] += Ph[i] * innov / S;
    for (i = 0; i < n; i++) for (j = 0; j < n; j++) this.cov[i][j] -= Ph[i] * Ph[j] / S;
    this.n++;
    return { innovation: innov, predicted: pred };
  };
  Learner.prototype.meter = function (lkey, sector) {
    var im = this.idx['m:' + lkey];
    if (im == null) return { mult: 1, sd: Math.hypot(this.priorMsd, this.priorDsd), base: 1, baseSd: this.priorMsd };
    var id = this.idx['d:' + lkey + ':' + sector];
    var v = this.cov[im][im] + this.cov[id][id] + 2 * this.cov[im][id];
    return { mult: this.mean[im] + this.mean[id], sd: Math.sqrt(Math.max(v, 0)),
             base: this.mean[im], baseSd: Math.sqrt(this.cov[im][im]) };
  };
  // correlation between two meters' multipliers for one wind sector. Strongly
  // negative means the shots can't yet tell which of the two is off.
  Learner.prototype.corr = function (lkeyA, lkeyB, sector) {
    var self = this;
    function ix(k) { var a = self.idx['m:' + k], b = self.idx['d:' + k + ':' + sector]; return a == null ? null : [a, b]; }
    var A = ix(lkeyA), B = ix(lkeyB); if (!A || !B) return 0;
    function cv(P, Q) { var s = 0; P.forEach(function (i) { Q.forEach(function (j) { s += self.cov[i][j]; }); }); return s; }
    var v = cv(A, B) / Math.sqrt(Math.max(cv(A, A) * cv(B, B), 1e-12));
    return Math.max(-1, Math.min(1, v));
  };
  Learner.prototype.bias = function () {
    var i = this.idx.b; return { mean: this.mean[i], sd: Math.sqrt(this.cov[i][i]) };
  };
  Learner.prototype.toJSON = function () {
    return { keys: this.keys, mean: this.mean, cov: this.cov, n: this.n, anchor: this.anchor };
  };

  // ------------------------------------------------------------- relations
  /*
   * How the meters relate to each other, learned live from their timed 1 s
   * wind speeds (the base's /windsec readings, or the simulator's).
   *
   * For every pair of meters and every wind sector it keeps running sums of
   * the lagged cross-correlation of their gusts (speed minus a 60 s running
   * mean) for lags -L..+L seconds. From those it answers:
   *   - do the two meters see the same gusts (peak correlation r)?
   *   - which one sees them first, and by how many seconds (lag at the peak)?
   *   - does the leader predict the follower better than the follower's own
   *     past does (skill over persistence)?
   *   - how their average speeds compare (ratio) and how far apart their
   *     directions sit (offset and spread), in that wind.
   * It needs no positions; when positions are known, expectedLag() gives the
   * lag frozen air moving at the mean wind would produce, as a cross-check.
   *
   * ingest(tSec, {id: {spd, dir}}) once per second, in time order, with the
   * readings for that second (dir = degrees the wind blows FROM). Missing
   * meters are simply left out.
   */
  function Relations(state, cfg) {
    cfg = cfg || {};
    state = state || null;
    this.L = (state && state.L) || cfg.maxLag || 40;
    this.tau = (state && state.tau) || cfg.tau || 60;
    // s = sector 0..7 or 'all'
    this.P = (state && state.P) || {};     // 'a|b|s' -> {n,ab,aa,bb,ds,dc} arrays, index lag+L (lag>0: a leads b)
    this.A = (state && state.A) || {};     // 'id|s'  -> {n,c} arrays, autocovariance lags 0..L
    this.M = (state && state.M) || {};     // 'id|s'  -> [n, sum speed]
    this.secs = (state && state.secs) || 0;
    this.sectorSecs = (state && state.sectorSecs) || new Array(SECTORS).fill(0);
    this.rt = {};                          // per meter: {ema, ring} (not saved)
    this.lastT = null;
  }
  function zeros(n) { return new Array(n).fill(0); }
  function pairKey(a, b) { return String(a) < String(b) ? [String(a), String(b)] : [String(b), String(a)]; }
  Relations.prototype._ring = function (id) {
    var r = this.rt[id];
    if (!r) r = this.rt[id] = { ema: null, ring: [] };
    return r;
  };
  Relations.prototype.ingest = function (t, readings) {
    var L = this.L, self = this, id;
    // a gap in time pushes empty seconds so lags stay honest
    if (this.lastT != null) {
      var gap = Math.min(L + 1, Math.round(t - this.lastT) - 1);
      for (var g = 0; g < gap; g++) for (id in this.rt) { this.rt[id].ring.unshift(null); this.rt[id].ring.length = Math.min(this.rt[id].ring.length, L + 1); }
      if (t - this.lastT > 600) for (id in this.rt) { this.rt[id].ema = null; this.rt[id].ring = []; }   // long break: start the running means again
    }
    this.lastT = t;
    // field wind for this second picks the sector
    var sx = 0, sy = 0, cnt = 0, ids = Object.keys(readings);
    ids.forEach(function (k) {
      var r = readings[k]; if (!r || !isFinite(r.spd) || !isFinite(r.dir)) return;
      var d = r.dir * Math.PI / 180; sx += r.spd * Math.sin(d); sy += r.spd * Math.cos(d); cnt++;
    });
    // a wind within 7.5 deg of a sector edge counts for both neighbours, so a
    // wind sitting on the W/SW line does not split its learning in two
    var sectors = [];
    if (cnt && Math.hypot(sx, sy) / cnt >= 1) {
      var fd = (Math.atan2(sx, sy) * 180 / Math.PI + 360) % 360, s0 = sectorOf(fd);
      var off = ((fd - s0 * 45 + 540) % 360) - 180;        // -22.5..22.5 from the sector centre
      sectors.push(s0);
      if (off > 15) sectors.push((s0 + 1) % SECTORS); else if (off < -15) sectors.push((s0 + SECTORS - 1) % SECTORS);
    }
    // gust = speed minus its running mean (before this second)
    var present = {};
    for (id in this.rt) present[id] = false;
    ids.forEach(function (k) {
      var r = readings[k]; if (!r || !isFinite(r.spd)) return;
      var R = self._ring(k);
      // running mean for the first tau seconds, then an EMA: no start-up bias
      if (R.ema == null) { R.ema = r.spd; R.k = 0; }
      var f = r.spd - R.ema;
      R.k = (R.k || 0) + 1;
      R.ema += (r.spd - R.ema) / Math.min(R.k, self.tau);
      R.ring.unshift({ f: f, spd: r.spd, dir: r.dir });
      if (R.ring.length > L + 1) R.ring.length = L + 1;
      present[k] = true;
    });
    for (id in present) if (!present[id]) { var R0 = this.rt[id]; R0.ring.unshift(null); if (R0.ring.length > L + 1) R0.ring.length = L + 1; }
    if (!sectors.length) return;
    this.secs++;
    this._accumulate('all', present);
    for (var si = 0; si < sectors.length; si++) this._accumulate(sectors[si], present);
  };
  Relations.prototype._accumulate = function (sector, present) {
    var self = this, L = this.L;
    if (sector !== 'all') this.sectorSecs[sector]++;
    var live = Object.keys(present).filter(function (k) { return present[k]; });
    live.forEach(function (k) {
      var R = self.rt[k], cur = R.ring[0], key = k + '|' + sector;
      var Mm = self.M[key] || (self.M[key] = [0, 0]); Mm[0]++; Mm[1] += cur.spd;
      var A = self.A[key] || (self.A[key] = { n: zeros(L + 1), c: zeros(L + 1) });
      for (var j = 0; j <= L && j < R.ring.length; j++) { var o = R.ring[j]; if (!o) continue; A.n[j]++; A.c[j] += cur.f * o.f; }
    });
    var all = Object.keys(this.rt);
    for (var i = 0; i < all.length; i++) for (var j2 = i + 1; j2 < all.length; j2++) {
      var pk = pairKey(all[i], all[j2]), a = pk[0], b = pk[1];
      var Ra = this.rt[a].ring, Rb = this.rt[b].ring;
      if (!Ra[0] && !Rb[0]) continue;
      var key2 = a + '|' + b + '|' + sector;
      var P = this.P[key2] || (this.P[key2] = { n: zeros(2 * L + 1), ab: zeros(2 * L + 1), aa: zeros(2 * L + 1), bb: zeros(2 * L + 1), ds: zeros(2 * L + 1), dc: zeros(2 * L + 1) });
      for (var k = 0; k <= L; k++) {
        // +k: a at t-k with b now (a leads b by k s)
        if (Rb[0] && Ra[k]) this._add(P, L + k, Ra[k], Rb[0]);
        // -k: b at t-k with a now (b leads a by k s)
        if (k > 0 && Ra[0] && Rb[k]) this._add(P, L - k, Ra[0], Rb[k], true);
      }
    }
  };
  // x = a's sample, y = b's sample; ds/dc hold direction(b) - direction(a)
  Relations.prototype._add = function (P, ix, x, y) {
    P.n[ix]++; P.ab[ix] += x.f * y.f; P.aa[ix] += x.f * x.f; P.bb[ix] += y.f * y.f;
    var dd = (y.dir - x.dir) * Math.PI / 180; P.ds[ix] += Math.sin(dd); P.dc[ix] += Math.cos(dd);
  };
  Relations.prototype.meters = function () {
    var seen = {};
    Object.keys(this.M).forEach(function (k) { seen[k.split('|')[0]] = true; });
    Object.keys(this.rt).forEach(function (k) { seen[k] = true; });
    return Object.keys(seen).sort();
  };
  // sector: 0..7, or 'all' (every direction, kept as its own running sums)
  Relations.prototype._P = function (a, b, sector) { return this.P[a + '|' + b + '|' + sector] || null; };
  Relations.prototype._A = function (id, sector) { return this.A[id + '|' + sector] || null; };
  Relations.prototype._mean = function (id, sector) { var m = this.M[id + '|' + sector]; return m && m[0] ? m[1] / m[0] : null; };
  // autocorrelation of one meter's gusts at lag j
  Relations.prototype.autoCorr = function (id, sector, j) {
    var A = this._A(id, sector); if (!A || !A.n[0] || !A.n[j]) return 0;
    var c0 = A.c[0] / A.n[0]; return c0 > 0 ? (A.c[j] / A.n[j]) / c0 : 0;
  };
  // the lagged correlation curve: [{lag, r, n}], lag > 0 means a leads b
  Relations.prototype.curve = function (a, b, sector) {
    var pk = pairKey(a, b), flip = pk[0] !== String(a), P = this._P(pk[0], pk[1], sector), L = this.L, out = [];
    if (!P) return out;
    for (var i = 0; i <= 2 * L; i++) {
      var n = P.n[i], d = Math.sqrt(P.aa[i] * P.bb[i]);
      out.push({ lag: flip ? L - i : i - L, r: n && d > 0 ? P.ab[i] / d : 0, n: n });
    }
    if (flip) out.reverse();
    return out;
  };
  /*
   * One pair, one sector (or 'all'). Returns null until there is enough data.
   * lead/follow name the meters so that lead sees the gusts lagS seconds first.
   */
  Relations.prototype.pair = function (a, b, sector, opts) {
    opts = opts || {};
    var minN = opts.minN || 120, L = this.L;
    var pk = pairKey(a, b); a = pk[0]; b = pk[1];
    var P = this._P(a, b, sector); if (!P) return null;
    var best = -1, bestR = -2;
    for (var i = 0; i <= 2 * L; i++) {
      if (P.n[i] < minN) continue;
      var d = Math.sqrt(P.aa[i] * P.bb[i]), r = d > 0 ? P.ab[i] / d : 0;
      // ties and near-ties go to the smaller lag (simplest explanation)
      if (r > bestR + 0.005 || (Math.abs(r - bestR) <= 0.005 && Math.abs(i - L) < Math.abs(best - L))) { bestR = r; best = i; }
    }
    if (best < 0) return null;
    var k = best - L, lead = k >= 0 ? a : b, follow = k >= 0 ? b : a, lag = Math.abs(k);
    var n = P.n[best];
    // effective sample size (Bartlett): neighbouring seconds are not independent
    var T = 1;
    for (var j = 1; j <= L; j++) T += 2 * this.autoCorr(a, sector, j) * this.autoCorr(b, sector, j);
    var neff = Math.max(4, n / Math.max(1, T)), se = 1 / Math.sqrt(Math.max(1, neff - 3));
    var rPers = this.autoCorr(follow, sector, lag);
    var varLead = k >= 0 ? P.aa[best] : P.bb[best], varFol = k >= 0 ? P.bb[best] : P.aa[best];
    var gain = varLead > 0 ? P.ab[best] / varLead : 0;
    var sdFol = Math.sqrt(varFol / n);
    var ds = P.ds[best], dc = P.dc[best], Rl = Math.min(1, Math.hypot(ds, dc) / n);
    var dirOff = Math.atan2(ds, dc) * 180 / Math.PI;               // b minus a
    if (k < 0) dirOff = -dirOff;                                     // follow minus lead
    var mLead = this._mean(lead, sector), mFol = this._mean(follow, sector);
    return {
      a: a, b: b, lead: lead, follow: follow, lagS: lag, r: bestR, n: n, neff: neff, se: se,
      significant: bestR > 3 * se, rPersist: rPers, skill: bestR - Math.max(0, rPers),
      gain: gain, sdFollow: sdFol, ratio: mLead && mFol != null ? mFol / mLead : 1,
      meanLead: mLead, meanFollow: mFol,
      dirOff: dirOff, dirSd: Rl > 1e-9 ? Math.sqrt(-2 * Math.log(Rl)) * 180 / Math.PI : 180
    };
  };
  // Plain-language class for a pair result
  function relationKind(p) {
    if (!p) return 'learning';
    if (!p.significant || p.r < 0.2) return 'independent';
    if (p.lagS <= 1 && p.r >= 0.55) return 'same air';
    if (p.lagS >= 2 && p.r >= 0.3 && p.skill > 0.05) return 'leads';
    return 'related';
  }
  // Every pair for a sector, plus per meter: which other meter predicts it best
  Relations.prototype.summary = function (sector, opts) {
    var ids = this.meters(), pairs = [], best = {}, self = this;
    for (var i = 0; i < ids.length; i++) for (var j = i + 1; j < ids.length; j++) {
      var p = this.pair(ids[i], ids[j], sector, opts);
      var row = { a: ids[i], b: ids[j], p: p, kind: relationKind(p) };
      pairs.push(row);
      if (p && p.significant && p.lagS >= 1 && p.skill > 0.05) {
        var cur = best[p.follow];
        if (!cur || p.skill > cur.skill) best[p.follow] = { by: p.lead, lagS: p.lagS, r: p.r, skill: p.skill };
      }
    }
    var secs = sector === 'all' ? this.secs : this.sectorSecs[sector];
    var means = {}; ids.forEach(function (id) { means[id] = self._mean(id, sector); });
    return { sector: sector, seconds: secs, pairs: pairs, predictors: best, means: means };
  };
  /*
   * Sentinel pairs this data supports for one wind sector: the leader becomes
   * the Sentinel, the follower the paired node. Tolerances come from how far
   * the follower normally strays from what the leader said lagS seconds ago.
   */
  Relations.prototype.suggestSentinels = function (sector, opts) {
    opts = opts || {};
    var minR = opts.minR || 0.35, maxPerNode = opts.maxPerNode || 1, out = [];
    var S = this.summary(sector, opts), byFollow = {};
    S.pairs.forEach(function (row) {
      var p = row.p; if (row.kind !== 'leads' || p.r < minR || p.skill <= 0.05) return;
      (byFollow[p.follow] = byFollow[p.follow] || []).push(p);
    });
    Object.keys(byFollow).forEach(function (f) {
      byFollow[f].sort(function (x, y) { return y.skill - x.skill; }).slice(0, maxPerNode).forEach(function (p) {
        var resid = p.sdFollow * Math.sqrt(Math.max(0, 1 - p.r * p.r));
        var mean = Math.max(2, p.meanFollow || 0);
        var spdPct = Math.min(50, Math.max(8, Math.round(1.5 * resid / mean * 100 / 5) * 5));
        var dirDeg = Math.min(45, Math.max(8, Math.round(1.5 * Math.min(p.dirSd, 30) / 5) * 5));
        out.push({ sentId: p.lead, nodeId: p.follow, lagS: p.lagS, spdPct: spdPct, dirDeg: dirDeg,
                   ratio: Math.round(p.ratio * 1000) / 1000, dirOff: Math.round(p.dirOff), r: Math.round(p.r * 100) / 100,
                   skill: Math.round(p.skill * 100) / 100 });
      });
    });
    return out;
  };
  // Lag frozen air would give: seconds for the air at a to reach b's crosswise line
  function expectedLag(pa, pb, windTo) {
    var S = Math.hypot(windTo.x, windTo.y); if (S < 0.5) return null;
    var along = ((pb.x - pa.x) * windTo.x + (pb.y - pa.y) * windTo.y) / S;   // yd, + = b downwind of a
    var cross = Math.abs(((pb.x - pa.x) * windTo.y - (pb.y - pa.y) * windTo.x) / S);
    return { lagS: along / (S * YD_PER_S_PER_MPH), alongYd: along, crossYd: cross };
  }
  // A Sentinel has to be upwind of the meter it warns. With known spots, drop a
  // pairing whose "leader" sits downwind (a coincidence of slow swings).
  function upwindOk(lead, follow, windTo) {
    if (!lead || !follow || lead.x == null || follow.x == null) return true;
    var e = expectedLag(lead, follow, windTo);
    return !e || e.alongYd > -10;
  }
  function round6(v) { return Math.abs(v) < 1e-9 ? 0 : +v.toPrecision(6); }
  Relations.prototype.toJSON = function () {
    function pack(o) { var r = {}; for (var k in o) { r[k] = {}; for (var f in o[k]) r[k][f] = o[k][f].map(round6); } return r; }
    var M = {}; for (var k in this.M) M[k] = this.M[k].map(round6);
    return { L: this.L, tau: this.tau, P: pack(this.P), A: pack(this.A), M: M, secs: this.secs, sectorSecs: this.sectorSecs.slice() };
  };

  /*
   * Lines /windsec readings (or simulator seconds) up on whole seconds.
   * add(id, tSec, spd, dir) as readings arrive (any order, repeats fine);
   * release(nowSec, fn) hands every second older than nowSec - settle to
   * fn(tSec, {id: {spd, dir}}) in order, once.
   */
  function SecondAligner(settle) {
    this.settle = settle == null ? 8 : settle;
    this.buckets = {}; this.done = -Infinity;
  }
  SecondAligner.prototype.add = function (id, tSec, spd, dir) {
    var s = Math.round(tSec); if (s <= this.done || !isFinite(spd)) return;
    var B = this.buckets[s] || (this.buckets[s] = {});
    var o = B[id];
    if (!o) B[id] = { spd: spd, dir: dir, n: 1, sx: Math.sin(dir * Math.PI / 180), cy: Math.cos(dir * Math.PI / 180) };
    else { o.n++; o.spd += (spd - o.spd) / o.n; o.sx += Math.sin(dir * Math.PI / 180); o.cy += Math.cos(dir * Math.PI / 180); o.dir = (Math.atan2(o.sx, o.cy) * 180 / Math.PI + 360) % 360; }
  };
  SecondAligner.prototype.release = function (nowSec, fn) {
    var lim = nowSec - this.settle, self = this;
    var ks = Object.keys(this.buckets).map(Number).filter(function (s) { return s <= lim; }).sort(function (a, b) { return a - b; });
    ks.forEach(function (s) {
      var B = self.buckets[s], out = {};
      for (var id in B) out[id] = { spd: B[id].spd, dir: B[id].dir };
      delete self.buckets[s]; self.done = s; fn(s, out);
    });
    return ks.length;
  };

  /*
   * The model Wind Lab runs live, one entry per wind sector:
   *   mult[node]  learned speed multiplier for that wind
   *   wPct[node]  path weight (%) for that wind; 0 = meter not used
   *   sent[]      Sentinel pairs with lag, ratio and tolerances
   * o: {name, meters:[{id, nodeId, lkey, x, y, role}], shooter, target, kernel,
   *     learner, relations, windMph, drive:{mult, weights, sentinels}, azDeg}
   */
  function buildLiveModel(o) {
    var lof = lineOfFire(o.shooter, o.target), S = o.windMph || 10, sectors = [];
    var byId = {}; o.meters.forEach(function (m) { byId[m.id] = m; });
    for (var s = 0; s < SECTORS; s++) {
      var from = s * 45 * Math.PI / 180, wind = { x: -Math.sin(from) * S, y: -Math.cos(from) * S };
      // sentinel-only meters warn; they never take a share of the path
      var geo = geometry(o.meters.filter(function (m) { return m.role !== 'sentinel'; }), o.shooter, o.target, wind, o.geoOpts || {});
      var w = physicsWeights(geo.rows, lof.R, o.kernel);
      var sec = { mult: {}, wPct: {}, sent: [], learned: {} };
      o.meters.forEach(function (m) {
        var mi = o.learner ? o.learner.meter(m.lkey, s) : { mult: 1, sd: 1 };
        var learned = o.learner && o.learner.idx['m:' + m.lkey] != null;
        sec.mult[m.nodeId] = Math.round(Math.max(0.3, Math.min(3, learned ? mi.mult : 1)) * 1000) / 1000;
        sec.wPct[m.nodeId] = Math.round((w[m.id] || 0) * 1000) / 10;
        sec.learned[m.nodeId] = learned ? Math.round(mi.sd * 1000) / 1000 : null;
      });
      if (o.relations) {
        o.relations.suggestSentinels(s, o.relOpts).forEach(function (p) {
          var A = byId[p.sentId], B = byId[p.nodeId]; if (!A || !B || !upwindOk(A, B, wind)) return;
          sec.sent.push({ sentId: A.nodeId, nodeId: B.nodeId, lagS: p.lagS, spdPct: p.spdPct, dirDeg: p.dirDeg, ratio: p.ratio, dirOff: p.dirOff, r: p.r });
        });
      }
      sectors.push(sec);
    }
    var b = o.learner ? o.learner.bias() : { mean: 0, sd: 0 };
    return {
      kind: 'coriolis-live-model', v: 1, name: o.name || 'Learned model', created: new Date().toISOString(),
      rangeYd: Math.round(lof.R), azDeg: o.azDeg == null ? Math.round(lof.bearing) : o.azDeg,
      shots: o.learner ? o.learner.n : 0, relSeconds: o.relations ? o.relations.secs : 0,
      drive: o.drive || { mult: true, weights: true, sentinels: true },
      biasMph: Math.round(b.mean * 100) / 100,
      nodes: o.meters.map(function (m) { return { id: m.nodeId, meter: m.id }; }),
      sectors: sectors
    };
  }

  root.WindLearner = {
    YD_PER_S_PER_MPH: YD_PER_S_PER_MPH, SECTORS: SECTORS, SECTOR_NAMES: SECTOR_NAMES,
    lineOfFire: lineOfFire, components: components, fromDeg: fromDeg, sectorOf: sectorOf,
    geometry: geometry, physicsWeights: physicsWeights, Learner: Learner,
    Relations: Relations, relationKind: relationKind, expectedLag: expectedLag, upwindOk: upwindOk,
    SecondAligner: SecondAligner, buildLiveModel: buildLiveModel
  };
})(typeof window !== 'undefined' ? window : globalThis);
