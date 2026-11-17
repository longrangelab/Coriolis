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
    this.adapt = cfg.adapt !== false;
    this.keys = []; this.idx = {}; this.mean = []; this.cov = []; this.n = 0;
    this.anchor = {};
    // change watch: running mean of the normalised shot surprises
    this.zm = 0; this.since = 0; this.events = [];
    // learned noise scale: how big the shot surprises really are against
    // what the model said (1 = honest); widens or narrows the stated doubt
    this.k2 = 1;
    if (state) {
      this.zm = state.zm || 0; this.k2 = state.k2 || 1; this.since = state.since || 0; this.events = (state.events || []).slice();
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
    var z = innov / Math.sqrt(S), zs = z / Math.sqrt(this.k2);
    this.k2 += Math.max(0.04, 1 / (this.n + 4)) * (Math.min(16, z * z) - this.k2);
    this.k2 = Math.max(0.3, Math.min(9, this.k2));
    var changed = this.adapt ? this._watch(zs) : false;
    return { innovation: innov, predicted: pred, z: z, changed: changed };
  };
  /*
   * Conditions change (a meter gets knocked, the wind swings onto a new
   * terrain feature, the zero shifts). The shots then keep landing off to
   * the same side of the model's call. When the running mean of the
   * normalised surprises (about the last 5 shots) passes 1 (3x its usual
   * spread), the model widens its doubt on every multiplier and the offset
   * so the next shots move it quickly, instead of slowly unlearning.
   */
  Learner.prototype._watch = function (z) {
    this.since++;
    this.zm += 0.2 * (Math.max(-4, Math.min(4, z)) - this.zm);
    if (this.since < 8 || Math.abs(this.zm) < 1) return false;
    for (var i = 0; i < this.keys.length; i++) {
      var k = this.keys[i], sd = k === 'b' ? this.priorBsd : k.charAt(0) === 'm' ? this.priorMsd : this.priorDsd;
      this.cov[i][i] += 0.5 * sd * sd;
    }
    this.events.push({ n: this.n, zm: Math.round(this.zm * 100) / 100 });
    if (this.events.length > 20) this.events.shift();
    this.zm = 0; this.since = 0;
    return true;
  };
  // real shot surprises over stated ones (1 SD ratio); > 1 = the model's
  // own doubt has been too small and the page widens it
  Learner.prototype.noiseScale = function () { return Math.sqrt(this.k2); };
  // the call's doubt from the model alone, for a set of features (mph, 1 SD)
  Learner.prototype.doubt = function (feats, sector) { return this.predict(feats, sector).sd; };
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
    return { keys: this.keys, mean: this.mean, cov: this.cov, n: this.n, anchor: this.anchor, zm: this.zm, since: this.since, events: this.events, k2: this.k2 };
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
  /*
   * Pairs are learned by DISTANCE along the wind, not by seconds: a gust
   * that reaches meter b 60 yd after meter a does so in 12 s at 10 mph and
   * 6 s at 20 mph. Each second, bin j compares b now with a as it was
   * j * 6 yd of travel ago at the field's running wind speed (seconds
   * interpolated). So every speed adds to the same peak, and the lag at
   * any live speed is alongYd / speed. Out to 480 yd (98 s at 10 mph),
   * as far as the 200 s of history reaches.
   */
  var REL_BIN = 6, REL_D = 480, REL_RING = 200;
  function Relations(state, cfg) {
    cfg = cfg || {};
    if (state && state.kind !== 'dist') state = null;   // older by-seconds data: start over
    this.L = 40;                                         // own-gust persistence, seconds (for n_eff)
    this.NB = Math.round(((state && state.D) || cfg.maxYd || REL_D) / REL_BIN);
    this.tau = (state && state.tau) || cfg.tau || 60;
    // s = sector 0..7 or 'all'
    this.P = (state && state.P) || {};     // 'a|b|s' -> {n,ab,aa,bb,ds,dc} arrays, index j+NB (j>0: a leads b by j*BIN yd)
    this.A = (state && state.A) || {};     // 'id|s'  -> {n,c} arrays, autocovariance lags 0..L s
    this.M = (state && state.M) || {};     // 'id|s'  -> [n, sum speed]
    this.U = (state && state.U) || {};     // s -> [n, sum field speed mph] (turns yards back into seconds)
    this.secs = (state && state.secs) || 0;
    this.sectorSecs = (state && state.sectorSecs) || new Array(SECTORS).fill(0);
    this.rt = {};                          // per meter: {ema, ring} (not saved)
    this.lastT = null; this.fu = null;
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
      var gap = Math.min(REL_RING, Math.round(t - this.lastT) - 1);
      for (var g = 0; g < gap; g++) for (id in this.rt) { this.rt[id].ring.unshift(null); this.rt[id].ring.length = Math.min(this.rt[id].ring.length, REL_RING); }
      if (t - this.lastT > 600) for (id in this.rt) { this.rt[id].ema = null; this.rt[id].ring = []; }   // long break: start the running means again
    }
    this.lastT = t;
    // field wind for this second picks the sector
    var sx = 0, sy = 0, cnt = 0, ss = 0, ids = Object.keys(readings);
    ids.forEach(function (k) {
      var r = readings[k]; if (!r || !isFinite(r.spd) || !isFinite(r.dir)) return;
      var d = r.dir * Math.PI / 180; sx += r.spd * Math.sin(d); sy += r.spd * Math.cos(d); cnt++; ss += r.spd;
    });
    // the speed gusts travel at: the field's running mean speed (2 min; a
    // gust rides the bigger air mass, it doesn't race ahead at its own speed)
    if (cnt) { this.fk = (this.fk || 0) + 1; this.fu = this.fu == null ? ss / cnt : this.fu + (ss / cnt - this.fu) / Math.min(this.fk, 120); }
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
      if (R.ring.length > REL_RING) R.ring.length = REL_RING;
      present[k] = true;
    });
    for (id in present) if (!present[id]) { var R0 = this.rt[id]; R0.ring.unshift(null); if (R0.ring.length > REL_RING) R0.ring.length = REL_RING; }
    if (!sectors.length) return;
    this.secs++;
    this._accumulate('all', present);
    for (var si = 0; si < sectors.length; si++) this._accumulate(sectors[si], present);
  };
  Relations.prototype._accumulate = function (sector, present) {
    var self = this, L = this.L, NB = this.NB, ups = Math.max(0.5, this.fu || 0) * YD_PER_S_PER_MPH;
    if (sector !== 'all') this.sectorSecs[sector]++;
    var Us = this.U[sector] || (this.U[sector] = [0, 0]); Us[0]++; Us[1] += this.fu || 0;
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
      var key2 = a + '|' + b + '|' + sector, W = 2 * NB + 1;
      var P = this.P[key2] || (this.P[key2] = { n: zeros(W), ab: zeros(W), aa: zeros(W), bb: zeros(W), ds: zeros(W), dc: zeros(W) });
      for (var jb = 0; jb <= NB; jb++) {
        var kf = jb * REL_BIN / ups; if (kf > REL_RING - 2) break;
        // +j: a as it was j bins of travel ago, with b now (a leads b)
        if (Rb[0]) { var xa = ringAt(Ra, kf); if (xa) this._add(P, NB + jb, xa, Rb[0]); }
        // -j: b as it was, with a now (b leads a)
        if (jb > 0 && Ra[0]) { var xb = ringAt(Rb, kf); if (xb) this._add(P, NB - jb, Ra[0], xb); }
      }
    }
  };
  // a meter's reading kf seconds ago, between whole seconds by straight line
  function ringAt(R, kf) {
    var k0 = Math.floor(kf), w = kf - k0, o0 = R[k0], o1 = R[k0 + 1];
    if (w < 1e-6) return o0 || null;
    if (!o0 || !o1) return (w < 0.5 ? o0 : o1) || null;
    return { f: o0.f + w * (o1.f - o0.f), spd: o0.spd + w * (o1.spd - o0.spd), dir: w < 0.5 ? o0.dir : o1.dir };
  }
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
    var pk = pairKey(a, b), flip = pk[0] !== String(a), P = this._P(pk[0], pk[1], sector), NB = this.NB, out = [];
    if (!P) return out;
    var ups = Math.max(0.5, this.speed(sector) || 10) * YD_PER_S_PER_MPH;
    for (var i = 0; i <= 2 * NB; i++) {
      var n = P.n[i], d = Math.sqrt(P.aa[i] * P.bb[i]), yd = (flip ? NB - i : i - NB) * REL_BIN;
      out.push({ lag: Math.round(yd / ups * 10) / 10, yd: yd, r: n && d > 0 ? P.ab[i] / d : 0, n: n });
    }
    if (flip) out.reverse();
    return out;
  };
  /*
   * One pair, one sector (or 'all'). Returns null until there is enough data.
   * lead/follow name the meters so that lead sees the gusts lagS seconds first.
   */
  // the sector's mean wind speed while it learned (mph), to turn yards into seconds
  Relations.prototype.speed = function (sector) { var u = this.U[sector]; return u && u[0] ? u[1] / u[0] : null; };
  Relations.prototype.pair = function (a, b, sector, opts) {
    opts = opts || {};
    var minN = opts.minN || 120, NB = this.NB, L = this.L;
    var pk = pairKey(a, b); a = pk[0]; b = pk[1];
    var P = this._P(a, b, sector); if (!P) return null;
    var best = -1, bestR = -2, rAt = function (i) { var d = Math.sqrt(P.aa[i] * P.bb[i]); return P.n[i] && d > 0 ? P.ab[i] / d : 0; };
    for (var i = 0; i <= 2 * NB; i++) {
      if (P.n[i] < minN) continue;
      var r = rAt(i);
      // ties and near-ties go to the shorter distance (simplest explanation)
      if (r > bestR + 0.005 || (Math.abs(r - bestR) <= 0.005 && Math.abs(i - NB) < Math.abs(best - NB))) { bestR = r; best = i; }
    }
    if (best < 0) return null;
    // the peak between bins: a parabola through the best bin and its neighbours
    var dj = 0;
    if (best > 0 && best < 2 * NB && P.n[best - 1] >= minN && P.n[best + 1] >= minN) {
      var r0 = rAt(best - 1), r2 = rAt(best + 1), den = r0 - 2 * bestR + r2;
      if (den < -1e-9) dj = Math.max(-0.5, Math.min(0.5, 0.5 * (r0 - r2) / den));
    }
    var yd = (best - NB + dj) * REL_BIN, mph = Math.max(0.5, this.speed(sector) || 10);
    var k = best - NB, lead = k >= 0 ? a : b, follow = k >= 0 ? b : a, along = Math.abs(yd);
    var lagF = along / (mph * YD_PER_S_PER_MPH), lag = Math.round(lagF);
    var n = P.n[best];
    // effective sample size (Bartlett): neighbouring seconds are not independent
    var T = 1;
    for (var j = 1; j <= L; j++) T += 2 * this.autoCorr(a, sector, j) * this.autoCorr(b, sector, j);
    var neff = Math.max(4, n / Math.max(1, T)), se = 1 / Math.sqrt(Math.max(1, neff - 3));
    var rPers = this.autoCorr(follow, sector, Math.min(L, lag));
    var varLead = k >= 0 ? P.aa[best] : P.bb[best], varFol = k >= 0 ? P.bb[best] : P.aa[best];
    var gain = varLead > 0 ? P.ab[best] / varLead : 0;
    var sdFol = Math.sqrt(varFol / n);
    var ds = P.ds[best], dc = P.dc[best], Rl = Math.min(1, Math.hypot(ds, dc) / n);
    var dirOff = Math.atan2(ds, dc) * 180 / Math.PI;               // b minus a
    if (k < 0) dirOff = -dirOff;                                     // follow minus lead
    var mLead = this._mean(lead, sector), mFol = this._mean(follow, sector);
    return {
      a: a, b: b, lead: lead, follow: follow, lagS: lag, lagF: lagF, alongYd: Math.round(along), atMph: mph, r: bestR, n: n, neff: neff, se: se,
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
        if (!cur || p.skill > cur.skill) best[p.follow] = { by: p.lead, lagS: p.lagS, alongYd: p.alongYd, r: p.r, skill: p.skill, ratio: p.ratio };
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
        out.push({ sentId: p.lead, nodeId: p.follow, lagS: p.lagS, alongYd: p.alongYd, atMph: Math.round(p.atMph * 10) / 10, spdPct: spdPct, dirDeg: dirDeg,
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
  /*
   * Sentinel pairs from the layout alone, for day one before any wind data:
   * for each path meter, the meter whose air reaches it from upwind, close
   * to straight down the wind (cross offset <= 30 yd or 30 % of the run),
   * at least 3 s ahead and up to 480 yd upwind, preferring about 15 s. Tolerances are wide (30 %, 20
   * deg) because nothing has confirmed the pair yet. meters: [{id, x, y,
   * role}], windTo: air motion (mph). Returns one pair per meter at most.
   */
  function geometryPairs(meters, windTo, opts) {
    opts = opts || {};
    var S = Math.hypot(windTo.x, windTo.y), out = [];
    if (S < 1) return out;
    meters.forEach(function (B) {
      if (B.role === 'sentinel' || B.x == null) return;
      var best = null;
      meters.forEach(function (A) {
        if (A === B || A.x == null) return;
        var e = expectedLag(A, B, windTo); if (!e || e.alongYd < 20) return;
        if (e.lagS < (opts.minLag || 3) || (opts.maxLag ? e.lagS > opts.maxLag : e.alongYd > (opts.maxYd || REL_D))) return;   // out to 480 yd, as far as the wind data looks
        if (e.crossYd > Math.max(30, 0.3 * e.alongYd)) return;
        var score = e.crossYd / e.alongYd + 0.3 * Math.abs(Math.log(e.lagS / 15));
        if (!best || score < best.score) best = { score: score, A: A, e: e };
      });
      if (best) out.push({ sentId: best.A.id, nodeId: B.id, lagS: Math.round(best.e.lagS), alongYd: Math.round(best.e.alongYd), crossYd: Math.round(best.e.crossYd),
                           spdPct: 30, dirDeg: 20, ratio: 1, dirOff: 0, src: 'layout' });
    });
    return out;
  }
  function round4(v) { return Math.abs(v) < 1e-12 ? 0 : +v.toPrecision(4); }
  function round6(v) { return Math.abs(v) < 1e-9 ? 0 : +v.toPrecision(6); }
  Relations.prototype.toJSON = function () {
    // 5 significant figures: plenty for correlations, and a third smaller saved
    function r5(v) { return Math.abs(v) < 1e-9 ? 0 : +v.toPrecision(5); }
    function pack(o) { var r = {}; for (var k in o) { r[k] = {}; for (var f in o[k]) r[k][f] = o[k][f].map(r5); } return r; }
    var M = {}; for (var k in this.M) M[k] = this.M[k].map(round6);
    var U = {}; for (var u in this.U) U[u] = this.U[u].map(round6);
    return { kind: 'dist', D: this.NB * REL_BIN, tau: this.tau, P: pack(this.P), A: pack(this.A), M: M, U: U, secs: this.secs, sectorSecs: this.sectorSecs.slice() };
  };

  // ------------------------------------------------------------- forecaster
  /*
   * Short-range wind forecast for every meter, trained on the meters alone:
   * each second that arrives is the answer to the forecasts made 2, 5, 10
   * and 20 s earlier, so it learns all day without a single shot.
   *
   * Per meter, horizon h and wind component (east u, north v of the air's
   * motion) a small recursive-least-squares model with forgetting:
   *   u(t+h) = w0 + w1 u(t) + w2 mean60(t) + w3 (u(t) - u(t-5)) + w4 adv
   * adv = the air the meter's best early warning (from Relations) saw that
   * reaches this meter at t+h: the leader's reading from (lag - h) s before t
   * (or its newest one when the lag is shorter than h), times the speed ratio.
   * It starts as "the wind stays as it is" (w1 = 1) and moves off that only
   * as far as the data earns. Each model also keeps its forecast error and
   * the error of "stays as it is", so it reports its own skill.
   *
   * add(id, tSec, mph, fromDeg) with every timed 1 s reading as it arrives.
   * predict(id, tSec) -> {x, y (mph, air moves-to), sd, h, n, skill}.
   */
  function Forecaster(state, cfg) {
    cfg = cfg || {}; state = state || null;
    this.H = (state && state.H) || cfg.horizons || [2, 5, 10, 20];
    this.lam = (state && state.lam) || cfg.lambda || 0.998;      // memory ~ 500 s of readings
    this.M = (state && state.M) || {};                          // 'id|h|u' -> {w, P, n, e2, p2}
    this.rt = {};                                               // per meter: {s: {sec: {u,v,mu,mv}}, k, last}
    this.leaderFn = cfg.leaderFn || null;                       // id -> {by, lagS, ratio} or null
    this.useLeader = cfg.useLeader !== false;
  }
  function eye(d) { return d.map(function (v, i) { var r = new Array(d.length).fill(0); r[i] = v; return r; }); }
  Forecaster.prototype._model = function (key) {
    return this.M[key] || (this.M[key] = { w: [0, 1, 0, 0, 0], P: eye([1, 0.01, 0.01, 0.01, 0.01]), n: 0, e2: 0, p2: 0 });
  };
  Forecaster.prototype._at = function (id, sec) { var R = this.rt[id]; return R ? R.s[sec] || null : null; };
  Forecaster.prototype._feats = function (id, s0, h) {
    var cur = this._at(id, s0); if (!cur) return null;
    var prev = this._at(id, s0 - 5) || this._at(id, s0 - 4) || this._at(id, s0 - 6) || cur;
    var adv = { u: cur.u, v: cur.v };
    var L = this.useLeader && this.leaderFn ? this.leaderFn(id) : null;
    if (L && L.by !== id) {
      var sl = Math.min(s0, s0 + h - Math.round(L.lagS)), lv = this._at(L.by, sl) || this._at(L.by, sl - 1) || this._at(L.by, sl + 1);
      var ra = L.ratio > 0 ? L.ratio : 1;
      if (lv) adv = { u: lv.u * ra, v: lv.v * ra };
    }
    return {
      u: [1, cur.u, cur.mu, cur.u - prev.u, adv.u],
      v: [1, cur.v, cur.mv, cur.v - prev.v, adv.v],
      cur: cur
    };
  };
  function rlsStep(m, x, y, lam) {
    var n = x.length, Px = new Array(n), d = lam, i, j, e = y;
    for (i = 0; i < n; i++) { e -= m.w[i] * x[i]; var s = 0; for (j = 0; j < n; j++) s += m.P[i][j] * x[j]; Px[i] = s; d += x[i] * s; }
    for (i = 0; i < n; i++) m.w[i] += Px[i] * e / d;
    var tr = 0;
    for (i = 0; i < n; i++) { for (j = 0; j < n; j++) m.P[i][j] = (m.P[i][j] - Px[i] * Px[j] / d) / lam; tr += m.P[i][i]; }
    if (tr > 20) for (i = 0; i < n; i++) for (j = 0; j < n; j++) m.P[i][j] *= 20 / tr;   // calm spells must not blow it up
    return e;
  }
  Forecaster.prototype.add = function (id, t, mph, dir) {
    var sec = Math.round(t); id = String(id);
    if (!isFinite(mph) || !isFinite(dir)) return;
    var R = this.rt[id] || (this.rt[id] = { s: {}, k: 0, mu: null, mv: null, last: -Infinity });
    if (R.s[sec]) return;
    var d = dir * Math.PI / 180, u = -Math.sin(d) * mph, v = -Math.cos(d) * mph;
    if (R.mu == null) { R.mu = u; R.mv = v; }
    R.k++; var a = 1 / Math.min(R.k, 60);
    R.mu += a * (u - R.mu); R.mv += a * (v - R.mv);
    R.s[sec] = { u: u, v: v, mu: R.mu, mv: R.mv };
    if (sec > R.last) R.last = sec;
    for (var k in R.s) if (+k < R.last - 90) delete R.s[k];
    // this reading answers the forecasts made h seconds ago
    for (var i = 0; i < this.H.length; i++) {
      var h = this.H[i], F = this._feats(id, sec - h, h); if (!F) continue;
      var tgt = { u: u, v: v };
      for (var c in tgt) {
        var m = this._model(id + '|' + h + '|' + c);
        var e = rlsStep(m, F[c], tgt[c], this.lam), p = tgt[c] - F.cur[c];
        m.n++; var al = Math.max(1 / m.n, 1 / 400);
        m.e2 += al * (e * e - m.e2); m.p2 += al * (p * p - m.p2);
      }
    }
  };
  Forecaster.prototype.latest = function (id) { var R = this.rt[String(id)]; return R && isFinite(R.last) ? R.last : null; };
  Forecaster.prototype._one = function (id, s0, h) {
    var F = this._feats(id, s0, h); if (!F) return null;
    var mu = this.M[id + '|' + h + '|u'], mv = this.M[id + '|' + h + '|v'];
    function dot(m, x) { var r = 0; for (var i = 0; i < x.length; i++) r += m.w[i] * x[i]; return r; }
    var ready = mu && mv && mu.n >= 60;
    var x = ready ? dot(mu, F.u) : F.cur.u, y = ready ? dot(mv, F.v) : F.cur.v;
    var e2 = ready ? (mu.e2 + mv.e2) / 2 : (mu && mu.n >= 10 ? (mu.p2 + mv.p2) / 2 : null);
    var p2 = mu && mu.n >= 10 ? (mu.p2 + mv.p2) / 2 : null;
    return { x: x, y: y, sd: e2 == null ? null : Math.sqrt(e2), sdStay: p2 == null ? null : Math.sqrt(p2), n: mu ? mu.n : 0, ready: !!ready };
  };
  // forecast for meter id at time t (s), from its newest reading
  Forecaster.prototype.predict = function (id, t) {
    id = String(id); var s0 = this.latest(id); if (s0 == null) return null;
    var h = t - s0, H = this.H;
    if (h <= 0.5) { var c = this._at(id, Math.min(s0, Math.round(t))) || this._at(id, s0); return { x: c.u, y: c.v, sd: 0, h: 0, n: 0, ready: true }; }
    var lo = { h: 0, f: { x: this._at(id, s0).u, y: this._at(id, s0).v, sd: 0, ready: true, n: 0 } }, hi = null;
    for (var i = 0; i < H.length; i++) {
      var f = this._one(id, s0, H[i]); if (!f) break;
      if (H[i] <= h) lo = { h: H[i], f: f }; else { hi = { h: H[i], f: f }; break; }
    }
    if (!hi) { var g = lo.f; return { x: g.x, y: g.y, sd: g.sd, h: h, n: g.n, ready: g.ready }; }
    var a = (h - lo.h) / (hi.h - lo.h), A = lo.f, B = hi.f;
    var sd = A.sd == null || B.sd == null ? null : A.sd + a * (B.sd - A.sd);
    return { x: A.x + a * (B.x - A.x), y: A.y + a * (B.y - A.y), sd: sd, h: h, n: B.n, ready: A.ready && B.ready };
  };
  // how good each meter's forecasts are: rmse vs "the wind stays as it is"
  Forecaster.prototype.stats = function (id) {
    var self = this; id = String(id);
    return this.H.map(function (h) {
      var mu = self.M[id + '|' + h + '|u'], mv = self.M[id + '|' + h + '|v'];
      if (!mu || !mv || !mu.n) return { h: h, n: 0 };
      var e = Math.sqrt((mu.e2 + mv.e2) / 2), p = Math.sqrt((mu.p2 + mv.p2) / 2);
      return { h: h, n: mu.n, rmse: e, rmseStay: p, skill: p > 0 ? 1 - e / p : 0, wLeader: (mu.w[4] + mv.w[4]) / 2, wNow: (mu.w[1] + mv.w[1]) / 2 };
    });
  };
  Forecaster.prototype.meters = function () { return Object.keys(this.rt).sort(); };
  Forecaster.prototype.toJSON = function () {
    var M = {};
    for (var k in this.M) { var m = this.M[k]; M[k] = { w: m.w.map(round6), P: m.P.map(function (r) { return r.map(round6); }), n: m.n, e2: round6(m.e2), p2: round6(m.p2) }; }
    return { H: this.H, lam: this.lam, M: M };
  };

  /*
   * ForecastLoop: the self-improving wind prediction loop. No shots needed.
   *
   * Every second, for every meter and horizon (2, 5, 10, 20 s) each model
   * in that meter's pool makes a prediction and it is written down. When
   * that second's wind arrives the prediction is checked, the error goes on
   * the model's score card, and then every model learns from the answer.
   * The page uses a blend of the best few models (weighted by their recent
   * error), and the blend's own predictions are checked the same way, so the
   * skill it reports is earned on wind it had not seen yet.
   *
   * Every generation (90 s of wind per meter) the pool evolves: the worst
   * grown-up model is retired and a new one is tried. New models come from
   * the Relations tab's early-warning pairs, from small changes to the
   * current best (shift a Sentinel's lag, add or drop a Sentinel, change how
   * fast it forgets, swap the linear model for a small neural net or back)
   * and now and then from a blind guess (any other meter, any lag), so it
   * finds Sentinel/node pairs by trying them, keeping only what predicts.
   *
   * Model kinds: 'stay' (the wind stays as it is: the bar to beat), 'rls'
   * (recursive least squares with forgetting on [1, now, 60 s mean, 5 s
   * trend, Sentinel readings]), 'mlp' (a small tanh network on the same
   * inputs that predicts the change from now, trained by SGD).
   *
   * Other sensors: add(id, t, mph, dir, {light: .., pressure: .., temp: ..})
   * for channels a meter carries, addEnv(t, {...}) for ones at the base.
   * Any channel name works. Each one becomes three inputs (how far from its
   * usual value, its change over a minute, and that times the wind now) and
   * the loop tries models with and without it, so a sensor earns its place
   * the same way a Sentinel does. Nothing changes until a channel shows up.
   *
   * Same calls as Forecaster: add(id, t, mph, dir), predict(id, t),
   * latest(id), stats(id), meters(), toJSON(); plus pool(id), history(id),
   * log(id), trace(id), provenPairs().
   */
  var LOOP_H = [2, 5, 10, 20];
  // A neural net learns by small steps and needs 30-60 min of wind to catch
  // up with a linear model it may later beat, so it can't be retired for its
  // first 20 generations (30 min) unless it does worse than "stays as it
  // is" (it starts out as exactly that: its output weights start at zero). Only big pools (the meters that matter most) carry
  // young nets: two at a time in a pool of 10, one in 7-9, none below, so
  // they don't crowd out the search for Sentinels and sensors.
  var MLP_GRACE = 20;
  function ForecastLoop(state, cfg) {
    cfg = cfg || {}; state = state || {};
    this.H = state.H || LOOP_H;
    this.maxC = cfg.maxCandidates || state.maxC || 8;
    this.genS = cfg.generationS || state.genS || 90;
    this.evolve = cfg.evolve !== false;
    this.leadersFn = cfg.leadersFn || null;        // id -> [{by, lagS}] suggested pairs (may be empty)
    this.importanceFn = cfg.importanceFn || null;  // id -> share of the shot 0..1 (null: every meter the same)
    var seed = cfg.seed || state.seed || 12345;
    this.rnd = (function (s) { return function () { s |= 0; s = s + 0x6D2B79F5 | 0; var t = Math.imul(s ^ s >>> 15, 1 | s); t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t; return ((t ^ t >>> 14) >>> 0) / 4294967296; }; })(seed);
    this.seed = seed;
    this.rt = {};                                  // per meter ring of seconds
    this.env = { s: {}, cs: {} };                  // base-level channels by second
    this.chan = {};                                // channel names seen so far
    this.P = {};                                   // per meter pool, for the wind blowing now
    // memory by wind: every direction (8 x 45 deg) and speed band (3-8,
    // 8-15, 15+ mph, plus calm) keeps its own pools. When the wind turns,
    // the pools for the wind it left are put away (best few models each)
    // and the ones for the new wind come back, learned weights and all.
    this.regimes = cfg.regimes != null ? cfg.regimes !== false : state.regimes !== false;
    this.noSpeedBands = cfg.speedBands != null ? cfg.speedBands === false : !!state.noSpeedBands;
    this.cutN = cfg.cutN || 1e9;   // scores carry over a switch as they are (tested: cutting them back did worse)
    this.reg = { cur: state.regCur || null, cand: null, candN: 0, last: -Infinity };
    this.A = {};                                   // regime -> meter -> pool put away
    this.regSecs = state.regSecs || {};            // regime -> seconds of wind seen
    this.RB = state.RB || {};                      // regime -> meter -> horizon -> blend error record
    var self = this;
    (state.channels || []).forEach(function (k) { self.chan[k] = true; });
    if (state.P) Object.keys(state.P).forEach(function (id) { self.P[id] = self._restorePool(state.P[id]); });
    if (state.A) Object.keys(state.A).forEach(function (rk) { self.A[rk] = {}; Object.keys(state.A[rk]).forEach(function (id) { self.A[rk][id] = state.A[rk][id]; }); });
  }
  // the wind regime for a field wind (mph, from-degrees)
  function regimeKey(mph, from, noSpeed) {
    if (!(mph >= 3)) return 'calm';
    return (Math.round(((from % 360) + 360) % 360 / 45) % 8) + '/' + (noSpeed ? 'a' : mph < 8 ? 0 : mph < 15 ? 1 : 2);
  }
  // still inside regime k, with margins: 1 mph past a speed edge and 10 deg
  // past a direction edge before it counts as a different wind
  function regimeHolds(k, mph, from) {
    if (k === 'calm') return mph < 4;
    var q = String(k).split('/'), sct = +q[0], b = +q[1];
    if (mph < 2) return false;
    var lo = q[1] === 'a' ? 2 : [2, 7, 14][b], hi = q[1] === 'a' ? Infinity : [9, 16, Infinity][b];
    var off = Math.abs(((from - sct * 45) % 360 + 540) % 360 - 180);
    return mph >= lo && mph < hi && off <= 32.5;
  }
  var REG_DIR = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'], REG_SPD = ['3-8 mph', '8-15 mph', '15+ mph'];
  function regimeName(k) { if (k === 'calm') return 'Calm (under 3 mph)'; var q = String(k).split('/'); return 'From ' + REG_DIR[+q[0]] + (q[1] === 'a' ? '' : ', ' + REG_SPD[+q[1]]); }
  // once a second: which wind is this? A new one has to hold for 60 s
  // before the pools change over, so a passing swing doesn't churn them
  ForecastLoop.prototype._regTick = function (sec) {
    var G = this.reg; if (sec <= G.last) return; G.last = sec;
    var sx = 0, sy = 0, n = 0;
    for (var id in this.rt) { var R = this.rt[id]; if (R.last >= sec - 10 && R.mu != null) { sx += R.mu; sy += R.mv; n++; } }
    if (!n) return;
    var u = sx / n, v = sy / n, mph = Math.hypot(u, v), from = (Math.atan2(-u, -v) * 180 / Math.PI + 360) % 360;
    var k = regimeKey(mph, from, this.noSpeedBands);
    if (G.cur == null) G.cur = k;
    if (k !== G.cur && regimeHolds(G.cur, mph, from)) k = G.cur;   // near an edge: stay put
    this.regSecs[G.cur] = (this.regSecs[G.cur] || 0) + 1;
    if (k === G.cur) { G.cand = null; G.candN = 0; return; }
    if (k !== G.cand) { G.cand = k; G.candN = 0; }
    if (++G.candN >= 60) { this._switch(k, sec); G.cand = null; G.candN = 0; }
  };
  // The wind turned: what it learned for the old wind is put away, and what
  // it learned before for the new wind joins the pool, weights and all, to
  // compete with the models running now. Nothing is thrown out at the
  // switch; the next generations keep whichever do best in this wind. Every
  // score is cut back to 30 checks, so this wind's own record soon decides.
  ForecastLoop.prototype._switch = function (k, sec) {
    var old = this.reg.cur, me = this, keep = this.A[old] || (this.A[old] = {});
    Object.keys(this.P).forEach(function (id) {
      var p = me.P[id];
      keep[id] = me._packPool(p, true);
      p.list.forEach(function (c) { for (var h in c.S) c.S[h].n = Math.min(c.S[h].n, me.cutN); });
      var back = me.A[k] && me.A[k][id], n = 0;
      if (back) {
        var bp = me._restorePool(back), sigs = p.list.map(lsig);
        bp.list.forEach(function (c) {
          if (c.kind === 'stay' || isBase(c) || sigs.indexOf(lsig(c)) >= 0) return;
          c.id = p.next++; c.born = p.g; c.origin = 'remembered'; c.from = null;
          for (var h in c.S) c.S[h].n = Math.min(c.S[h].n, me.cutN);
          p.list.push(c); n++;
        });
        delete me.A[k][id];
      }
      me._log(p, sec, 'Wind now ' + regimeName(k) + (n ? ': brought back ' + n + ' model' + (n > 1 ? 's' : '') + ' it learned in this wind before' : ': new to this wind'));
    });
    if (this.A[k] && !Object.keys(this.A[k]).length) delete this.A[k];
    this.reg.cur = k;
  };
  ForecastLoop.prototype._log = function (p, sec, txt) { p.log.push({ g: p.g, t: sec, txt: txt }); if (p.log.length > 80) p.log.shift(); };
  // what it knows, wind by wind: [{key, name, now, seconds, meters: {id: {skill10, champ, pairs}}}]
  ForecastLoop.prototype.regimeList = function () {
    var me = this, out = {}, keys = Object.keys(this.regSecs);
    if (this.reg.cur && keys.indexOf(this.reg.cur) < 0) keys.push(this.reg.cur);
    keys.forEach(function (k) {
      var pools = k === me.reg.cur ? me.P : me.A[k] || {}, m = {};
      Object.keys(pools).forEach(function (id) {
        var p = pools[id] === me.P[id] && k === me.reg.cur ? me.P[id] : me._restorePool(pools[id]);
        var B = ((me.RB[k] || {})[id] || {})[10], champ = p.list.filter(function (c) { return c.id === p.champ; })[0];
        m[id] = { skill10: B && B.n >= 60 && B.p2 > 0 ? 1 - Math.sqrt(B.e2 / B.p2) : null, checked: p.checked, generation: p.g, champ: champ ? cdesc(champ) : null,
          leaders: champ ? champ.leaders.slice() : [], inputs: champ ? (champ.inputs || []).slice() : [] };
      });
      out[k] = { key: k, name: regimeName(k), now: k === me.reg.cur, seconds: me.regSecs[k] || 0, meters: m };
    });
    return Object.keys(out).map(function (k) { return out[k]; }).sort(function (a, b) { return b.now - a.now || b.seconds - a.seconds; });
  };
  ForecastLoop.prototype.regime = function () { return this.reg.cur; };
  ForecastLoop.prototype._newPool = function () {
    var p = { list: [], next: 1, g: 0, k: 0, checked: 0, B: {}, W: {}, hist: [], log: [], champ: null, pend: {}, trace: [] };
    this.H.forEach(function (h) { p.B[h] = { e2: 0, p2: 0, n: 0 }; p.W[h] = { se: 0, sp: 0, n: 0 }; });
    var me = this;
    [{ kind: 'stay' }, { kind: 'rls', lam: 0.998 }, { kind: 'rls', lam: 0.99 }, { kind: 'mlp', hidden: 6, lr: 0.03 }]
      .forEach(function (c) { me._addCand(p, c, 'start'); });
    return p;
  };
  ForecastLoop.prototype._pool = function (id) { return this.P[id] || (this.P[id] = this._newPool()); };
  // the starting linear model on the meter's own history is never retired:
  // it is the fair yardstick a Sentinel or a sensor has to beat
  function isBase(c) { return c.origin === 'start' && c.kind === 'rls' && c.lam === 0.998 && !c.leaders.length && !(c.inputs || []).length; }
  function lsig(c) { return c.kind + '|' + (c.lam || '') + '|' + (c.hidden || '') + '|' + (c.leaders || []).map(function (l) { return l.by + '@' + l.lag; }).sort().join(',') + '|' + (c.inputs || []).map(function (q) { return q.k + '@' + q.lag; }).sort().join(','); }
  // names of a model's inputs, so a child can start from its parent's weights
  function fnames(c) {
    var n = ['1', 'cur', 'mean', 'trend'];
    c.leaders.forEach(function (l) { n.push('L:' + l.by); });
    (c.inputs || []).forEach(function (q) { n.push('C:' + q.k + ':z', 'C:' + q.k + ':d', 'C:' + q.k + ':x'); });
    return n;
  }
  function warmStart(c, par) {
    if (!par || par.kind !== c.kind || c.kind === 'stay') return;
    var nc = fnames(c), np = fnames(par), map = nc.map(function (k) { return np.indexOf(k); });
    for (var h in c.M) for (var comp in c.M[h]) {
      var A = c.M[h][comp], B = par.M[h] && par.M[h][comp]; if (!B) continue;
      if (c.kind === 'rls') map.forEach(function (j, i) { if (j >= 0) A.w[i] = B.w[j]; });
      else if (c.hidden === par.hidden) {
        A.b1 = B.b1.slice(); A.W2 = B.W2.slice(); A.b2 = B.b2;
        A.W1 = B.W1.map(function (row, r) { return map.slice(1).map(function (j, i) { return j >= 1 ? row[j - 1] : A.W1[r][i]; }); });
      }
    }
  }
  ForecastLoop.prototype._addCand = function (p, spec, origin) {
    var c = { id: p.next++, kind: spec.kind, lam: spec.lam || 0.998, hidden: spec.hidden || 6, lr: spec.lr || 0.03,
      leaders: (spec.leaders || []).map(function (l) { return { by: String(l.by), lag: Math.max(1, Math.round(l.lag)) }; })
        .filter(function (l, i, A) { return A.findIndex(function (o) { return o.by === l.by && Math.abs(o.lag - l.lag) < 2; }) === i; }),
      inputs: (spec.inputs || []).map(function (q) { return typeof q === 'string' ? { k: q, lag: 0 } : { k: String(q.k), lag: Math.max(0, Math.round(q.lag || 0)) }; })
        .filter(function (q, i, A) { return A.findIndex(function (o) { return o.k === q.k; }) === i; }),
      born: p.g, origin: origin, from: spec.from || null, M: {}, S: {} };
    var me = this;
    this.H.forEach(function (h) { c.S[h] = { e2: 0, n: 0 }; c.M[h] = { u: me._model(c), v: me._model(c) }; });
    if (spec.parent) warmStart(c, spec.parent);
    p.list.push(c);
    return c;
  };
  // train one model on the seconds still in the meter's buffer (~150 s)
  ForecastLoop.prototype._replay = function (id, c, epochs) {
    var R = this.rt[id]; if (!R) return;
    var secs = Object.keys(R.s).map(Number).sort(function (a, b) { return a - b; });
    for (var e = 0; e < epochs; e++) for (var i = 0; i < this.H.length; i++) {
      var h = this.H[i];
      for (var j = 0; j < secs.length; j++) { var s1 = secs[j], c0 = this._at(id, s1 - h), cur = R.s[s1]; if (c0) this._learn(c, h, this._x(c, id, s1 - h, h, c0), cur.u, cur.v); }
    }
  };
  ForecastLoop.prototype._dim = function (c) { return 4 + c.leaders.length + 3 * (c.inputs || []).length; };
  ForecastLoop.prototype._model = function (c) {
    var n = this._dim(c), r = this.rnd;
    if (c.kind === 'rls') {
      var w = new Array(n).fill(0); w[1] = 1;
      var P = eye(new Array(n).fill(0).map(function (_, i) { return i === 0 ? 1 : i < 4 ? 0.01 : 0.2; }));
      return { w: w, P: P };
    }
    if (c.kind === 'mlp') {
      var H = c.hidden, ni = n - 1, W1 = [], b1 = [], W2 = [];
      for (var i = 0; i < H; i++) { var row = []; for (var j = 0; j < ni; j++) row.push((r() - 0.5) * 0.6); W1.push(row); b1.push(0); W2.push(0); }   // W2 = 0: starts as "stays", random W1 keeps the units different
      return { W1: W1, b1: b1, W2: W2, b2: 0 };
    }
    return {};
  };
  ForecastLoop.prototype._at = function (id, sec) { var R = this.rt[id]; return R ? R.s[sec] || null : null; };
  ForecastLoop.prototype._near = function (id, sec) { return this._at(id, sec) || this._at(id, sec - 1) || this._at(id, sec + 1) || this._at(id, sec - 2); };
  // inputs for a model of meter id, made at second s0 for s0 + h
  // a channel at second s0, from the meter if it carries it, else the base:
  // z = how far from its usual value (in SDs), d = its change over 60 s
  ForecastLoop.prototype._chan = function (id, k, s0) {
    var R = this.rt[id], E = this.env, src = null, st = null;
    function get(S, s) { for (var j = 0; j <= 2; j++) { var o = S[s - j]; if (o && o.x && o.x[k] != null) return o.x[k]; } return null; }
    var v = R ? get(R.s, s0) : null;
    if (v != null) { src = R.s; st = R.cs[k]; } else { v = get(E.s, s0); if (v != null) { src = E.s; st = E.cs[k]; } }
    if (v == null || !st || st.n < 10) return { z: 0, d: 0 };
    var sd = Math.sqrt(Math.max(st.v, 1e-9)), old = get(src, s0 - 60);
    return { z: Math.max(-4, Math.min(4, (v - st.m) / sd)), d: old == null ? 0 : Math.max(-4, Math.min(4, (v - old) / sd)) };
  };
  function chanStats(cs, x) {
    for (var k in x) {
      var v = x[k]; if (!isFinite(v)) continue;
      var S = cs[k] || (cs[k] = { m: v, v: 0, n: 0 }); S.n++;
      var a = 1 / Math.min(S.n, 600), d = v - S.m; S.m += a * d; S.v += a * (d * d - S.v);
    }
  }
  ForecastLoop.prototype.addEnv = function (t, x) {
    var sec = Math.round(t); if (!x) return;
    var E = this.env;
    if (E.last != null && sec < E.last - 120) { E.s = {}; E.last = sec; }   // the base restarted
    if (E.last == null || sec > E.last) E.last = sec;
    if (this.env.s[sec]) return;
    this.env.s[sec] = { x: x }; chanStats(this.env.cs, x);
    for (var k in x) this.chan[k] = true;
    for (var q in this.env.s) if (+q < sec - 150) delete this.env.s[q];
  };
  ForecastLoop.prototype.channels = function () { return Object.keys(this.chan).sort(); };
  ForecastLoop.prototype._x = function (c, id, s0, h, cur) {
    var prev = this._at(id, s0 - 5) || this._at(id, s0 - 4) || this._at(id, s0 - 6) || cur;
    var xu = [1, cur.u, cur.mu, cur.u - prev.u], xv = [1, cur.v, cur.mv, cur.v - prev.v];
    for (var i = 0; i < c.leaders.length; i++) {
      var L = c.leaders[i], lv = this._near(L.by, Math.min(s0, s0 + h - L.lag));
      if (!lv) { var R = this.rt[L.by]; lv = R && isFinite(R.last) && R.last <= s0 && R.last > s0 - 10 ? R.s[R.last] : null; }
      xu.push(lv ? lv.u : cur.u); xv.push(lv ? lv.v : cur.v);
    }
    for (i = 0; i < (c.inputs || []).length; i++) {
      var I = c.inputs[i], f = this._chan(id, I.k, I.lag ? Math.min(s0, s0 + h - I.lag) : s0);
      xu.push(f.z, f.d, f.z * cur.u / 10); xv.push(f.z, f.d, f.z * cur.v / 10);
    }
    return { u: xu, v: xv };
  };
  function mlpOut(m, x) {
    var H = m.W1.length, a = new Array(H), o = m.b2;
    for (var i = 0; i < H; i++) { var s = m.b1[i], w = m.W1[i]; for (var j = 1; j < x.length; j++) s += w[j - 1] * x[j] / 10; a[i] = Math.tanh(s); o += m.W2[i] * a[i]; }
    return { o: o, a: a };
  }
  function mlpStep(m, x, y, lr) {          // y: change from now / 10
    var f = mlpOut(m, x), e = Math.max(-3, Math.min(3, f.o - y)), H = m.W1.length;
    for (var i = 0; i < H; i++) {
      var g = e * m.W2[i] * (1 - f.a[i] * f.a[i]);
      m.W2[i] -= lr * e * f.a[i];
      m.b1[i] -= lr * g;
      for (var j = 1; j < x.length; j++) m.W1[i][j - 1] -= lr * g * x[j] / 10;
    }
    m.b2 -= lr * e;
  }
  ForecastLoop.prototype._pred = function (c, h, X) {
    if (c.kind === 'stay') return { x: X.u[1], y: X.v[1] };
    var mu = c.M[h].u, mv = c.M[h].v;
    if (c.kind === 'rls') { var a = 0, b = 0; for (var i = 0; i < X.u.length; i++) { a += mu.w[i] * X.u[i]; b += mv.w[i] * X.v[i]; } return { x: a, y: b }; }
    return { x: X.u[1] + 10 * mlpOut(mu, X.u).o, y: X.v[1] + 10 * mlpOut(mv, X.v).o };
  };
  ForecastLoop.prototype._learn = function (c, h, X, tu, tv) {
    if (c.kind === 'rls') { rlsStep(c.M[h].u, X.u, tu, c.lam); rlsStep(c.M[h].v, X.v, tv, c.lam); }
    else if (c.kind === 'mlp') { mlpStep(c.M[h].u, X.u, (tu - X.u[1]) / 10, c.lr); mlpStep(c.M[h].v, X.v, (tv - X.v[1]) / 10, c.lr); }
  };
  // the blend: the best few grown-up models, weighted by 1 / recent error
  ForecastLoop.prototype._blend = function (p, h, preds) {
    var ok = p.list.filter(function (c) { return c.S[h].n >= 60 && preds[c.id]; }).sort(function (a, b) { return a.S[h].e2 - b.S[h].e2; }).slice(0, 3);
    if (!ok.length) { var st = p.list[0]; return preds[st.id]; }
    var sw = 0, x = 0, y = 0;
    ok.forEach(function (c) { var w = 1 / Math.max(1e-3, c.S[h].e2); sw += w; x += w * preds[c.id].x; y += w * preds[c.id].y; });
    return { x: x / sw, y: y / sw };
  };
  ForecastLoop.prototype.add = function (id, t, mph, dir, extra) {
    var sec = Math.round(t); id = String(id);
    if (!isFinite(mph) || !isFinite(dir)) return;
    var R = this.rt[id] || (this.rt[id] = { s: {}, k: 0, mu: null, mv: null, last: -Infinity, cs: {} });
    // the clock went back (the base restarted and counts from zero again):
    // start this meter's timeline over, keep its models and what they learned
    if (sec < R.last - 120) { R.s = {}; R.last = -Infinity; if (this.P[id]) this.P[id].pend = {}; }
    if (R.s[sec]) return;
    var d = dir * Math.PI / 180, u = -Math.sin(d) * mph, v = -Math.cos(d) * mph;
    if (R.mu == null) { R.mu = u; R.mv = v; }
    R.k++; var al = 1 / Math.min(R.k, 60);
    R.mu += al * (u - R.mu); R.mv += al * (v - R.mv);
    var cur = R.s[sec] = { u: u, v: v, mu: R.mu, mv: R.mv };
    if (extra) { cur.x = extra; chanStats(R.cs, extra); for (var ck in extra) this.chan[ck] = true; }
    var newest = sec > R.last; if (newest) R.last = sec;
    for (var k in R.s) if (+k < R.last - 200) delete R.s[k];
    if (this.regimes && newest) this._regTick(sec);
    var p = this._pool(id), me = this, i;
    // 1) check what was predicted for this second
    var due = p.pend[sec];
    if (due) {
      delete p.pend[sec];
      due.forEach(function (q) {
        var h = q.h;
        p.list.forEach(function (c) {
          var pr = q.c[c.id]; if (!pr) return;
          var e = ((pr.x - u) * (pr.x - u) + (pr.y - v) * (pr.y - v)) / 2, S = c.S[h];
          S.n++; S.e2 += Math.max(1 / S.n, 1 / 200) * (e - S.e2);     // recent error: a slow starter is judged on what it does now
        });
        var eb = ((q.b.x - u) * (q.b.x - u) + (q.b.y - v) * (q.b.y - v)) / 2, ep = ((q.s.x - u) * (q.s.x - u) + (q.s.y - v) * (q.s.y - v)) / 2, B = p.B[h], W = p.W[h];
        B.n++; var a = Math.max(1 / B.n, 1 / 600); B.e2 += a * (eb - B.e2); B.p2 += a * (ep - B.p2);
        if (me.reg.cur != null) {                   // the same record, kept per wind
          var RBm = (me.RB[me.reg.cur] || (me.RB[me.reg.cur] = {}))[id] || (me.RB[me.reg.cur][id] = {}), rb = RBm[h] || (RBm[h] = { e2: 0, p2: 0, n: 0 });
          rb.n++; var ar = Math.max(1 / rb.n, 1 / 600); rb.e2 += ar * (eb - rb.e2); rb.p2 += ar * (ep - rb.p2);
        }
        W.se += eb; W.sp += ep; W.n++;
        if (h === 10) { p.trace.push({ t: sec, a: Math.hypot(u, v), p: Math.hypot(q.b.x, q.b.y), au: u, av: v, pu: q.b.x, pv: q.b.y }); if (p.trace.length > 240) p.trace.shift(); }
        p.checked++;
      });
    }
    for (var s in p.pend) if (+s < R.last - 30) delete p.pend[s];
    // 2) every model learns from it
    for (i = 0; i < this.H.length; i++) {
      var h = this.H[i], c0 = this._at(id, sec - h); if (!c0) continue;
      for (var j = 0; j < p.list.length; j++) { var c = p.list[j]; if (c.kind !== 'stay') this._learn(c, h, this._x(c, id, sec - h, h, c0), u, v); }
    }
    // 3) predict from this second, write it down
    if (newest) {
      for (i = 0; i < this.H.length; i++) {
        var hh = this.H[i], preds = {};
        p.list.forEach(function (c) { preds[c.id] = me._pred(c, hh, me._x(c, id, sec, hh, cur)); });
        (p.pend[sec + hh] || (p.pend[sec + hh] = [])).push({ h: hh, c: preds, b: this._blend(p, hh, preds), s: preds[p.list[0].id] });
      }
    }
    // 4) evolve
    p.k++;
    if (p.k % this.genS === 0) this._generation(id, p, sec);
  };
  function cdesc(c) {
    var k = c.kind === 'stay' ? 'Stays as it is' : c.kind === 'mlp' ? 'Neural net (' + c.hidden + ')' : 'Linear' + (c.lam < 0.995 ? ', fast memory' : '');
    if (c.kind !== 'stay') k += c.leaders.length ? ' + ' + c.leaders.map(function (l) { return l.by + ' ' + l.lag + ' s ahead'; }).join(' + ') : (c.inputs && c.inputs.length ? '' : ', own history');
    if (c.inputs && c.inputs.length) k += ' + ' + c.inputs.map(function (q) { return q.k + (q.lag ? ' ' + q.lag + ' s ahead' : ''); }).join(' + ');
    return k;
  }
  ForecastLoop.describe = cdesc;
  // relative score: mean over horizons of error / "stays" error (lower is better)
  ForecastLoop.prototype._score = function (p, c) {
    var st = p.list[0], s = 0, n = 0;
    for (var i = 0; i < this.H.length; i++) { var h = this.H[i]; if (c.S[h].n < 120 || st.S[h].e2 <= 0) return null; s += c.S[h].e2 / st.S[h].e2; n++; }
    return s / n;
  };
  ForecastLoop.prototype._generation = function (id, p, sec) {
    p.g++;
    var me = this, sk = {};
    this.H.forEach(function (h) { var W = p.W[h]; sk[h] = W.n && W.sp > 0 ? Math.round((1 - Math.sqrt(W.se / W.sp)) * 1000) / 1000 : null; p.W[h] = { se: 0, sp: 0, n: 0 }; });
    var ranked = p.list.map(function (c) { return { c: c, s: me._score(p, c) }; }).filter(function (r) { return r.s != null; }).sort(function (a, b) { return a.s - b.s; });
    var best = ranked.length ? ranked[0].c : null;
    p.hist.push({ g: p.g, t: sec, skill: sk, best: best ? best.id : null, size: p.list.length });
    if (p.hist.length > 400) p.hist.shift();
    function log(txt) { p.log.push({ g: p.g, t: sec, txt: txt }); if (p.log.length > 80) p.log.shift(); }
    if (best && best.id !== p.champ) { if (p.champ != null) log('New best: #' + best.id + ' ' + cdesc(best) + ', ' + Math.round((1 - Math.sqrt(ranked[0].s)) * 100) + '% better than "stays as it is"'); p.champ = best.id; }
    if (!this.evolve) return;
    // how much training this meter gets follows how much it matters to the
    // shot: a bigger pool and two new models a generation for the meters
    // that carry the call, a small pool for the ones that hardly count
    var cap = this.cap(id), tries = this.importanceFn && this._imp(id) >= 0.25 ? 2 : 1, me2 = this;
    function retire() {
      var cand = ranked.filter(function (r) { return p.list.indexOf(r.c) >= 0 && r.c.kind !== 'stay' && !isBase(r.c) && r.c.id !== p.champ && p.g - r.c.born >= (r.c.kind === 'mlp' && !(r.s > 1.05) ? MLP_GRACE : 3); });
      if (!cand.length) return false;
      var w = cand[cand.length - 1];
      p.list.splice(p.list.indexOf(w.c), 1);
      p.tabu = p.tabu || {}; p.tabu[lsig(w.c)] = p.g;
      log('Retired #' + w.c.id + ' ' + cdesc(w.c) + (best && w.s ? ' (misses ' + Math.sqrt(w.s / ranked[0].s).toFixed(1) + '× as much as the best)' : ''));
      return true;
    }
    // the cap counts the models that compete; "stays" and the own-history
    // yardstick ride along on top of it
    var nc = function () { return p.list.filter(function (c) { return c.kind !== 'stay' && !isBase(c); }).length; };
    while (nc() > cap && retire()) { /* the meter matters less now */ }
    for (var k = 0; k < tries; k++) {
      if (nc() >= cap && !retire()) break;
      var sigs = p.list.map(lsig), tabu = p.tabu || {};
      var bad = function (sp) { var g = lsig(sp); return sigs.indexOf(g) >= 0 || (tabu[g] != null && p.g - tabu[g] < 15); };
      var spec = me2._spawn(id, p, best);
      for (var t2 = 0; spec && t2 < 8 && bad(spec); t2++) spec = me2._spawn(id, p, best);
      if (!spec || bad(spec)) break;
      if (spec.kind === 'mlp' && p.list.filter(function (q) { return q.kind === 'mlp' && p.g - q.born < MLP_GRACE; }).length >= (cap >= 10 ? 2 : cap >= 7 ? 1 : 0)) { spec.kind = 'rls'; if (bad(spec)) break; }
      var c = me2._addCand(p, spec, spec.origin);
      // a net that can't start from a parent net's weights starts from the
      // last couple of minutes of wind instead of from random weights
      if (c.kind === 'mlp' && !(spec.parent && spec.parent.kind === 'mlp' && spec.parent.hidden === c.hidden)) me2._replay(id, c, 12);
      log('Trying #' + c.id + ' ' + cdesc(c) + ' (' + (spec.origin === 'relation' ? 'Relations found this pair' : spec.origin === 'explore' ? 'a blind guess' : 'a change to #' + spec.from) + ')');
    }
  };
  // a meter's share of the shot (0..1) from the page, and the pool size it earns
  ForecastLoop.prototype._imp = function (id) { var v = this.importanceFn ? +this.importanceFn(id) : NaN; return isFinite(v) ? Math.max(0, Math.min(1, v)) : null; };
  ForecastLoop.prototype.cap = function (id) { var v = this._imp(id); return v == null ? this.maxC : Math.max(4, Math.min(10, Math.round(4 + 16 * v))); };
  ForecastLoop.prototype._spawn = function (id, p, best) {
    var r = this.rnd, others = Object.keys(this.rt).filter(function (k) { return k !== id; });
    var sug = (this.leadersFn ? this.leadersFn(id) || [] : []).filter(function (l) { return l && l.by !== id; });
    var have = {}; p.list.forEach(function (c) { c.leaders.forEach(function (l) { have[l.by + '@' + l.lag] = 1; }); });
    var fresh = sug.filter(function (l) { return !have[l.by + '@' + Math.round(l.lagS)]; });
    var x = r();
    if (fresh.length && x < 0.4) { var l = fresh[Math.floor(r() * fresh.length)]; return { kind: 'rls', lam: 0.998, leaders: [{ by: l.by, lag: l.lagS }], origin: 'relation' }; }
    if (best && best.kind !== 'stay' && x < 0.8) {
      var s = { kind: best.kind, lam: best.lam, hidden: best.hidden, lr: best.lr, leaders: best.leaders.map(function (l) { return { by: l.by, lag: l.lag }; }), inputs: (best.inputs || []).map(function (q) { return { k: q.k, lag: q.lag }; }), origin: 'mutation', from: best.id, parent: best };
      var m = r(), chans = this.channels(), lagged = s.leaders.concat(s.inputs);
      if (lagged.length && m < 0.4) {                  // walk a Sentinel's (or sensor's) lag: next door first, then further
        var L = lagged[Math.floor(r() * lagged.length)], sigs = p.list.map(lsig), tabu = p.tabu || {}, base = L.lag;
        var steps = r() < 0.7 ? [1, -1, 2, -2, 3, -3, 4, -4] : [8, -8, 6, -6, 12, -12], pickd = null;
        for (var q = 0; q < steps.length && pickd == null; q++) { L.lag = Math.max(L.k ? 0 : 1, base + steps[q]); var g = lsig(s); if (sigs.indexOf(g) < 0 && !(tabu[g] != null && p.g - tabu[g] < 15)) pickd = L.lag; }
        L.lag = pickd == null ? Math.max(1, base + (r() < 0.5 ? -1 : 1) * (1 + Math.floor(r() * 6))) : pickd;
      }
      else if (chans.length && m < 0.5) { var ck = chans[Math.floor(r() * chans.length)], ix = s.inputs.findIndex(function (q) { return q.k === ck; }); if (ix >= 0) s.inputs.splice(ix, 1); else s.inputs.push({ k: ck, lag: r() < 0.5 ? 0 : 2 + Math.floor(r() * 38) }); }
      else if (m < 0.55 && s.leaders.length < 2 && others.length) { var pick = sug.length && r() < 0.6 ? sug[Math.floor(r() * sug.length)] : null; s.leaders.push(pick ? { by: pick.by, lag: pick.lagS } : { by: others[Math.floor(r() * others.length)], lag: 2 + Math.floor(r() * 38) }); }
      else if (m < 0.65 && s.leaders.length) s.leaders.splice(Math.floor(r() * s.leaders.length), 1);
      else if (m < 0.82) s.lam = [0.99, 0.995, 0.998, 0.999][Math.floor(r() * 4)];
      else if (s.kind === 'rls') { s.kind = 'mlp'; s.hidden = 6; } else { s.kind = 'rls'; }
      return s;
    }
    var cs = this.channels();
    if (cs.length && (r() < 0.25 || !others.length)) return { kind: r() < 0.3 ? 'mlp' : 'rls', lam: 0.998, hidden: 6, leaders: [], inputs: [{ k: cs[Math.floor(r() * cs.length)], lag: r() < 0.5 ? 0 : 2 + Math.floor(r() * 38) }], origin: 'explore' };
    if (!others.length) return null;
    // a blind guess: any other meter, up to 2 min ahead (short leads more often)
    var bl = r() < 0.7 ? 2 + Math.floor(r() * 38) : 40 + Math.floor(r() * 80);
    return { kind: r() < 0.25 ? 'mlp' : 'rls', lam: 0.998, hidden: 6, leaders: [{ by: others[Math.floor(r() * others.length)], lag: bl }], origin: 'explore' };
  };
  ForecastLoop.prototype.latest = function (id) { var R = this.rt[String(id)]; return R && isFinite(R.last) ? R.last : null; };
  ForecastLoop.prototype._one = function (id, s0, h) {
    var cur = this._at(id, s0), p = this.P[id]; if (!cur || !p) return null;
    var me = this, preds = {};
    p.list.forEach(function (c) { preds[c.id] = me._pred(c, h, me._x(c, id, s0, h, cur)); });
    var b = this._blend(p, h, preds), B = p.B[h], ready = B.n >= 60;
    return { x: b.x, y: b.y, sd: B.n >= 10 ? Math.sqrt(B.e2) : null, n: B.n, ready: ready };
  };
  ForecastLoop.prototype.predict = Forecaster.prototype.predict;
  ForecastLoop.prototype.meters = function () { return Object.keys(this.rt).sort(); };
  ForecastLoop.prototype.stats = function (id) {
    var p = this.P[String(id)];
    return this.H.map(function (h) {
      var B = p && p.B[h]; if (!B || !B.n) return { h: h, n: 0 };
      var e = Math.sqrt(B.e2), q = Math.sqrt(B.p2);
      return { h: h, n: B.n, rmse: e, rmseStay: q, skill: q > 0 ? 1 - e / q : 0 };
    });
  };
  // the pool for a meter, best first at horizon h (default 10 s)
  ForecastLoop.prototype.pool = function (id, h) {
    var p = this.P[String(id)], me = this; h = h || 10; if (!p) return [];
    var st = p.list[0];
    return p.list.map(function (c) {
      var S = c.S[h], sc = me._score(p, c);
      return { id: c.id, kind: c.kind, leaders: c.leaders.slice(), inputs: (c.inputs || []).slice(), desc: cdesc(c), origin: c.origin, from: c.from, born: c.born, n: S.n,
        rmse: S.n ? Math.sqrt(S.e2) : null, skill: S.n >= 30 && st.S[h].e2 > 0 ? 1 - Math.sqrt(S.e2 / st.S[h].e2) : null, score: sc, champ: c.id === p.champ };
    }).sort(function (a, b) { return (a.rmse == null) - (b.rmse == null) || a.rmse - b.rmse; });
  };
  ForecastLoop.prototype.history = function (id) { var p = this.P[String(id)]; return p ? p.hist.slice() : []; };
  ForecastLoop.prototype.log = function (id) { var p = this.P[String(id)]; return p ? p.log.slice() : []; };
  ForecastLoop.prototype.trace = function (id) { var p = this.P[String(id)]; return p ? p.trace.slice() : []; };
  ForecastLoop.prototype.info = function (id) { var p = this.P[String(id)]; return p ? { generation: p.g, checked: p.checked, models: p.list.length, cap: this.cap(String(id)), share: this._imp(String(id)), champ: p.champ } : null; };
  // Sentinel/node pairs the loop has proven: the best model at 10 s leans
  // on another meter and beats the best model without one by >= 10 %
  ForecastLoop.prototype.provenPairs = function () {
    var me = this, out = [];
    Object.keys(this.P).forEach(function (id) {
      var L = me.pool(id, 10).filter(function (c) { return c.n >= 120 && c.rmse != null; });
      var withL = L.filter(function (c) { return c.leaders.length; })[0], own = L.filter(function (c) { return !c.leaders.length && c.kind !== 'stay'; })[0];
      if (!withL || !own || withL.rmse > own.rmse * 0.9) return;
      withL.leaders.forEach(function (l) { out.push({ sentId: l.by, nodeId: id, lagS: l.lag, gain: 1 - withL.rmse / own.rmse, model: withL.id }); });
    });
    return out;
  };
  function packModel(m, noCov) {
    // the covariance P to 4 figures (it only sets how fast weights move), weights to 6
    var o = {}; for (var k in m) { if (noCov && k === 'P') { o.P = null; continue; } var v = m[k], rf = k === 'P' ? round4 : round6; o[k] = Array.isArray(v) ? v.map(function (x) { return Array.isArray(x) ? x.map(rf) : rf(x); }) : rf(v); }
    return o;
  }
  // a pool as saved data. compact (a wind not blowing now): the best few
  // models only, linear models without their covariance (it rebuilds in a
  // few minutes of that wind), short history
  ForecastLoop.prototype._packPool = function (p, compact) {
    var me = this, list = p.list;
    if (compact) {
      var ranked = list.map(function (c) { return { c: c, s: me._score(p, c) }; }).filter(function (r) { return r.c.kind !== 'stay' && !isBase(r.c); })
        .sort(function (a, b) { return (a.s == null) - (b.s == null) || a.s - b.s; }).slice(0, 4).map(function (r) { return r.c; });
      list = list.filter(function (c) { return c.kind === 'stay' || isBase(c) || c.id === p.champ || ranked.indexOf(c) >= 0; });
    }
    return { next: p.next, g: p.g, k: p.k, checked: p.checked, champ: p.champ, B: p.B, hist: compact ? p.hist.slice(-60) : p.hist, log: compact ? p.log.slice(-20) : p.log, tabu: p.tabu || {},
      list: list.map(function (c) {
        var M = {}; for (var h in c.M) M[h] = { u: packModel(c.M[h].u, compact), v: packModel(c.M[h].v, compact) };
        return { id: c.id, kind: c.kind, lam: c.lam, hidden: c.hidden, lr: c.lr, leaders: c.leaders, inputs: c.inputs || [], born: c.born, origin: c.origin, from: c.from, S: c.S, M: M };
      }) };
  };
  ForecastLoop.prototype.toJSON = function () {
    var P = {}, A = {}, me = this;
    for (var id in this.P) P[id] = this._packPool(this.P[id], false);
    for (var rk in this.A) { A[rk] = {}; for (var id2 in this.A[rk]) A[rk][id2] = this.A[rk][id2]; }
    return { kind: 'loop', v: 1, H: this.H, maxC: this.maxC, genS: this.genS, seed: this.seed, channels: this.channels(), P: P, A: A,
      regimes: this.regimes, noSpeedBands: this.noSpeedBands, regCur: this.reg.cur, regSecs: this.regSecs, RB: this.RB };
  };
  ForecastLoop.prototype._restorePool = function (o) {
    var p = this._newPool(), H = this.H;
    p.list = o.list.map(function (c) {
      var d = JSON.parse(JSON.stringify(c)); H.forEach(function (h) { d.S[h] = d.S[h] || { e2: 0, n: 0 }; });
      // put away without its covariance: start it modest, it grows back in that wind
      for (var h in d.M) ['u', 'v'].forEach(function (q) { var m = d.M[h][q]; if (m && m.w && m.P == null) m.P = eye(m.w.map(function () { return 0.05; })); });
      return d;
    });
    p.list.forEach(function (c) { c.inputs = (c.inputs || []).map(function (q) { return typeof q === 'string' ? { k: q, lag: 0 } : q; }); });
    ['next', 'g', 'k', 'checked', 'champ', 'hist', 'log', 'tabu'].forEach(function (k) { if (o[k] != null) p[k] = o[k]; });
    if (o.B) p.B = JSON.parse(JSON.stringify(o.B));
    return p;
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
          var ex = expectedLag(A, B, wind);
          // alongYd: how far the air travels from Sentinel to node, learned
          // from the wind itself, so Wind Lab can time the warning at any speed
          sec.sent.push({ sentId: A.nodeId, nodeId: B.nodeId, lagS: p.lagS, spdPct: p.spdPct, dirDeg: p.dirDeg, ratio: p.ratio, dirOff: p.dirOff, r: p.r, src: 'data', alongYd: p.alongYd > 0 ? p.alongYd : ex && ex.alongYd > 0 ? Math.round(ex.alongYd) : undefined });
        });
      }
      // meters the data has not paired yet get a pair from the layout
      if (o.layoutPairs !== false) geometryPairs(o.meters, wind).forEach(function (p) {
        var A = byId[p.sentId], B = byId[p.nodeId];
        if (sec.sent.some(function (q) { return q.nodeId === B.nodeId; })) return;
        sec.sent.push(Object.assign({}, p, { sentId: A.nodeId, nodeId: B.nodeId }));
      });
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

  // chance a shot lands on a plate, from the windage doubt and the rifle's
  // spread (mil, 1 SD) and the plate's half width / half height (mil)
  function erf(x) { var t = 1 / (1 + 0.3275911 * Math.abs(x)), y = 1 - (((((1.061405429 * t - 1.453152027) * t) + 1.421413741) * t - 0.284496736) * t + 0.254829592) * t * Math.exp(-x * x); return x >= 0 ? y : -y; }
  function hitChance(o) {
    var sx = Math.max(1e-6, Math.sqrt(o.windSd * o.windSd + o.rifleSd * o.rifleSd)), sy = Math.max(1e-6, o.rifleSd * 1.1);
    var off = o.offset || 0, hw = o.halfW, hh = o.halfH;
    if (o.round) { hw = hh = o.halfW * 0.8862; }               // the square with the same area
    var px = 0.5 * (erf((hw - off) / (sx * Math.SQRT2)) + erf((hw + off) / (sx * Math.SQRT2)));
    return px * erf(hh / (sy * Math.SQRT2));
  }

  root.WindLearner = {
    YD_PER_S_PER_MPH: YD_PER_S_PER_MPH, SECTORS: SECTORS, SECTOR_NAMES: SECTOR_NAMES,
    lineOfFire: lineOfFire, components: components, fromDeg: fromDeg, sectorOf: sectorOf,
    geometry: geometry, physicsWeights: physicsWeights, Learner: Learner,
    Relations: Relations, relationKind: relationKind, expectedLag: expectedLag, upwindOk: upwindOk,
    SecondAligner: SecondAligner, buildLiveModel: buildLiveModel, geometryPairs: geometryPairs, Forecaster: Forecaster, ForecastLoop: ForecastLoop, regimeKey: regimeKey, regimeName: regimeName,
    hitChance: hitChance
  };
})(typeof window !== 'undefined' ? window : globalThis);
