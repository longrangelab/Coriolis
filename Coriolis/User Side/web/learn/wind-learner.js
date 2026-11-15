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

  root.WindLearner = {
    YD_PER_S_PER_MPH: YD_PER_S_PER_MPH, SECTORS: SECTORS, SECTOR_NAMES: SECTOR_NAMES,
    lineOfFire: lineOfFire, components: components, fromDeg: fromDeg, sectorOf: sectorOf,
    geometry: geometry, physicsWeights: physicsWeights, Learner: Learner
  };
})(typeof window !== 'undefined' ? window : globalThis);
