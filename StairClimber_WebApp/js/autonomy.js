/*
 * The robot's autonomy stack: perception, decision and control.
 * Same algorithms as the Python/PyBullet code (perception/step_detector.py,
 * perception/kalman.py, control/*.py), extended with obstacle detection,
 * surface classification, the landing planner and the U-turn mission.
 */
(function (G) {
  "use strict";
  const { R, gauss } = G.SCRobot;
  const SURF = G.SCWorld.SURFACES;
  const DEG = Math.PI / 180;

  // =================================================================== perception
  const H_TOL = 20 * DEG, V_TOL = 70 * DEG, MIN_RISER = 0.04, MIN_TREAD_SEEN = 0.04;
  const SENSOR_LEVEL_BAND = 0.008, WALL_MIN = 0.40;
  const median = a => { const s = a.slice().sort((p, q) => p - q); const n = s.length; return n % 2 ? s[(n - 1) / 2] : (s[n / 2 - 1] + s[n / 2]) / 2; };
  const isNum = p => !Number.isNaN(p[0]);

  class Seg {
    constructor(kind, pts) {
      this.kind = kind; this.pts = pts;
      const xs = pts.map(p => p[0]), zs = pts.map(p => p[1]);
      this.x = median(xs); this.z = median(zs);
      this.xmin = Math.min(...xs); this.xmax = Math.max(...xs);
      this.zmin = Math.min(...zs); this.zmax = Math.max(...zs);
    }
  }

  // H/V label from the principal direction of a neighbourhood that spans >= 1.5 cm
  function labelPoints(pts) {
    const n = pts.length, labels = new Array(n).fill("-");
    const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);
    for (let i = 0; i < n; i++) {
      if (!isNum(pts[i])) continue;
      const idx = [i], ends = { "-1": i, "1": i };
      for (let it = 0; it < 20; it++) {
        let grown = false;
        for (const st of [-1, 1]) {
          const k2 = ends[st] + st;
          if (k2 >= 0 && k2 < n && isNum(pts[k2]) && dist(pts[k2], pts[ends[st]]) <= 0.05) { idx.push(k2); ends[st] = k2; grown = true; }
        }
        if (dist(pts[ends[1]], pts[ends[-1]]) >= 0.03 || !grown) break;
      }
      if (idx.length < 3 || dist(pts[ends[1]], pts[ends[-1]]) < 0.015) continue;
      let mx = 0, mz = 0;
      idx.forEach(k => { mx += pts[k][0]; mz += pts[k][1]; });
      mx /= idx.length; mz /= idx.length;
      let sxx = 0, szz = 0, sxz = 0;
      idx.forEach(k => { const dx = pts[k][0] - mx, dz = pts[k][1] - mz; sxx += dx * dx; szz += dz * dz; sxz += dx * dz; });
      const ang0 = 0.5 * Math.atan2(2 * sxz, sxx - szz);           // principal axis angle
      let ang = Math.abs(ang0); ang = Math.min(ang, Math.PI - ang);
      if (ang < H_TOL) labels[i] = "H"; else if (ang > V_TOL) labels[i] = "V";
    }
    return labels;
  }

  function segment(pts, labels) {
    const segs = []; let cur = [], prev = null;
    const flush = () => { if (cur.length >= 3) segs.push(new Seg(prev, cur)); cur = []; };
    for (let i = 0; i < pts.length; i++) {
      const lab = labels[i];
      if (lab !== "H" && lab !== "V") { flush(); prev = null; continue; }
      if (cur.length && (lab !== prev || Math.hypot(pts[i][0] - cur[cur.length - 1][0], pts[i][1] - cur[cur.length - 1][1]) > 0.06)) flush();
      cur.push(pts[i]); prev = lab;
    }
    flush();
    return segs;
  }

  function mergeLevels(hsegs) {
    const levels = [];
    hsegs.slice().sort((a, b) => a.z - b.z).forEach(s => {
      const L = levels[levels.length - 1];
      if (L && Math.abs(s.z - L.z) < 0.015) { L.pts = L.pts.concat(s.pts); L.z = median(L.pts.map(p => p[1])); }
      else levels.push({ z: s.z, pts: s.pts.slice() });
    });
    return levels.map(L => ({ z: L.z, seg: new Seg("H", L.pts) }));
  }

  function mergeFaces(vsegs) {
    const faces = [];
    vsegs.slice().sort((a, b) => a.x - b.x).forEach(s => {
      const f = faces[faces.length - 1];
      if (f && Math.abs(s.x - f.x) < 0.02) faces[faces.length - 1] = new Seg("V", f.pts.concat(s.pts));
      else faces.push(new Seg("V", s.pts));
    });
    return faces;
  }

  function detectStep(pts, groundPrior) {
    const valid = pts.filter(isNum);
    const out = { stepAhead: false, segs: [], faces: [], levels: [] };
    if (valid.length < 10) return out;
    const labels = labelPoints(pts);
    const segs = segment(pts, labels);
    const faces = mergeFaces(segs.filter(s => s.kind === "V"));
    const levels = mergeLevels(segs.filter(s => s.kind === "H"));
    Object.assign(out, { segs, faces, levels, labels });
    const match = levels.filter(l => Math.abs(l.z - groundPrior) < 0.02);
    const z0 = match.length ? match[0].z : groundPrior;
    out.groundZ = z0;
    const above = levels.filter(l => l.z > z0 + MIN_RISER && l.seg.xmax - l.seg.xmin >= MIN_TREAD_SEEN);
    if (!above.length) {
      const walls = faces.filter(f => f.zmin < z0 + 0.06 && f.zmax > z0 + MIN_RISER);
      if (!walls.length) return out;
      const w = walls[0];
      if (w.zmax - z0 > WALL_MIN) return Object.assign(out, { wall: true, wallDist: w.x });
      if (w.zmax > -SENSOR_LEVEL_BAND) return Object.assign(out, { stepAhead: true, blocked: true, height: w.zmax - z0, riserDist: w.x });
      // sparse tread recovery: grazing beams on a tread near sensor height
      const nxt = faces.filter(f => f.x > w.x + 0.1 && f.zmax > w.zmax + 0.02);
      const xEnd = nxt.length ? nxt[0].x - 0.02 : w.x + 1;
      const on = valid.filter(p => p[0] > w.x + 0.02 && p[0] < xEnd && Math.abs(p[1] - w.zmax) < 0.015);
      let top = w.zmax;
      if (on.length >= 2) {
        const zs = on.map(p => p[1]), m = zs.reduce((a, b) => a + b, 0) / zs.length;
        const sd = Math.sqrt(zs.reduce((a, b) => a + (b - m) ** 2, 0) / zs.length);
        if (sd < 0.005) top = median(zs);
      }
      return Object.assign(out, { stepAhead: true, height: top - z0, riserDist: w.x, depth: nxt.length ? nxt[0].x - w.x : null });
    }
    const z1 = above[0].z, tread1 = above[0].seg;
    const risers = faces.filter(f => f.zmin < z1 - 0.01 && f.zmax > z0 + 0.01 && f.x < tread1.xmin + 0.03);
    const riserX = risers.length ? risers[risers.length - 1].x : tread1.xmin;
    let depth = null;
    const nf = faces.filter(f => f.x > riserX + 0.1 && f.zmax > z1 + 0.02);
    if (nf.length) depth = nf[0].x - riserX;
    else if (above.length > 1 && above[1].seg.xmin > riserX + 0.1) depth = above[1].seg.xmin - riserX;
    const isLanding = depth == null ? tread1.xmax - riserX > 0.6 : depth > 0.6;
    return Object.assign(out, { stepAhead: true, height: z1 - z0, riserDist: riserX, depth, isLanding, z1 });
  }

  class Kalman1D {
    constructor(q, r) { this.q = q; this.r = r; this.reset(); }
    reset() { this.x = null; this.p = 1; this.n = 0; }
    update(z, r) {
      if (z == null || Number.isNaN(z)) return this.x;
      r = r == null ? this.r : r;
      if (this.x == null) { this.x = z; this.p = r; }
      else { this.p += this.q; const k = this.p / (this.p + r); this.x += k * (z - this.x); this.p *= 1 - k; }
      this.n++;
      return this.x;
    }
    get std() { return Math.sqrt(this.p); }
  }

  class Perception {
    constructor(sensors) {
      this.s = sensors;
      const r = 2 * 0.004 ** 2;
      this.kfH = new Kalman1D(1e-8, r); this.kfD = new Kalman1D(1e-8, 4 * r);
      this.kfRiser = new Kalman1D(1e-8, r); this.kfW = new Kalman1D(1e-8, 2 * 0.005 ** 2);
      this.reset();
    }
    reset() {
      [this.kfH, this.kfD, this.kfRiser].forEach(k => k.reset());
      this.votes = { step: 0, none: 0, landing: 0, blocked: 0, wall: 0 };
      this.wallDist = []; this.history = []; this.surfProbs = null; this.surfN = 0;
    }
    observe(odomS) {
      const imu = this.s.imu();
      const pts = this.s.lidar(imu);
      const obs = detectStep(pts, -R.LIDAR_HEIGHT);
      this.lastPts = pts; this.lastObs = obs;
      if (obs.stepAhead) {
        this.votes.step++;
        if (obs.blocked) this.votes.blocked++;
        this.kfH.update(obs.height);
        this.kfRiser.update(odomS + R.LIDAR_X + obs.riserDist);
        if (obs.depth != null) this.kfD.update(obs.depth);
        if (obs.isLanding) this.votes.landing++;
      } else {
        this.votes.none++;
        if (obs.wall) { this.votes.wall++; this.wallDist.push(obs.wallDist); }
      }
      this.history.push({ raw: obs.height == null ? null : obs.height, kf: this.kfH.x, rawD: obs.depth, kfD: this.kfD.x });
      return obs;
    }
    measureWidth(us, span) {
      if (us.sideL.d == null || us.sideR.d == null) return this.kfW.x;
      return this.kfW.update(span + R.UNIT_W + us.sideL.d + us.sideR.d);
    }
    estimate() {
      const v = this.votes;
      if (v.step <= v.none) {
        return { stepAhead: false, wall: v.wall > v.none / 2, wallDist: this.wallDist.length ? median(this.wallDist) : null, width: this.kfW.x };
      }
      return {
        stepAhead: true, height: this.kfH.x, depth: this.kfD.x, riserS: this.kfRiser.x, width: this.kfW.x,
        isLanding: v.landing > v.step / 2, blocked: v.blocked > v.step / 2, hStd: this.kfH.std, n: v.step,
      };
    }
    // simulated CNN on the camera image: the true class of the patch it looks at,
    // passed through a noisy softmax (about 95 % top-1 accuracy), averaged over frames
    classifySurface(points) {
      const counts = {};
      points.forEach(p => { if (p.up && p.surface) counts[p.surface] = (counts[p.surface] || 0) + 1; });
      const truth = Object.keys(counts).sort((a, b) => counts[b] - counts[a])[0];
      if (!truth) return this.surface();
      const keys = Object.keys(SURF);
      const logits = keys.map(k => (k === truth ? 2.6 : 0) + 0.7 * gauss(this.s.rng));
      const mx = Math.max(...logits), ex = logits.map(l => Math.exp(l - mx)), sum = ex.reduce((a, b) => a + b, 0);
      const probs = ex.map(e => e / sum);
      if (!this.surfProbs) this.surfProbs = probs.map(() => 0);
      this.surfProbs = this.surfProbs.map((p, i) => p + probs[i]);
      this.surfN++;
      return this.surface();
    }
    surface() {
      if (!this.surfProbs) return null;
      const keys = Object.keys(SURF);
      const avg = this.surfProbs.map(p => p / this.surfN);
      let bi = 0; avg.forEach((p, i) => { if (p > avg[bi]) bi = i; });
      return { label: keys[bi], conf: avg[bi], probs: Object.fromEntries(keys.map((k, i) => [k, avg[i]])) };
    }
  }

  // obstacle detection from depth-camera points against the stair model from the LiDAR
  function detectObstacles(pts, model) {
    const expected = x => {
      let z = model.z0;
      for (const rz of model.risers) if (x > rz.x) z = rz.z;
      return z;
    };
    const obsPts = [], cliffPts = [];
    pts.forEach(p => {
      if (p.y > model.yMax - 0.02 || p.y < model.yMin + 0.02 || p.x < 0.05) return;
      if (model.risers.some(rz => Math.abs(p.x - rz.x) < 0.03)) return;
      if (model.xMax != null && p.x > model.xMax) return;
      const e = expected(p.x);
      if (p.z > e + 0.022) obsPts.push(p);
      else if (model.flat && p.z < e - 0.04) cliffPts.push(p);
    });
    // connected components on a 6 cm hash grid
    const key = p => Math.floor(p.x / 0.06) + "," + Math.floor(p.y / 0.06);
    const buckets = new Map();
    obsPts.forEach((p, i) => { const k = key(p); if (!buckets.has(k)) buckets.set(k, []); buckets.get(k).push(i); });
    const seen = new Array(obsPts.length).fill(false), clusters = [];
    for (let i = 0; i < obsPts.length; i++) {
      if (seen[i]) continue;
      const q = [i]; seen[i] = true; const mem = [];
      while (q.length) {
        const a = q.pop(); mem.push(obsPts[a]);
        const [bx, by] = key(obsPts[a]).split(",").map(Number);
        for (let dx = -1; dx <= 1; dx++) for (let dy = -1; dy <= 1; dy++) {
          (buckets.get((bx + dx) + "," + (by + dy)) || []).forEach(j => {
            if (!seen[j] && Math.hypot(obsPts[j].x - obsPts[a].x, obsPts[j].y - obsPts[a].y) < 0.07) { seen[j] = true; q.push(j); }
          });
        }
      }
      if (mem.length < 3) continue;
      const xs = mem.map(p => p.x), ys = mem.map(p => p.y);
      const pad = 0.025;   // half the camera's ray spacing at this range
      clusters.push({
        x0: Math.min(...xs) - pad, x1: Math.max(...xs) + pad, y0: Math.min(...ys) - pad, y1: Math.max(...ys) + pad,
        top: Math.max(...mem.map(p => p.z - expected(p.x))), n: mem.length, pts: mem,
      });
    }
    return { clusters, cliffPts };
  }

  // =================================================================== control
  class PID {
    constructor(kp, ki, kd, outLim, iLim) { Object.assign(this, { kp, ki, kd, outLim: outLim || Infinity, iLim: iLim || Infinity }); this.reset(); }
    reset() { this.i = 0; this.prev = null; }
    step(err, dt) {
      this.i = Math.max(-this.iLim, Math.min(this.iLim, this.i + err * dt));
      const d = this.prev == null ? 0 : (err - this.prev) / dt;
      this.prev = err;
      return Math.max(-this.outLim, Math.min(this.outLim, this.kp * err + this.ki * this.i + this.kd * d));
    }
  }

  class Servo {
    constructor(robot, joint, kp, ki, maxVel) {
      this.robot = robot; this.joint = joint;
      this.pid = new PID(kp || 8, ki == null ? 2 : ki, 0, maxVel || 0.06, 0.01);
      this.target = robot.j[joint];
    }
    set(target, maxVel) { this.target = target; if (maxVel != null) this.pid.outLim = maxVel; }
    update(dt) { this.robot.jv[this.joint] = this.pid.step(this.target - this.robot.j[this.joint], dt); }
    err() { return this.target - this.robot.j[this.joint]; }
    at(tol) { return Math.abs(this.err()) < (tol || 0.002); }
  }

  const GAITS = [
    { name: "LOW", maxH: 0.13, margin: 0.020, drive: 0.09, lift: 0.07 },
    { name: "STANDARD", maxH: 0.185, margin: 0.025, drive: 0.08, lift: 0.06 },
    { name: "HIGH", maxH: R.RACK_TRAVEL - 0.03, margin: 0.030, drive: 0.05, lift: 0.04 },
  ];
  const MIN_TREAD = R.BACK_LEN + R.FRONT_LEN + 0.02;

  function selectGait(h, d, w) {
    if (w != null) {
      const need = w - R.UNIT_W - 2 * SCWorld.CLEAR;
      if (need < R.SPAN_MIN) return [null, `stairs too narrow (${(w * 1000).toFixed(0)} mm) for the frame's minimum width`];
      if (need > R.SPAN_MAX) return [null, `stairs too wide (${(w * 1000).toFixed(0)} mm) for the frame's maximum width`];
    }
    if (h == null) return [null, "no step geometry"];
    if (d != null && d < MIN_TREAD) return [null, `tread too shallow (${(d * 1000).toFixed(0)} mm, needs ${(MIN_TREAD * 1000).toFixed(0)} mm)`];
    for (const g of GAITS) if (h <= g.maxH) return [g, null];
    return [null, `riser too high (${(h * 1000).toFixed(0)} mm, rack limit ${(GAITS[2].maxH * 1000).toFixed(0)} mm)`];
  }

  // =================================================================== landing planner
  class OccGrid {
    constructor(x0, x1, y0, y1, res) {
      this.x0 = x0; this.y0 = y0; this.res = res;
      this.nx = Math.max(1, Math.ceil((x1 - x0) / res)); this.ny = Math.max(1, Math.ceil((y1 - y0) / res));
      this.occ = new Uint8Array(this.nx * this.ny);
      this.version = 0;
    }
    idx(x, y) { return [Math.floor((x - this.x0) / this.res), Math.floor((y - this.y0) / this.res)]; }
    center(ix, iy) { return [this.x0 + (ix + 0.5) * this.res, this.y0 + (iy + 0.5) * this.res]; }
    inside(ix, iy) { return ix >= 0 && iy >= 0 && ix < this.nx && iy < this.ny; }
    get(ix, iy) { return this.inside(ix, iy) ? this.occ[iy * this.nx + ix] : 1; }
    markDisk(x, y, rad) {
      const [cx, cy] = this.idx(x, y), n = Math.ceil(rad / this.res);
      let changed = false;
      for (let a = -n; a <= n; a++) for (let b = -n; b <= n; b++) {
        if ((a * a + b * b) * this.res * this.res > rad * rad) continue;
        const ix = cx + a, iy = cy + b;
        if (this.inside(ix, iy) && !this.occ[iy * this.nx + ix]) { this.occ[iy * this.nx + ix] = 1; changed = true; }
      }
      if (changed) this.version++;
      return changed;
    }
  }

  // footprint cell offsets for the robot at heading psi (relative to the odom frame)
  function footprint(grid, psi, spanL, spanR, margin) {
    const xs = [-R.BACK_LEN / 2 - 0.025 - margin, R.FRONT_FACE_X + margin];
    const ys = [-(spanR + R.UNIT_W / 2) - margin, spanL + R.UNIT_W / 2 + margin];
    const c = Math.cos(psi), s = Math.sin(psi), res = grid.res, out = [];
    const rmax = Math.hypot(Math.max(-xs[0], xs[1]), Math.max(-ys[0], ys[1]));
    const n = Math.ceil(rmax / res) + 1;
    for (let a = -n; a <= n; a++) for (let b = -n; b <= n; b++) {
      const wx = a * res, wy = b * res;
      const bx = wx * c + wy * s, by = -wx * s + wy * c;
      if (bx >= xs[0] && bx <= xs[1] && by >= ys[0] && by <= ys[1]) out.push([a, b]);
    }
    return { cells: out, radius: rmax };
  }

  function fits(grid, ix, iy, fp) {
    for (const [a, b] of fp.cells) if (grid.get(ix + a, iy + b)) return false;
    return true;
  }

  function rotationFree(grid, ix, iy, radius) {
    const n = Math.ceil(radius / grid.res);
    for (let a = -n; a <= n; a++) for (let b = -n; b <= n; b++) {
      if ((a * a + b * b) * grid.res * grid.res > radius * radius) continue;
      if (grid.get(ix + a, iy + b)) return false;
    }
    return true;
  }

  function astar(grid, start, goal, fp) {
    const key = (x, y) => y * grid.nx + x;
    const h = (x, y) => Math.hypot(x - goal[0], y - goal[1]);
    const open = [[h(...start), 0, start[0], start[1]]];
    const g = new Map([[key(...start), 0]]), from = new Map();
    const closed = new Set();
    let iter = 0;
    while (open.length && iter++ < 60000) {
      let bi = 0; for (let i = 1; i < open.length; i++) if (open[i][0] < open[bi][0]) bi = i;
      const [, gc, x, y] = open.splice(bi, 1)[0];
      const k = key(x, y);
      if (closed.has(k)) continue;
      closed.add(k);
      if (x === goal[0] && y === goal[1]) {
        const path = [[x, y]]; let c = k;
        while (from.has(c)) { c = from.get(c); path.push([c % grid.nx, Math.floor(c / grid.nx)]); }
        return path.reverse();
      }
      for (const [dx, dy] of [[1, 0], [-1, 0], [0, 1], [0, -1], [1, 1], [1, -1], [-1, 1], [-1, -1]]) {
        const nx = x + dx, ny = y + dy, nk = key(nx, ny);
        if (!grid.inside(nx, ny) || closed.has(nk) || !fits(grid, nx, ny, fp)) continue;
        if (grid.allowed && !grid.allowed(nx, ny) && grid.allowed(x, y)) continue;
        const ng = gc + Math.hypot(dx, dy);
        if (ng < (g.get(nk) ?? Infinity)) { g.set(nk, ng); from.set(nk, k); open.push([ng + h(nx, ny), ng, nx, ny]); }
      }
    }
    return null;
  }

  function simplify(grid, path, fp) {
    if (!path || path.length < 3) return path;
    const clear = (a, b) => {
      const n = Math.ceil(Math.hypot(b[0] - a[0], b[1] - a[1]) * 2);
      for (let i = 0; i <= n; i++) {
        const x = Math.round(a[0] + (b[0] - a[0]) * i / n), y = Math.round(a[1] + (b[1] - a[1]) * i / n);
        if (!fits(grid, x, y, fp)) return false;
      }
      return true;
    };
    const out = [path[0]]; let i = 0;
    while (i < path.length - 1) {
      let j = path.length - 1;
      while (j > i + 1 && !clear(path[i], path[j])) j--;
      out.push(path[j]); i = j;
    }
    return out;
  }

  // =================================================================== mission / state machine
  const STATE_INFO = {
    SCAN: "Scan the next step: LiDAR, depth camera, side rangefinders",
    OBSTACLE_WAIT: "Obstacle in the path: waiting for it to be removed",
    ALIGN_WIDTH: "Set frame width x so each unit clears its wall",
    APPROACH: "Drive to the riser (distance + heading PID)",
    CLEAN_TREAD: "Mop and vacuum the tread in rows",
    FRONT_LIFT: "Front racks lift the front units",
    ADVANCE_FRONT: "Push the raised front units over the next tread",
    PLACE_FRONT: "Lower front units until the wheels touch",
    REAR_LIFT: "Rear racks lift the back units, IMU keeps body level",
    ADVANCE_REAR: "Pull the raised back units over the tread",
    RETRACT_REAR: "Lower the back units onto the tread",
    LANDING_ADVANCE: "Landing: drive to its centre",
    LANDING_NARROW: "Landing: retract the frame to turn in less space",
    LANDING_PLAN: "Landing: map obstacles and plan a path (A*)",
    LANDING_MOVE: "Landing: follow the planned path (omni wheels)",
    LANDING_TURN: "Landing: turn 180° in place",
    LANDING_ALIGN: "Landing: line up with flight 2 using the wall and riser sensors",
    DONE: "Mission complete",
    SAFE_ABORT: "Stopped safely: every actuator held",
  };

  class Mission {
    constructor(world, opts) {
      opts = Object.assign({ seed: 3, lidarNoise: 0.004, span0: 0.6 }, opts || {});
      let a = opts.seed >>> 0 || 1;
      this.rng = () => { a |= 0; a = (a + 0x6d2b79f5) | 0; let t = Math.imul(a ^ (a >>> 15), 1 | a); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
      this.world = world;
      const W = world.width;
      const span0 = Math.max(R.SPAN_MIN, Math.min(opts.span0, W - R.UNIT_W - 0.04));
      this.robot = new SCRobot.Robot(world, { x: -0.35 - R.FRONT_FACE_X, y: W / 2, yaw: 0 }, span0, this.rng);
      this.sensors = new SCRobot.Sensors(this.robot, world, this.rng, { lidarNoise: opts.lidarNoise });
      this.per = new Perception(this.sensors);
      const r = this.robot;
      this.sv = {
        fl: new Servo(r, "fl"), leg: new Servo(r, "leg"),
        spanL: new Servo(r, "spanL", 4, 0.5, 0.12), spanR: new Servo(r, "spanR", 4, 0.5, 0.12),
        rail: new Servo(r, "rail", 6, 0, 0.25), reach: new Servo(r, "reach", 6, 0, 0.2), head: new Servo(r, "head", 6, 0, 0.1),
      };
      this.drivePid = new PID(2.5, 0.3, 0, 0.08, 0.02);
      this.pitchPid = new PID(0.25, 0.8, 0, 0.03, 0.04);
      this.yawPid = new PID(3, 0.5, 0, 0.5, 0.2);
      this.t = 0; this.k = 0; this.state = "SCAN"; this.tEnter = 0; this.entered = false;
      this.flight = 1; this.flightYaw = 0; this.stepInFlight = 0; this.tread = 0;
      this.riserHist = []; this.events = []; this.stepRecords = []; this.trim = 0;
      this.result = null; this.alert = null; this.gait = null; this.est = null; this.hContact = null;
      this.progress = []; this.history = []; this.surfaceByTread = {}; this.noClean = [];
      this.obstacles = { clusters: [] }; this.lastDepth = null; this.us = this.sensors.ultrasonics(); this.cl = this.sensors.cliff();
      this.landing = null; this.Wflight = null;
      this.pitch = 0; this.roll = 0;
      this.log("SCAN", "mission started");
    }

    get finished() { return this.state === "DONE" || this.state === "SAFE_ABORT"; }
    get heading() { return [Math.cos(this.flightYaw), Math.sin(this.flightYaw)]; }
    odomS() { const [hx, hy] = this.heading; return this.robot.odo.x * hx + this.robot.odo.y * hy; }
    odomLat() { const [hx, hy] = this.heading; return -this.robot.odo.x * hy + this.robot.odo.y * hx; }

    log(state, note) { this.events.push({ t: +this.t.toFixed(2), state, note: note || "" }); }
    goto(state, note) {
      this.log(state, note);
      this.state = state; this.tEnter = this.t; this.entered = false;
      this.drivePid.reset(); this.yawPid.reset(); this.progress = [];
      this.robot.cmd = { vx: 0, vy: 0, wz: 0 };
      this.decision = note || STATE_INFO[state];
    }
    abort(reason) {
      this.result = "Stopped safely: " + reason;
      Object.values(this.sv).forEach(s => s.set(this.robot.j[s.joint]));
      this.alert = { level: "crit", text: reason };
      this.goto("SAFE_ABORT", reason);
    }
    both(kind, target, maxVel) {
      if (kind === "span") { this.sv.spanL.set(target, maxVel); this.sv.spanR.set(target, maxVel); }
      else this.sv[kind].set(target, maxVel);
    }
    yawHold(dt, target) {
      let e = (target == null ? this.flightYaw : target) - this.robot.odo.yaw;
      e = Math.atan2(Math.sin(e), Math.cos(e));
      return this.yawPid.step(e, dt);
    }
    driveTo(target, dt, vmax, window) {
      this.drivePid.outLim = vmax;
      const err = target - this.odomS();
      const v = this.drivePid.step(err, dt);
      this.robot.cmd = { vx: v, vy: 0, wz: this.yawHold(dt) };
      if (Math.abs(err) < 0.002) return true;
      const t = this.robot.touch;
      this.progress.push(this.odomS());
      if (this.progress.length > Math.round(0.4 / dt)) {
        const moved = Math.abs(this.progress[this.progress.length - 1] - this.progress.shift());
        if (err > 0 && err < (window || 0.03) && moved < 0.001 && (t.front || t.back || t.pad)) {
          this.decision = `touch sensor: contact ${(err * 1000).toFixed(0)} mm before the odometry target`;
          return true;
        }
      }
      return false;
    }

    // drive towards an odometry target, but never slower than a creep: the move is
    // ended by a sensor (bumper / cliff sensor), not by odometry
    creepTo(target, dt, vmax) {
      this.drivePid.outLim = vmax;
      const v = Math.max(0.015, this.drivePid.step(target - this.odomS(), dt));
      this.robot.cmd = { vx: v, vy: 0, wz: this.yawHold(dt) };
    }

    surfaceHere() {
      const c = this.surfaceByTread[this.tread];
      return c ? c.label : this.world.P.surf1;
    }
    speedFactor() { return (SURF[this.surfaceHere()] || SURF.tile).speed; }

    // ---------------------------------------------------------- main tick
    tick(dt) {
      this.t += dt; this.k++;
      const r = this.robot;
      const imu = this.sensors.imu(); this.pitch = imu.pitch; this.roll = imu.roll;
      if (this.k % 8 === 0) { this.us = this.sensors.ultrasonics(); this.cl = this.sensors.cliff(); }
      if (!this.finished) {
        if (Math.abs(this.pitch) > 20 * DEG || Math.abs(this.roll) > 15 * DEG) this.abort(`tilt limit (pitch ${(this.pitch / DEG).toFixed(1)}°, roll ${(this.roll / DEG).toFixed(1)}°)`);
        else if (r.tipped) this.abort("centre of mass left the support area");
        else if (this.state !== "OBSTACLE_WAIT" && this.t - this.tEnter > (this.state.startsWith("LANDING") ? 90 : 40)) this.abort(`timeout in ${this.state}`);
      }
      const first = !this.entered; this.entered = true;
      this["s_" + this.state](dt, first);
      Object.values(this.sv).forEach(s => s.update(dt));
      r.cleaning = this.state === "CLEAN_TREAD" && r.j.head > R.HEAD_DOWN - 0.01;
      if (r.cleaning) this.cleanDirt();
      r.step(dt);
      if (this.k % 12 === 0) this.history.push({ t: this.t, pitch: r.pitch / DEG, roll: r.roll / DEG, fl: r.j.fl * 1000, leg: r.j.leg * 1000, state: this.state });
      if (this.history.length > 3000) this.history.shift();
    }

    cleanDirt() {
      const r = this.robot, c = Math.cos(r.yaw), s = Math.sin(r.yaw);
      const hc = r.worldOf([r.j.reach, r.j.rail, R.CLEANER_BOTTOM - r.j.head]);
      const hx = R.HEAD_D / 2, hy = R.HEAD_W / 2 + R.EDGE;
      for (const cell of this.world.dirt) {
        if (cell.clean) continue;
        const dx = cell.x - hc[0], dy = cell.y - hc[1];
        if (Math.abs(dx) > 0.3 || Math.abs(dy) > 0.4 || Math.abs(cell.z - hc[2]) > 0.05) continue;
        const bx = dx * c + dy * s, by = -dx * s + dy * c;
        if (Math.abs(bx) <= hx && Math.abs(by) <= hy) { cell.clean = true; cell.at = this.t; }
      }
    }

    coverage() {
      const d = this.world.dirt.filter(c => !c.blocked);
      return d.length ? d.filter(c => c.clean).length / d.length : 0;
    }

    // ---------------------------------------------------------- climbing states
    s_SCAN(dt, first) {
      const r = this.robot;
      if (first) { this.per.reset(); this.sides = []; this.scans = 0; this.depthScans = []; r.cmd = { vx: 0, vy: 0, wz: 0 }; this.alert = null; }
      r.cmd = { vx: 0, vy: 0, wz: 0 };
      if (this.t - this.tEnter < 0.3 || this.k % 8) return;
      this.per.observe(this.odomS());
      this.sides.push({ L: this.us.sideL.d, R: this.us.sideR.d });
      if (this.scans % 3 === 0) {
        const pts = this.sensors.depthCamera();
        this.depthScans.push(pts);
        this.lastDepth = pts;
      }
      this.scans++;
      if (this.scans < 12) return;
      // fuse the side-rangefinder widths only when standing in a flight (not on an open landing)
      const preview = this.per.votes.step > this.per.votes.none;
      if (preview && !this.onLanding) this.sides.forEach(sd => this.per.measureWidth({ sideL: { d: sd.L }, sideR: { d: sd.R } }, r.span));
      const est = this.est = this.per.estimate();
      if (this.onLanding && this.Wflight) est.width = this.Wflight;
      else if (est.stepAhead && est.width) this.Wflight = est.width;
      // a real riser spans the full width: the depth camera must see the raised tread
      // in both drive-unit lanes, otherwise the LiDAR has hit an object, not a step
      if (est.stepAhead && !est.blocked) {
        const c = this.confirmStep(est);
        if (c === false) {
          this.log("SCAN", "LiDAR edge not confirmed across the width by the depth camera: it is an object, not a step");
          est.stepAhead = false; est.objectAhead = true;
        }
      }
      const rec = { tread: this.tread, flight: this.flight, est: Object.assign({}, est), history: this.per.history.slice(), t: this.t };
      this.stepRecords.push(rec);
      if (!est.stepAhead) {
        if (this.tread === 0) { this.result = "No stairs detected"; return this.goto("DONE", this.result); }
        if (this.world.uturn && this.flight === 1) {
          return this.goto("LANDING_ADVANCE", `no riser ahead, wall at ${((est.wallDist || 0) * 1000).toFixed(0)} mm: landing reached`);
        }
        this.result = "Top of the staircase reached";
        return this.goto("DONE", this.result);
      }
      if (est.blocked) return this.abort(`riser of at least ${(est.height * 1000).toFixed(0)} mm rises above the LiDAR: too high to climb`);
      const [g, reason] = selectGait(est.height, est.depth, est.width);
      if (!g) return this.abort(reason);
      this.gait = g;
      // classify the surface of the tread ahead from the camera
      this.per.surfProbs = null; this.per.surfN = 0;
      const riserRel = est.riserS - this.odomS();
      this.depthScans.forEach(pts => this.per.classifySurface(pts.filter(p => p.x > riserRel + 0.03 && p.x < riserRel + (est.depth || 0.3) - 0.03)));
      const sc = this.per.surface();
      if (sc) this.surfaceByTread[this.tread + 1] = sc;
      // frame width targets
      let lt, rt;
      if (this.onLanding) {
        // at the foot of flight 2 the inner (spine) wall has not started yet: use the
        // width from flight 1 and align against the outer wall the right rangefinder sees
        const total = est.width - R.UNIT_W - 2 * SCWorld.CLEAR;
        const dr = this.sides.map(s => s.R).filter(v => v != null);
        rt = dr.length ? r.j.spanR + dr.reduce((a, b) => a + b, 0) / dr.length - SCWorld.CLEAR : total / 2;
        lt = total - rt;
      } else {
        const avg = k => { const v = this.sides.map(s => s[k]).filter(v => v != null); return v.length ? v.reduce((a, b) => a + b, 0) / v.length : null; };
        const dl = avg("L"), dr = avg("R");
        lt = dl == null ? r.j.spanL : r.j.spanL + dl - SCWorld.CLEAR;
        rt = dr == null ? r.j.spanR : r.j.spanR + dr - SCWorld.CLEAR;
      }
      lt = Math.max(R.SPAN_MIN / 2, Math.min(R.SPAN_MAX / 2, lt));
      rt = Math.max(R.SPAN_MIN / 2, Math.min(R.SPAN_MAX / 2, rt));
      this.spanTarget = [lt, rt];
      // obstacles on this tread or the next one, from the depth camera
      const model = {
        z0: -R.BASE_Z, flat: false, yMin: -(rt + R.UNIT_W / 2 + SCWorld.CLEAR), yMax: lt + R.UNIT_W / 2 + SCWorld.CLEAR,
        risers: [{ x: riserRel, z: -R.BASE_Z + est.height }].concat(est.depth ? [{ x: riserRel + est.depth, z: -R.BASE_Z + 2 * est.height }] : []),
        // on a long tread (landing / top floor) only the area the robot will stand on
        // after this climb matters; the landing planner deals with anything beyond it
        xMax: riserRel + (est.depth && est.depth < 0.6 ? est.depth : R.BACK_LEN + R.FRONT_LEN + 0.06),
      };
      this.obsModel = model;
      const found = this.evaluateObstacles();
      rec.gait = g.name; rec.surface = sc;
      const note = `step ${this.tread + 1}: h=${(est.height * 1000).toFixed(1)} mm, d=${est.depth ? (est.depth * 1000).toFixed(1) + " mm" : "long"}, w=${((est.width || 0) * 1000).toFixed(0)} mm, ${sc ? SURF[sc.label].name.toLowerCase() + " ahead" : ""} -> gait ${g.name}`;
      if (found.block) { this.alert = { level: "warn", text: found.block }; return this.goto("OBSTACLE_WAIT", found.block); }
      if (found.pass) this.log("SCAN", found.pass);
      if (Math.abs(lt - r.j.spanL) > 0.005 || Math.abs(rt - r.j.spanR) > 0.005) return this.goto("ALIGN_WIDTH", note);
      this.goto("APPROACH", note);
    }

    confirmStep(est) {
      const pts = [].concat(...this.depthScans);
      const riserRel = est.riserS - this.odomS(), z0 = -R.BASE_Z, z1 = z0 + est.height;
      const lanes = [this.robot.j.spanL, -this.robot.j.spanR];
      let seen = 0;
      for (const yc of lanes) {
        let up = 0, flat = 0;
        pts.forEach(p => {
          if (Math.abs(p.y - yc) > 0.06 || p.x < riserRel + 0.04 || p.x > riserRel + 0.22) return;
          if (Math.abs(p.z - z1) < 0.025) up++; else if (Math.abs(p.z - z0) < 0.025) flat++;
        });
        if (up + flat < 2) continue;          // camera cannot see this lane: no vote
        seen++;
        if (flat > up) return false;
      }
      return seen ? true : null;
    }

    evaluateObstacles() {
      const pts = [].concat(...(this.depthScans.length ? this.depthScans : [this.sensors.depthCamera()]));
      const det = detectObstacles(pts, this.obsModel);
      this.obstacles = det;
      const [lt, rt] = this.spanTarget;
      const lane = (y0, y1, c) => y1 > c - R.UNIT_W / 2 - 0.02 && y0 < c + R.UNIT_W / 2 + 0.02;
      const s0 = this.odomS(), lat0 = this.odomLat();
      let block = null, pass = null;
      det.clusters.forEach(cl => {
        const where = cl.x0 < this.obsModel.risers[0].x ? "this tread" : `step ${this.tread + 1}`;
        const size = `${(cl.top * 100).toFixed(0)} cm tall`;
        if (lane(cl.y0, cl.y1, lt)) block = block || `Obstacle on ${where} in the LEFT drive unit's path (${size}). Waiting for it to be removed.`;
        else if (lane(cl.y0, cl.y1, -rt)) block = block || `Obstacle on ${where} in the RIGHT drive unit's path (${size}). Waiting for it to be removed.`;
        else if (cl.top > 0.045) block = block || `Obstacle on ${where} between the units is ${size}, taller than the 4.5 cm clearance under the cleaning unit. Waiting.`;
        else {
          pass = `small obstacle (${size}) between the units on ${where}: the frame passes over it; cleaning will avoid it`;
          this.noClean.push({ s0: s0 + cl.x0 - 0.02, s1: s0 + cl.x1 + 0.02, l0: lat0 + cl.y0 - 0.02, l1: lat0 + cl.y1 + 0.02, flight: this.flight });
        }
      });
      return { block, pass };
    }

    s_OBSTACLE_WAIT(dt, first) {
      this.robot.cmd = { vx: 0, vy: 0, wz: 0 };
      if (first) { this.recheck = this.t; this.lastReplan = this.t; }
      if (this.t - this.recheck < 0.6) return;
      this.recheck = this.t;
      this.depthScans = [this.sensors.depthCamera(), this.sensors.depthCamera()];
      this.lastDepth = this.depthScans[0];
      if (this.landing) {
        // landing: rebuild the map from fresh sensor data every 2 s and try again
        if (this.t - this.lastReplan < 2) return;
        this.lastReplan = this.t;
        this.landing.grid = null; this.landing.pending = [];
        const fail = this.planLanding();
        if (!fail) { this.alert = null; return this.goto("LANDING_MOVE", "landing clear again: new A* path"); }
        return;
      }
      const found = this.evaluateObstacles();
      if (!found.block) { this.alert = null; this.goto("SCAN", "path clear again: re-scanning"); }
      else this.alert = { level: "warn", text: found.block };
    }

    s_ALIGN_WIDTH(dt, first) {
      if (first) { this.sv.spanL.set(this.spanTarget[0]); this.sv.spanR.set(this.spanTarget[1]); }
      this.robot.cmd = { vx: 0, vy: 0, wz: 0 };
      if (this.t - this.tEnter > 0.3 && this.sv.spanL.at(0.003) && this.sv.spanR.at(0.003)) this.goto("APPROACH", `frame width x = ${(this.robot.span * 1000).toFixed(0)} mm`);
    }

    s_APPROACH(dt) {
      if (this.driveTo(this.est.riserS - 0.015 - R.FRONT_FACE_X, dt, this.gait.drive * this.speedFactor())) {
        this.robot.cmd = { vx: 0, vy: 0, wz: 0 };
        const t = this.world.treads; void t;
        if (this.stepInFlight >= 1) this.goto("CLEAN_TREAD", `cleaning ${SURF[this.surfaceHere()].name.toLowerCase()}: ${SURF[this.surfaceHere()].mode.toLowerCase()}`);
        else this.goto("FRONT_LIFT");
      }
    }

    s_FRONT_LIFT(dt, first) {
      this.robot.cmd = { vx: 0, vy: 0, wz: 0 };
      if (first) { this.robot.compliant.fl = false; this.sv.fl.set(this.est.height + this.gait.margin, this.gait.lift); this.sv.leg.set(0, this.gait.lift); }
      else if (this.sv.fl.at()) this.goto("ADVANCE_FRONT");
    }
    s_ADVANCE_FRONT(dt, first) {
      // push the raised front units over the tread until the back units' bumpers touch the
      // riser: the touch sensors fix the position even when the wheels slip
      if (first) this.touchT = 0;
      const target = this.est.riserS - R.BACK_LEN / 2;
      this.creepTo(target + 0.02, dt, this.gait.drive * this.speedFactor());
      this.touchT = this.robot.touch.back ? this.touchT + dt : 0;
      if (this.touchT > 0.15) { this.robot.cmd = { vx: 0, vy: 0, wz: 0 }; return this.goto("PLACE_FRONT", "back bumpers touched the riser"); }
      if (this.odomS() > target + 0.06) this.abort("bumpers never touched the riser: position lost");
    }
    s_PLACE_FRONT(dt, first) {
      const r = this.robot;
      r.cmd = { vx: 0, vy: 0, wz: 0 };
      if (first) { r.compliant.fl = true; this.sv.fl.set(this.est.height - 0.006, 0.03); this.stall = 0; }
      this.stall = r.flStall ? this.stall + dt : 0;
      if (this.t - this.tEnter > 0.4 && (this.stall > 0.25 || this.sv.fl.at())) {
        this.hContact = r.j.fl + 0.0005 * gauss(this.rng);
        r.compliant.fl = false;
        this.sv.fl.set(this.hContact, 2 * this.gait.lift);
        this.pitchPid.reset();
        this.stepRecords[this.stepRecords.length - 1].hContact = this.hContact;
        this.goto("REAR_LIFT", `front wheels touched down at ${(this.hContact * 1000).toFixed(1)} mm (rack encoder)`);
      }
    }
    coordinateFront(dt, retracting) {
      this.trim = this.pitchPid.step(this.pitch, dt);
      const backLift = this.robot.j.leg - R.PAD_GAP;
      const tgt = retracting ? Math.min(0, this.hContact - backLift) : this.hContact - Math.max(backLift, 0);
      this.sv.fl.set(tgt + this.trim);
    }
    s_REAR_LIFT(dt, first) {
      this.robot.cmd = { vx: 0, vy: 0, wz: 0 };
      if (first) this.sv.leg.set(this.hContact + this.gait.margin + R.PAD_GAP, this.gait.lift);
      this.coordinateFront(dt, false);
      if (this.t - this.tEnter > 0.3 && this.sv.leg.at(0.003)) this.goto("ADVANCE_REAR");
    }
    s_ADVANCE_REAR(dt) {
      this.coordinateFront(dt, false);
      // drive until the rear IR cliff sensors see the upper tread under the back units'
      // trailing edge (or the pads touch the riser); odometry alone drifts on slippery treads
      const padFront = -(R.BACK_LEN / 2 + 0.01) + R.PAD_R;
      const target = this.est.riserS - 0.004 - padFront;
      this.creepTo(target + 0.03, dt, this.gait.drive * this.speedFactor());
      this.cl = this.sensors.cliff();
      const overTread = (this.robot.j.leg - R.PAD_GAP) - this.hContact + R.CLEARANCE;
      const thresh = overTread + 0.5 * this.hContact;
      const seen = this.cl.BL.d < thresh && this.cl.BR.d < thresh;
      if (seen || this.robot.touch.pad) {
        this.robot.cmd = { vx: 0, vy: 0, wz: 0 };
        return this.goto("RETRACT_REAR", seen ? "rear cliff sensors see the new tread" : "pads touched the riser");
      }
      if (this.odomS() > target + 0.08) this.abort("rear sensors never found the tread");
    }
    s_RETRACT_REAR(dt, first) {
      this.robot.cmd = { vx: 0, vy: 0, wz: 0 };
      if (first) this.sv.leg.set(0, this.gait.lift);
      this.coordinateFront(dt, true);
      if (this.sv.leg.at(0.003) && this.t - this.tEnter > 0.3) {
        this.trim = 0; this.sv.fl.set(0);
        if (this.sv.fl.at(0.004)) {
          this.riserHist.push(this.est.riserS);
          this.tread++; this.stepInFlight++; this.onLanding = false;
          this.goto("SCAN", `climbed step ${this.tread}`);
        }
      }
    }

    s_CLEAN_TREAD(dt, first) {
      const r = this.robot;
      r.cmd = { vx: 0, vy: 0, wz: 0 };
      if (first) this.planCleaning();
      if (this.wp >= this.cleanWps.length) {
        ["rail", "reach", "head"].forEach(k => this.sv[k].set(0));
        if (Math.abs(r.j.head) < 0.004 && Math.abs(r.j.reach) < 0.01 && Math.abs(r.j.rail) < 0.01) this.goto("FRONT_LIFT");
        return;
      }
      const [y, x, hd] = this.cleanWps[this.wp];
      const sweep = (SURF[this.surfaceHere()] || SURF.tile).sweep;
      this.sv.rail.set(y, sweep); this.sv.reach.set(x, 0.2); this.sv.head.set(hd, 0.1);
      const headOk = hd === 0 ? r.j.head < 0.004 : r.j.head > hd - 0.012;
      if (Math.abs(r.j.rail - y) < 0.004 && Math.abs(r.j.reach - x) < 0.004 && headOk) this.wp++;
    }

    planCleaning() {
      const r = this.robot, s = this.odomS(), lat = this.odomLat();
      const x0 = this.riserHist[this.riserHist.length - 1], x1 = this.est.riserS;
      const lo = Math.max(x0 + R.HEAD_D / 2 - 0.02, s - R.REACH_MAX), hi = Math.min(x1 - R.HEAD_D / 2 - 0.005, s + R.REACH_MAX);
      const nRows = Math.max(1, Math.ceil((hi - lo) / 0.12) + 1);
      const rows = Array.from({ length: nRows }, (_, i) => lo + (hi - lo) * (nRows === 1 ? 0 : i / (nRows - 1)) - s);
      const ymax = r.j.spanL - R.UNIT_W / 2 - R.HEAD_W / 2 - 0.01, ymin = -(r.j.spanR - R.UNIT_W / 2 - R.HEAD_W / 2 - 0.01);
      const down = R.HEAD_DOWN;
      const wps = [];
      let dir = 1;
      rows.forEach(rx => {
        // keep-out intervals from small obstacles the frame passed over
        const blocked = this.noClean.filter(z => z.flight === this.flight && z.s1 > s + rx - R.HEAD_D / 2 && z.s0 < s + rx + R.HEAD_D / 2)
          .map(z => [z.l0 - lat - R.HEAD_W / 2 - R.EDGE, z.l1 - lat + R.HEAD_W / 2 + R.EDGE]);
        let segs = [[ymin, ymax]];
        blocked.forEach(([b0, b1]) => {
          segs = segs.flatMap(([a, b]) => (b1 <= a || b0 >= b) ? [[a, b]] : [[a, b0], [b1, b]].filter(([p, q]) => q - p > 0.02));
        });
        if (dir < 0) segs = segs.reverse().map(([a, b]) => [b, a]);
        segs.forEach(([a, b]) => { wps.push([a, rx, 0], [a, rx, down], [b, rx, down], [b, rx, 0]); });
        dir = -dir;
      });
      this.cleanWps = wps; this.wp = 0;
    }

    // ---------------------------------------------------------- landing
    s_LANDING_ADVANCE(dt, first) {
      const r = this.robot;
      if (first) {
        const fd = [this.us.frontL.d, this.us.frontR.d].filter(v => v != null);
        // the farther echo is the wall; a nearer one in the other lane is an object
        const dWall = fd.length ? Math.max(...fd) : (this.est.wallDist || 1);
        const sLand = this.riserHist[this.riserHist.length - 1];
        const wallS = this.odomS() + R.FRONT_FACE_X + dWall;
        const L = wallS - sLand;
        this.landing = { sLand, wallS, L };
        const fpR = Math.hypot(R.SPAN_MIN / 2 + R.UNIT_W / 2, R.FRONT_FACE_X) + 0.03;
        if (L < 2 * fpR + 0.02) return this.abort(`landing only ${(L * 1000).toFixed(0)} mm deep: too small to turn`);
        this.landing.target = sLand + L / 2;
        this.log("LANDING_ADVANCE", `landing depth ${(L * 1000).toFixed(0)} mm measured (front ultrasonic + odometry)`);
      }
      if (this.k % 10 === 0) this.markLandingObstacles(this.lastDepth = this.sensors.depthCamera());
      const fd = [this.us.frontL.d, this.us.frontR.d].filter(v => v != null);
      const remaining = this.landing.target - this.odomS();
      const stop = why => { this.robot.cmd = { vx: 0, vy: 0, wz: 0 }; return this.goto("LANDING_NARROW", why); };
      if (fd.length && Math.min(...fd) < remaining + 0.08 && Math.min(...fd) < Math.max(...fd) - 0.1)
        return stop(`object ${(Math.min(...fd) * 1000).toFixed(0)} mm ahead on the landing (ultrasonic): stopping to plan around it`);
      if (this.k % 10 === 0 && this.lastDepth) {
        // depth camera: anything in the corridor the robot is about to sweep
        const r = this.robot;
        const det = detectObstacles(this.lastDepth, { z0: -R.BASE_Z, flat: true, risers: [],
          yMin: -(r.j.spanR + R.UNIT_W / 2 + 0.03), yMax: r.j.spanL + R.UNIT_W / 2 + 0.03, xMax: R.FRONT_FACE_X + remaining + 0.08 });
        if (det.clusters.length) return stop(`object in the robot's corridor on the landing (depth camera): stopping to plan around it`);
      }
      if (this.robot.touch.cleaner || this.robot.touch.front) return stop("bumper contact on the landing: stopping to plan around it");
      if (this.driveTo(this.landing.target, dt, 0.08)) this.goto("LANDING_NARROW", "frame retracts to turn in a smaller circle");
    }

    s_LANDING_NARROW(dt, first) {
      this.robot.cmd = { vx: 0, vy: 0, wz: 0 };
      if (first) this.both("span", R.SPAN_MIN / 2, 0.15);
      if (this.sv.spanL.at(0.003) && this.sv.spanR.at(0.003) && this.t - this.tEnter > 0.3) this.goto("LANDING_PLAN");
    }

    landingGrid() {
      if (this.landing.grid) return this.landing.grid;
      const r = this.robot;
      const us = this.sensors.ultrasonics();
      const dl = us.sideL.d, dr = us.sideR.d;
      if (dl == null || dr == null) return null;
      // re-measure the landing depth: an object may have hidden the end wall earlier
      if (Math.cos(r.odo.yaw) > 0) {
        const fd = [us.frontL.d, us.frontR.d].filter(v => v != null);
        if (fd.length) this.landing.wallS = Math.max(this.landing.wallS, this.odomS() + R.FRONT_FACE_X + Math.max(...fd));
      }
      const left = this.robot.odo.y + r.j.spanL + R.UNIT_W / 2 + dl;
      const right = this.robot.odo.y - r.j.spanR - R.UNIT_W / 2 - dr;
      Object.assign(this.landing, { left, right });
      // the grid starts 13 cm before the landing edge: the robot's tail may overhang the
      // drop to flight 1, but not the rising first step of flight 2 beside it
      const g = this.landing.grid = new OccGrid(this.landing.sLand - 0.13, this.landing.wallS - 0.005, right + 0.005, left - 0.005, 0.03);
      const flight2From = right + (this.Wflight || 0.9) - 0.03;
      for (let ix = 0; ix < g.nx; ix++) for (let iy = 0; iy < g.ny; iy++) {
        const [x, y] = g.center(ix, iy);
        // +6 cm: odometry can be a few cm off after a flight of wheel slip
        if (x < this.landing.sLand + 0.06 && y > flight2From) g.occ[iy * g.nx + ix] = 1;
      }
      // the back wheels (6 cm behind the centre) must stay on the landing
      g.allowed = (ix, iy) => g.center(ix, iy)[0] >= this.landing.sLand + 0.07;
      (this.landing.pending || []).forEach(([x, y]) => this.landing.grid.markDisk(x, y, 0.04));
      // bumper contacts are remembered for 20 s, then the robot is allowed to try again
      this.landing.marks = (this.landing.marks || []).filter(mk => this.t - mk.t < 20);
      this.landing.marks.forEach(mk => this.landing.grid.markDisk(mk.x, mk.y, mk.r));
      return this.landing.grid;
    }

    markLandingObstacles(pts) {
      if (!pts || !this.landing) return false;
      const r = this.robot, c = Math.cos(r.odo.yaw), s = Math.sin(r.odo.yaw);
      let changed = false;
      pts.forEach(p => {
        if (p.z < -R.BASE_Z + 0.022 || p.z > 0.3) return;
        const X = r.odo.x + p.x * c - p.y * s, Y = r.odo.y + p.x * s + p.y * c;
        if (X < this.landing.sLand + 0.02 || X > this.landing.wallS - 0.04) return;
        if (this.landing.grid) {
          if (Y < this.landing.right + 0.04 || Y > this.landing.left - 0.04) return;
          changed = this.landing.grid.markDisk(X, Y, 0.04) || changed;
        } else (this.landing.pending = this.landing.pending || []).push([X, Y]);
      });
      return changed;
    }

    // plan the next landing leg on the occupancy grid; returns null or a reason it failed
    planLanding() {
      const r = this.robot, grid = this.landingGrid(), L = this.landing;
      if (!grid) return "side rangefinders cannot see the landing walls";
      this.markLandingObstacles(this.lastDepth = this.sensors.depthCamera());
      const fp0 = footprint(grid, 0, r.j.spanL, r.j.spanR, 0.02);
      const fpPi = footprint(grid, Math.PI, r.j.spanL, r.j.spanR, 0.02);
      const start = grid.idx(r.odo.x, r.odo.y);
      if (Math.cos(r.odo.yaw) > 0) {
        // turning spot: the rotation-free cell nearest the landing centre
        const c = grid.idx((L.sLand + L.wallS) / 2, (L.left + L.right) / 2);
        let spot = null, best = Infinity;
        for (let ix = 0; ix < grid.nx; ix++) for (let iy = 0; iy < grid.ny; iy++) {
          const dd = Math.hypot(ix - c[0], iy - c[1]);
          if (dd < best && grid.allowed(ix, iy) && rotationFree(grid, ix, iy, fp0.radius + 0.02)) { best = dd; spot = [ix, iy]; }
        }
        if (!spot) return "no free circle on the landing big enough to turn";
        const path = astar(grid, start, spot, fp0);
        if (!path) return "no obstacle-free path to a turning spot";
        L.path = simplify(grid, path, fp0).map(([ix, iy]) => grid.center(ix, iy));
        L.leg = 1; L.fp = fp0; L.spot = grid.center(...spot);
        return null;
      }
      const gx = L.sLand + 0.35 + R.FRONT_FACE_X, gy = L.left - this.Wflight / 2;
      const path = astar(grid, start, grid.idx(gx, gy), fpPi);
      if (!path) return "no obstacle-free path to the foot of flight 2";
      L.path = simplify(grid, path, fpPi).map(([ix, iy]) => grid.center(ix, iy));
      L.path[L.path.length - 1] = [gx, gy];
      L.leg = 2; L.fp = fpPi;
      return null;
    }

    s_LANDING_PLAN(dt, first) {
      this.robot.cmd = { vx: 0, vy: 0, wz: 0 };
      if (!first) return;
      const fail = this.planLanding();
      if (fail) {
        if (/walls/.test(fail)) return this.abort(fail);
        this.alert = { level: "warn", text: `Landing blocked: ${fail}. Waiting for the obstacle to be removed.` };
        return this.goto("OBSTACLE_WAIT", `landing blocked: ${fail}`);
      }
      const L = this.landing;
      this.goto("LANDING_MOVE", L.leg === 1
        ? `A* path to a turning spot (${L.path.length - 1} segment${L.path.length > 2 ? "s" : ""})`
        : `A* path to the foot of flight 2 (${L.path.length - 1} segment${L.path.length > 2 ? "s" : ""})`);
    }

    s_LANDING_MOVE(dt, first) {
      const r = this.robot, L = this.landing;
      if (first) { L.wp = 1; L.gridVersion = L.grid.version; }
      if (this.k % 10 === 0) {
        const pts = this.sensors.depthCamera(); this.lastDepth = pts;
        if (this.markLandingObstacles(pts)) {
          // new obstacle cells: check the remaining path
          const g = L.grid;
          const blocked = L.path.slice(L.wp).some(([x, y]) => !fits(g, ...g.idx(x, y), L.fp));
          if (blocked) return this.goto("LANDING_PLAN", "new obstacle seen on the path: re-planning");
        }
      }
      if (Object.values(this.cl).some(c => c.cliff)) return this.abort("IR cliff sensor: edge detected under the robot");
      const yawT = L.leg === 1 ? 0 : Math.PI;
      if (L.wp >= L.path.length) {
        r.cmd = { vx: 0, vy: 0, wz: 0 };
        if (L.leg === 1) return this.goto("LANDING_TURN", "at the turning spot");
        return this.goto("LANDING_ALIGN", "at the foot of flight 2: correcting odometry against the walls");
      }
      const [tx, ty] = L.path[L.wp];
      const dx = tx - r.odo.x, dy = ty - r.odo.y, dist = Math.hypot(dx, dy);
      const last = L.wp === L.path.length - 1;
      if (dist < (last ? 0.004 : 0.02)) { L.wp++; return; }
      const sp = Math.min(0.12, (last ? 1.5 : 3) * dist + 0.01);
      const c = Math.cos(r.odo.yaw), s = Math.sin(r.odo.yaw);
      const vxw = dx / dist * sp, vyw = dy / dist * sp;
      // side ultrasonics look sideways, where the camera cannot: anything closer than
      // the wall goes into the map
      if (this.k % 8 === 0) {
        for (const [key, sgn] of [["sideL", 1], ["sideR", -1]]) {
          const u = this.us[key];
          if (u.d == null) continue;
          const yb = sgn > 0 ? r.j.spanL + R.UNIT_W / 2 + u.d : -(r.j.spanR + R.UNIT_W / 2 + u.d);
          const X = r.odo.x - yb * s, Y = r.odo.y + yb * c;
          if (Y < L.left - 0.08 && Y > L.right + 0.08 && L.grid.markDisk(X, Y, 0.05)) {
            if (L.path.slice(L.wp).some(([x, y]) => !fits(L.grid, ...L.grid.idx(x, y), L.fp))) return this.goto("LANDING_PLAN", "side ultrasonic found an object beside the path: re-planning");
          }
        }
      }
      const vb = [vxw * c + vyw * s, -vxw * s + vyw * c];
      r.cmd = { vx: vb[0], vy: vb[1], wz: this.yawHold(dt, yawT) };
      const t = r.touch;
      if (t.front || t.side || t.back || t.cleaner || t.pad) {
        // bumper hit: put the contact into the map along the leading edge, then re-plan
        r.cmd = { vx: 0, vy: 0, wz: 0 };
        // the bumper segment that closed tells where the contact is on the body
        const n = Math.hypot(vb[0], vb[1]) || 1;
        const [bx, by] = r.touchAt || [vb[0] / n * 0.25, vb[1] / n * 0.25];
        const mk = { x: r.odo.x + bx * c - by * s, y: r.odo.y + bx * s + by * c, r: 0.06, t: this.t };
        (L.marks = L.marks || []).push(mk);
        L.grid.markDisk(mk.x, mk.y, mk.r);
        L.contacts = (L.contacts || 0) + 1;
        const part = t.cleaner ? "cleaning unit" : t.side ? "side" : t.back ? "rear" : "front";
        return this.goto("LANDING_PLAN", `${part} bumper contact on the landing: marked on the map, re-planning`);
      }
    }

    // wheel slip on the landing makes odometry drift sideways; re-reference against the
    // outer wall (right rangefinder) and the first riser (front ultrasonics)
    s_LANDING_ALIGN(dt) {
      const r = this.robot;
      this.us = this.sensors.ultrasonics();
      const dR = this.us.sideR.d, f = [this.us.frontL.d, this.us.frontR.d].filter(v => v != null);
      if (dR == null || !f.length) return this.abort("cannot see the wall or the first riser of flight 2");
      const eLat = dR - (this.Wflight / 2 - r.j.spanR - R.UNIT_W / 2);     // > 0: too far left of the wall
      const eFwd = f.reduce((a, b) => a + b, 0) / f.length - 0.35;
      if (Math.abs(eLat) < 0.003 && Math.abs(eFwd) < 0.005) {
        r.cmd = { vx: 0, vy: 0, wz: 0 };
        this.flight = 2; this.flightYaw = Math.PI; this.stepInFlight = 0; this.onLanding = true;
        this.per.kfW.reset();
        this.riserHist = []; this.landing = null;
        return this.goto("SCAN", "lined up with flight 2");
      }
      const cl = v => Math.max(-0.06, Math.min(0.06, v));
      r.cmd = { vx: cl(1.5 * eFwd), vy: cl(-1.5 * eLat), wz: this.yawHold(dt, Math.PI) };
    }

    s_LANDING_TURN(dt) {
      const r = this.robot;
      let e = Math.PI - r.odo.yaw; e = Math.atan2(Math.sin(e), Math.cos(e));
      if (Math.abs(e) < 0.004) { r.cmd = { vx: 0, vy: 0, wz: 0 }; return this.goto("LANDING_PLAN", "turned 180° (IMU gyro)"); }
      r.cmd = { vx: 0, vy: 0, wz: Math.max(-0.6, Math.min(0.6, 2 * e + Math.sign(e) * 0.02)) };
      if (r.touch.side || r.touch.front || r.touch.back) this.abort("contact while turning");
    }

    s_DONE() { this.robot.cmd = { vx: 0, vy: 0, wz: 0 }; }
    s_SAFE_ABORT() { this.robot.cmd = { vx: 0, vy: 0, wz: 0 }; }
  }

  G.SCAutonomy = { Mission, detectStep, detectObstacles, Kalman1D, PID, selectGait, GAITS, STATE_INFO, OccGrid, footprint, astar };
})(typeof globalThis !== "undefined" ? globalThis : window);
