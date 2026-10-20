/*
 * Robot body model and sensors.
 *
 * The body is rigid (back units + rods + cleaning unit) with three actuated
 * offsets: the front units ride on the front racks (fl), the rear pads on the
 * rear racks (leg), and the frame width is set by the telescopic rods.
 * Each step the resting pose is found from the contact points (wheels, pads)
 * by minimising the height of the centre of mass subject to "no point below
 * the ground" - a small linear programme. That gives pitch, roll, which
 * contacts carry load, and tip-over when the CoM leaves the support.
 * Motion is blocked when any part of the body would run into the geometry
 * (that is what the touch sensors report).
 */
(function (G) {
  "use strict";

  const R = {
    FRONT_LEN: 0.10, BACK_LEN: 0.20, UNIT_W: 0.15, BODY_H: 0.284, CLEARANCE: 0.03,
    WHEEL_R: 0.035, BASE_Z: 0.172, BACK_WHEEL_X: 0.06, FRONT_WHEEL_X: 0.151,
    PAD_X: -0.11, PAD_GAP: 0.01, PAD_R: 0.012, RACK_TRAVEL: 0.25, FL_MIN: -0.05, LEG_MAX: 0.28,
    SPAN_MIN: 0.50, SPAN_MAX: 1.50, CLEAN: 0.15, REACH_MAX: 0.20, HEAD_MAX: 0.12,
    HEAD_W: 0.15, HEAD_D: 0.15, EDGE: 0.04, COM_X: 0.03,
    FRONT_FACE_X: 0.201, LIDAR_X: 0.211, LIDAR_Z: 0.05, CLEANER_BOTTOM: -0.115,
    CAM: [0.12, 0, 0.13], CAM_PITCH: -0.6,
  };
  R.LIDAR_HEIGHT = R.BASE_Z + R.LIDAR_Z;
  R.HEAD_DOWN = R.BASE_Z + R.CLEANER_BOTTOM - 0.004;   // head travel that just touches the tread

  // ------------------------------------------------------------------ helpers
  function gauss(rng) {
    let u = 0, v = 0;
    while (u === 0) u = rng();
    while (v === 0) v = rng();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
  }
  function rotMat(yaw, th, ph) {               // Rz(yaw) * Ry(-th) * Rx(ph)
    const cy = Math.cos(yaw), sy = Math.sin(yaw);
    const ct = Math.cos(th), st = Math.sin(th);
    const cp = Math.cos(ph), sp = Math.sin(ph);
    // Ry(-th) = [[ct,0,-st],[0,1,0],[st,0,ct]], Rx(ph) = [[1,0,0],[0,cp,-sp],[0,sp,cp]]
    const a = [[ct, -st * sp, -st * cp], [0, cp, -sp], [st, ct * sp, ct * cp]];
    return [
      [cy * a[0][0] - sy * a[1][0], cy * a[0][1] - sy * a[1][1], cy * a[0][2] - sy * a[1][2]],
      [sy * a[0][0] + cy * a[1][0], sy * a[0][1] + cy * a[1][1], sy * a[0][2] + cy * a[1][2]],
      a[2],
    ];
  }
  const mv = (M, p) => [M[0][0] * p[0] + M[0][1] * p[1] + M[0][2] * p[2],
                        M[1][0] * p[0] + M[1][1] * p[1] + M[1][2] * p[2],
                        M[2][0] * p[0] + M[2][1] * p[1] + M[2][2] * p[2]];

  function solve3(A, b) {
    const det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
    if (Math.abs(det) < 1e-10) return null;
    const r = [];
    for (let c = 0; c < 3; c++) {
      const M = A.map((row, i) => row.map((v, j) => (j === c ? b[i] : v)));
      r.push((M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0])) / det);
    }
    return r;
  }

  // ------------------------------------------------------------------ robot
  class Robot {
    constructor(world, pose, span, rng) {
      this.world = world;
      this.rng = rng || Math.random;
      this.x = pose.x; this.y = pose.y; this.yaw = pose.yaw || 0;
      this.j = { fl: 0, leg: 0, spanL: span / 2, spanR: span / 2, rail: 0, reach: 0, head: 0 };
      this.jv = { fl: 0, leg: 0, spanL: 0, spanR: 0, rail: 0, reach: 0, head: 0 };
      this.compliant = { fl: false };
      this.cmd = { vx: 0, vy: 0, wz: 0 };
      this.z = R.BASE_Z; this.t = 0; this.r = 0;
      this.tight = {}; this.tipped = false;
      this.touch = { front: false, back: false, pad: false, side: false, cleaner: false, frontWheelLoad: false };
      this.flStall = false;
      this.wheelAngle = 0; this.brushAngle = 0; this.lidarAngle = 0;
      this.odo = { x: pose.x, y: pose.y, yaw: this.yaw };
      this.yawBias = 0;
      this.time = 0;
      this.cleaning = false;
      this.solve();
    }

    get span() { return this.j.spanL + this.j.spanR; }
    get pitch() { return Math.atan(this.t); }
    get roll() { return Math.atan(this.r); }

    contactPoints(j) {
      j = j || this.j;
      const pts = [];
      for (const s of [1, -1]) {
        const py = s > 0 ? j.spanL : -j.spanR;
        pts.push({ p: [R.BACK_WHEEL_X, py, -R.BASE_Z], g: "backWheel", s });
        pts.push({ p: [-R.BACK_WHEEL_X, py, -R.BASE_Z], g: "backWheel", s });
        pts.push({ p: [R.FRONT_WHEEL_X, py, -R.BASE_Z + j.fl], g: "frontWheel", s });
        pts.push({ p: [R.PAD_X, py, -R.BASE_Z + R.PAD_GAP - j.leg], g: "pad", s });
      }
      return pts;
    }

    xyOf(p, x, y, yaw) {
      const c = Math.cos(yaw), s = Math.sin(yaw);
      return [x + p[0] * c - p[1] * s, y + p[0] * s + p[1] * c];
    }

    // resting pose from contacts: min (z + COM_X t) s.t. z + pz + px t + py r >= ground
    solve() {
      const pts = this.contactPoints();
      const rows = pts.map(c => {
        const [wx, wy] = this.xyOf(c.p, this.x, this.y, this.yaw);
        return { a: [1, c.p[0], c.p[1]], b: this.world.groundAt(wx, wy) - c.p[2], c };
      });
      const B = 0.7;
      rows.push({ a: [0, 1, 0], b: -B, bound: 1 }, { a: [0, -1, 0], b: -B, bound: 1 },
                { a: [0, 0, 1], b: -B, bound: 1 }, { a: [0, 0, -1], b: -B, bound: 1 });
      let best = null, bestObj = Infinity;
      const n = rows.length;
      for (let i = 0; i < n; i++) for (let k = i + 1; k < n; k++) for (let m = k + 1; m < n; m++) {
        const sol = solve3([rows[i].a, rows[k].a, rows[m].a], [rows[i].b, rows[k].b, rows[m].b]);
        if (!sol) continue;
        let feas = true;
        for (let q = 0; q < n; q++) {
          const v = rows[q].a[0] * sol[0] + rows[q].a[1] * sol[1] + rows[q].a[2] * sol[2] - rows[q].b;
          if (v < -1e-7) { feas = false; break; }
        }
        if (!feas) continue;
        const obj = sol[0] + R.COM_X * sol[1];
        if (obj < bestObj - 1e-9) { bestObj = obj; best = sol; }
      }
      if (!best) return;
      [this.z, this.t, this.r] = best;
      this.tight = {};
      this.tipped = false;
      rows.forEach(row => {
        const v = row.a[0] * best[0] + row.a[1] * best[1] + row.a[2] * best[2] - row.b;
        if (v < 2e-4) {
          if (row.bound) this.tipped = true;
          else this.tight[row.c.g + row.c.s] = true;
        }
      });
    }

    // z of a body point under the current plane (small-angle form used by the solver)
    zOf(p) { return this.z + p[2] + p[0] * this.t + p[1] * this.r; }

    // which part (if any) would collide at pose (x, y, yaw) with joints j
    collision(x, y, yaw, j) {
      j = j || this.j;
      const W = this.world;
      for (const c of this.contactPoints(j)) {
        const [wx, wy] = this.xyOf(c.p, x, y, yaw);
        if (W.groundAt(wx, wy) > this.zOf(c.p) + 0.012) { this.lastHit = [wx, wy]; return c.g === "pad" ? "pad" : c.g === "frontWheel" ? "front" : "back"; }
      }
      const rect = (x0, x1, y0, y1, zb, name, nx, ny) => {
        for (let a = 0; a < nx; a++) for (let b = 0; b < ny; b++) {
          const px = x0 + (x1 - x0) * a / (nx - 1), py = y0 + (y1 - y0) * b / (ny - 1);
          const p = [px, py, zb];
          const [wx, wy] = this.xyOf(p, x, y, yaw);
          if (W.groundAt(wx, wy) > this.zOf(p) + 0.003) { this.lastHit = [wx, wy]; return name; }
        }
        return null;
      };
      for (const s of [1, -1]) {
        const py = s > 0 ? j.spanL : -j.spanR, hw = R.UNIT_W / 2;
        const zb = -R.BODY_H / 2;
        let hit = rect(-R.BACK_LEN / 2, R.BACK_LEN / 2, py - hw, py + hw, zb, "back", 4, 3);
        if (hit) return hit;
        hit = rect(R.BACK_LEN / 2 + 0.001, R.FRONT_FACE_X, py - hw, py + hw, zb + j.fl, "front", 3, 3);
        if (hit) return hit;
      }
      const cz = R.CLEANER_BOTTOM - j.head;
      return rect(j.reach - R.CLEAN / 2, j.reach + R.CLEAN / 2, j.rail - R.CLEAN / 2, j.rail + R.CLEAN / 2, cz, "cleaner", 3, 3);
    }

    jointLimits(name) {
      const inL = Math.max(0, this.j.spanL - R.UNIT_W / 2 - R.CLEAN / 2 - 0.009);
      const inR = Math.max(0, this.j.spanR - R.UNIT_W / 2 - R.CLEAN / 2 - 0.009);
      return {
        fl: [R.FL_MIN, R.RACK_TRAVEL], leg: [0, R.LEG_MAX],
        spanL: [R.SPAN_MIN / 2, R.SPAN_MAX / 2], spanR: [R.SPAN_MIN / 2, R.SPAN_MAX / 2],
        rail: [-inR, inL], reach: [-R.REACH_MAX, R.REACH_MAX], head: [0, R.HEAD_MAX],
      }[name];
    }

    step(dt) {
      this.time += dt;
      const W = this.world;
      // ---- actuators
      this.flStall = false;
      for (const k of Object.keys(this.j)) {
        const lim = this.jointLimits(k);
        let v = this.jv[k];
        let nv = Math.max(lim[0], Math.min(lim[1], this.j[k] + v * dt));
        if (k === "fl" && v < 0) {
          // front units coming down: stop where the wheels meet the tread (compliant
          // mode) - the rack's force limit is below what it takes to lift the body
          let contact = -Infinity;
          for (const s of [1, -1]) {
            const p = [R.FRONT_WHEEL_X, s > 0 ? this.j.spanL : -this.j.spanR, -R.BASE_Z];
            const [wx, wy] = this.xyOf(p, this.x, this.y, this.yaw);
            contact = Math.max(contact, W.groundAt(wx, wy) - this.zOf(p));
          }
          if (this.compliant.fl && nv <= contact) { nv = Math.max(this.j.fl, contact); this.flStall = true; }
        }
        if ((k === "spanL" || k === "spanR") && nv !== this.j[k]) {
          const jj = Object.assign({}, this.j); jj[k] = nv;
          // a unit moving sideways into the wall: side bumper
          if (this.collision(this.x, this.y, this.yaw, jj) && !this.collision(this.x, this.y, this.yaw)) { nv = this.j[k]; this.touch.side = true; }
        }
        if (k === "head" && v > 0) {
          // mop head stops on contact with the tread
          const p = [this.j.reach, this.j.rail, R.CLEANER_BOTTOM];
          const [wx, wy] = this.xyOf(p, this.x, this.y, this.yaw);
          const maxHead = this.zOf(p) - W.groundAt(wx, wy) - 0.002;
          if (nv > maxHead) nv = Math.max(this.j.head, maxHead);
        }
        this.j[k] = nv;
      }
      this.solve();

      // ---- drive (omni wheels: forward, sideways, turn)
      const surf = SCWorld.SURFACES[W.surfaceAt(this.x, this.y)] || SCWorld.SURFACES.tile;
      const slip = surf.slip * (0.6 + 0.8 * this.rng());
      const { vx, vy, wz } = this.cmd;
      this.touch.front = this.touch.back = this.touch.pad = this.touch.cleaner = false;
      if (vx || vy || wz) {
        const c = Math.cos(this.yaw), s = Math.sin(this.yaw);
        const ax = vx * (1 - slip), ay = vy * (1 - slip), aw = wz * (1 - slip * 0.5);
        const nx = this.x + (ax * c - ay * s) * dt, ny = this.y + (ax * s + ay * c) * dt, nyaw = this.yaw + aw * dt;
        const hit = this.collision(nx, ny, nyaw);
        if (hit) {
          this.touch[hit] = true;          // bumper / pad touch sensor; motors stall
          // which bumper segment closed, expressed in the body frame (known from the switch layout)
          const dx = this.lastHit[0] - nx, dy = this.lastHit[1] - ny, cy = Math.cos(nyaw), sy = Math.sin(nyaw);
          this.touchAt = [dx * cy + dy * sy, -dx * sy + dy * cy];
        } else {
          this.x = nx; this.y = ny; this.yaw = nyaw;
          // encoders count commanded wheel travel, so slip shows up as odometry drift
          const oc = Math.cos(this.odo.yaw), os = Math.sin(this.odo.yaw);
          this.odo.x += (vx * oc - vy * os) * dt;
          this.odo.y += (vx * os + vy * oc) * dt;
          this.wheelAngle += vx * dt / R.WHEEL_R;
          this.solve();
        }
      }
      // gyro integrates the true turn rate with a tiny bias
      this.yawBias += 0.00002 * gauss(this.rng) * dt;
      this.odo.yaw = this.yaw + this.yawBias;
      this.brushAngle += (this.cleaning ? 18 : 0) * dt;
      this.lidarAngle += 40 * dt;
    }

    // pose for rendering / sensors
    worldOf(p) {
      const M = rotMat(this.yaw, this.pitch, this.roll);
      const q = mv(M, p);
      return [this.x + q[0], this.y + q[1], this.z + q[2]];
    }
    rot() { return rotMat(this.yaw, this.pitch, this.roll); }
  }

  // ------------------------------------------------------------------ sensors
  class Sensors {
    constructor(robot, world, rng, opts) {
      this.robot = robot; this.world = world; this.rng = rng;
      this.opts = Object.assign({ lidarNoise: 0.004 }, opts || {});
      this.N = 160;
      this.angles = Array.from({ length: this.N }, (_, i) => -85 * Math.PI / 180 + i * (110 * Math.PI / 180) / (this.N - 1));
      this.last = {};
    }

    imu() {
      const r = this.robot, n = 0.005;
      return { pitch: r.pitch + n * gauss(this.rng), roll: r.roll + n * gauss(this.rng), yaw: r.odo.yaw };
    }

    // vertical-fan 2D LiDAR on the left front unit
    lidar(imu) {
      const r = this.robot, M = r.rot();
      const o = r.worldOf([R.LIDAR_X, r.j.spanL, R.LIDAR_Z + r.j.fl]);
      const pts = [], hits = [];
      const cp = Math.cos(imu.pitch), sp = Math.sin(imu.pitch), cr = Math.cos(imu.roll), sr = Math.sin(imu.roll);
      for (const a of this.angles) {
        const db = [Math.cos(a), 0, Math.sin(a)];
        const dw = mv(M, db);
        const h = this.world.raycast(o, dw, 2.0);
        if (!h) { pts.push([NaN, NaN]); hits.push(null); continue; }
        const surf = h.box.surface ? SCWorld.SURFACES[h.box.surface] : null;
        const rng = h.t + this.opts.lidarNoise * (surf && h.nz ? surf.noise : 1) * gauss(this.rng);
        hits.push([o[0] + dw[0] * rng, o[1] + dw[1] * rng, o[2] + dw[2] * rng]);
        // level with the IMU: undo pitch then roll (body -> gravity-aligned heading frame)
        const bx = db[0] * rng, bz = db[2] * rng;
        // Rx(roll) then Ry(-pitch) applied to body vector (bx, 0, bz)
        const y1 = -sr * bz, z1 = cr * bz;
        const x2 = cp * bx - sp * z1, z2 = sp * bx + cp * z1;
        pts.push([x2, z2]);
        void y1;
      }
      this.last.lidar = { o, hits };
      return pts;
    }

    // forward depth camera on the frame (obstacles, surface classification)
    depthCamera(cols, rows) {
      cols = cols || 44; rows = rows || 30;
      const r = this.robot, M = r.rot();
      const o = r.worldOf(R.CAM);
      const pts = [];
      const hf = 43.5 * Math.PI / 180, vf = 29 * Math.PI / 180;   // RealSense D435-like 87° x 58°
      for (let i = 0; i < rows; i++) {
        const el = R.CAM_PITCH + vf * (i / (rows - 1) * 2 - 1);
        for (let k = 0; k < cols; k++) {
          const az = hf * (k / (cols - 1) * 2 - 1);
          const db = [Math.cos(el) * Math.cos(az), Math.cos(el) * Math.sin(az), Math.sin(el)];
          const dw = mv(M, db);
          const h = this.world.raycast(o, dw, 2.5);
          if (!h) continue;
          const t = h.t * (1 + 0.004 * gauss(this.rng));
          const w = [o[0] + dw[0] * t, o[1] + dw[1] * t, o[2] + dw[2] * t];
          // to the robot's heading frame (x forward, y left, z up) at the frame origin
          const c = Math.cos(r.odo.yaw), s = Math.sin(r.odo.yaw);
          const dx = w[0] - r.x, dy = w[1] - r.y;
          pts.push({ w, x: dx * c + dy * s, y: -dx * s + dy * c, z: w[2] - r.z, surface: h.box.surface || (h.box.kind === "floor" ? this.world.P.surf1 : null), up: h.nz === 1 });
        }
      }
      this.last.depth = { o, pts };
      return pts;
    }

    // ultrasonic: narrow cone, nearest echo
    ultrasonic(bodyPos, bodyDir, maxR, spread) {
      const r = this.robot, M = r.rot();
      const o = r.worldOf(bodyPos);
      let best = maxR, end = null;
      const sp = spread == null ? 0.2 : spread;
      for (const da of [-sp, -sp / 2, 0, sp / 2, sp]) {
        const c = Math.cos(da), s = Math.sin(da);
        const db = [bodyDir[0] * c - bodyDir[1] * s, bodyDir[0] * s + bodyDir[1] * c, 0];
        const dw = mv(M, db);
        const h = this.world.raycast(o, dw, maxR);
        if (h && h.t < best) { best = h.t; }
      }
      const dw = mv(M, bodyDir);
      end = [o[0] + dw[0] * best, o[1] + dw[1] * best, o[2] + dw[2] * best];
      const d = best < maxR ? best + 0.005 * gauss(this.rng) : null;
      return { d, o, end };
    }

    ultrasonics() {
      const r = this.robot;
      const u = {
        frontL: this.ultrasonic([R.FRONT_FACE_X + 0.002, r.j.spanL, -0.05 + r.j.fl], [1, 0, 0], 2.0, 0.07),
        frontR: this.ultrasonic([R.FRONT_FACE_X + 0.002, -r.j.spanR, -0.05 + r.j.fl], [1, 0, 0], 2.0, 0.07),
        sideL: this.ultrasonic([0, r.j.spanL + R.UNIT_W / 2 + 0.002, 0], [0, 1, 0], 3.0, 0.06),
        sideR: this.ultrasonic([0, -r.j.spanR - R.UNIT_W / 2 - 0.002, 0], [0, -1, 0], 3.0, 0.06),
      };
      this.last.us = u;
      return u;
    }

    // downward IR cliff sensors at the front corners of each front unit and rear of each back unit
    cliff() {
      const r = this.robot, out = {};
      const spots = {
        FL: [R.FRONT_FACE_X - 0.01, r.j.spanL, -R.BODY_H / 2 + r.j.fl],
        FR: [R.FRONT_FACE_X - 0.01, -r.j.spanR, -R.BODY_H / 2 + r.j.fl],
        BL: [-R.BACK_LEN / 2 + 0.01, r.j.spanL, -R.BODY_H / 2],
        BR: [-R.BACK_LEN / 2 + 0.01, -r.j.spanR, -R.BODY_H / 2],
      };
      for (const [k, p] of Object.entries(spots)) {
        const o = r.worldOf(p);
        const h = this.world.raycast(o, [0, 0, -1], 0.5);
        const d = h ? h.t : 0.5;
        out[k] = { d, o, cliff: d > 0.07 };
      }
      this.last.cliff = out;
      return out;
    }
  }

  G.SCRobot = { Robot, Sensors, R, gauss, rotMat, mv };
})(typeof globalThis !== "undefined" ? globalThis : window);
