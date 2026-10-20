/*
 * Staircase world: geometry as axis-aligned solid boxes, plus queries the
 * simulator and the robot's sensors need (ground height, ray casting,
 * surface type). No rendering here, so it runs in Node for tests as well.
 *
 * Coordinates: x along flight 1 (up the stairs), y to the left of flight 1,
 * z up. Metres.
 */
(function (G) {
  "use strict";

  const SURFACES = {
    tile:   { name: "Ceramic tile",    slip: 0.03, speed: 1.0,  noise: 1.0, mode: "Vacuum + wet mop",          suction: 55, sweep: 0.25 },
    marble: { name: "Polished marble", slip: 0.07, speed: 0.7,  noise: 1.0, mode: "Vacuum + low-water mop",    suction: 50, sweep: 0.22 },
    wood:   { name: "Hardwood",        slip: 0.02, speed: 1.0,  noise: 1.0, mode: "Vacuum + damp mop",         suction: 60, sweep: 0.25 },
    carpet: { name: "Carpet",          slip: 0.05, speed: 0.85, noise: 1.8, mode: "Vacuum only, high suction", suction: 95, sweep: 0.15 },
  };

  const OBSTACLES = {
    toy:  { name: "Toy block", sx: 0.06, sy: 0.06, sz: 0.04 },
    shoe: { name: "Shoe",      sx: 0.27, sy: 0.10, sz: 0.10 },
    box:  { name: "Parcel",    sx: 0.22, sy: 0.22, sz: 0.20 },
  };

  const DEFAULTS = {
    layout: "uturn", h: 0.17, d: 0.34, W: 0.9, n1: 5, n2: 4, L: 1.1, jitter: 0, seed: 7,
    surf1: "wood", surfL: "tile", surf2: "carpet", obstacles: [],
  };

  const WALL_T = 0.06;     // wall thickness
  const GAP = 0.12;        // spine wall between the two flights
  const PARAPET = 0.35;    // low wall height above each tread (handrail sits above it)
  const TALL = 1.0;        // stairwell wall at the end of the landing
  const CLEAR = 0.03;      // robot keeps this gap to the walls
  const UNIT_W = 0.15;

  function mulberry32(a) {
    return function () {
      a |= 0; a = (a + 0x6d2b79f5) | 0;
      let t = Math.imul(a ^ (a >>> 15), 1 | a);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  class World {
    constructor(params) {
      const P = (this.P = Object.assign({}, DEFAULTS, params));
      const rnd = mulberry32(P.seed || 1);
      const W = P.W, d = P.d, uturn = P.layout === "uturn";
      this.boxes = [];
      this.treads = [];        // every walkable level: floor, treads, landing, top floor
      this.walls = [];
      const nTot = P.n1 + (uturn ? P.n2 : 0);
      const hs = [];
      for (let i = 0; i < nTot; i++) hs.push(P.h + (P.jitter ? (rnd() * 2 - 1) * P.jitter : 0));
      this.heights = hs;
      const X0 = (P.n1 - 1) * d;
      this.X0 = X0;
      this.width = W;
      this.totalW = uturn ? 2 * W + GAP : W;
      this.uturn = uturn;
      const add = (min, max, kind, extra) => {
        const b = Object.assign({ min, max, kind, id: this.boxes.length }, extra || {});
        this.boxes.push(b);
        return b;
      };

      // ground floor in front of flight 1
      this.treads.push({ flight: 1, step: 0, kind: "floor", x0: -2.4, x1: 0, y0: 0, y1: W, z: 0, surface: P.surf1, dir: 1 });

      // flight 1 (rises along +x)
      let z = 0;
      for (let i = 1; i <= P.n1; i++) {
        z += hs[i - 1];
        const x0 = (i - 1) * d;
        const last = i === P.n1;
        let x1 = i * d, y1 = W, kind = "tread", surface = P.surf1;
        if (last && uturn) { x1 = X0 + P.L; y1 = 2 * W + GAP; kind = "landing"; surface = P.surfL; }
        else if (last) { x1 = x0 + 1.5; kind = "top"; }
        const b = add([x0, 0, 0], [x1, y1, z], kind, { surface });
        this.treads.push({ flight: 1, step: i, kind, x0, x1, y0: 0, y1, z, surface, dir: 1,
          riser: x0, h: hs[i - 1], box: b });
      }
      this.zLanding = uturn ? z : null;

      // flight 2 (rises along -x, beside flight 1)
      if (uturn) {
        const yA = W + GAP, yB = 2 * W + GAP;
        for (let j = 1; j <= P.n2; j++) {
          z += hs[P.n1 + j - 1];
          const xr = X0 - (j - 1) * d;
          const last = j === P.n2;
          const x0 = last ? xr - 1.6 : xr - d;
          const kind = last ? "top" : "tread";
          const b = add([x0, yA, 0], [xr, yB, z], kind, { surface: P.surf2 });
          this.treads.push({ flight: 2, step: j, kind, x0, x1: xr, y0: yA, y1: yB, z, surface: P.surf2, dir: -1,
            riser: xr, h: hs[P.n1 + j - 1], box: b });
        }
      }
      this.zTop = z;

      // ---- walls (low parapets that follow the steps; tall wall behind the landing)
      const wall = (x0, x1, y0, y1, top, kind) => this.walls.push(add([x0, y0, 0], [x1, y1, top], "wall", { wallKind: kind || "parapet" }));
      const f1 = this.treads.filter(t => t.flight === 1);
      // outer wall of flight 1 (y < 0)
      f1.forEach(t => wall(t.x0, t.x1, -WALL_T, 0, t.z + PARAPET));
      if (uturn) {
        const f2 = this.treads.filter(t => t.flight === 2);
        // spine wall between the flights follows the higher flight (flight 2)
        let xEnd = X0;
        f2.forEach(t => { wall(t.x0, t.x1, W, W + GAP, t.z + PARAPET, "spine"); xEnd = Math.min(xEnd, t.x0); });
        if (xEnd > -2.4) wall(-2.4, xEnd, W, W + GAP, this.zTop + PARAPET, "spine");
        // outer wall of flight 2
        f2.forEach(t => wall(t.x0, t.x1, 2 * W + GAP, 2 * W + GAP + WALL_T, t.z + PARAPET));
        // landing: outer side walls and the tall stairwell wall at its end
        wall(X0, X0 + P.L, 2 * W + GAP, 2 * W + GAP + WALL_T, this.zLanding + PARAPET);
        wall(X0 + P.L, X0 + P.L + WALL_T, -WALL_T, 2 * W + GAP + WALL_T, this.zLanding + TALL, "tall");
      } else {
        f1.forEach(t => wall(t.x0, t.x1, W, W + WALL_T, t.z + PARAPET));
      }

      // ---- obstacles
      this.obstacles = [];
      (P.obstacles || []).forEach(o => this.addObstacle(o));
      this.buildDirt();
    }

    flightTreads(f) { return this.treads.filter(t => t.flight === f); }

    obstaclePlacement(o) {
      const T = OBSTACLES[o.type] || OBSTACLES.toy;
      const W = this.width;
      let cx, cy, z;
      if (o.loc === "landing" && this.uturn) {
        const t = this.treads.find(t => t.kind === "landing");
        cx = this.X0 + (o.fx == null ? 0.5 : o.fx) * this.P.L;
        cy = (o.fy == null ? 0.5 : o.fy) * this.totalW;
        z = t.z;
      } else {
        const flight = o.loc === "f2" && this.uturn ? 2 : 1;
        const list = this.treads.filter(t => t.flight === flight && t.kind === "tread");
        if (!list.length) return null;
        const t = list[Math.max(0, Math.min(list.length - 1, (o.step || 1) - 1))];
        const fx = o.fx == null ? 0.5 : o.fx;
        cx = t.x0 + Math.max(T.sx / 2 + 0.02, Math.min(t.x1 - t.x0 - T.sx / 2 - 0.02, fx * (t.x1 - t.x0)));
        // fy: 0 = right wall, 1 = left wall, as seen while climbing that flight
        const fy = o.fy == null ? 0.5 : o.fy;
        const yRight = t.dir > 0 ? t.y0 : t.y1, yLeft = t.dir > 0 ? t.y1 : t.y0;
        cy = yRight + (yLeft - yRight) * fy;
        cy = Math.max(t.y0 + T.sy / 2 + 0.01, Math.min(t.y1 - T.sy / 2 - 0.01, cy));
        z = t.z;
      }
      return { min: [cx - T.sx / 2, cy - T.sy / 2, z], max: [cx + T.sx / 2, cy + T.sy / 2, z + T.sz], T };
    }

    addObstacle(o) {
      const pl = this.obstaclePlacement(o);
      if (!pl) return null;
      const b = Object.assign({ min: pl.min, max: pl.max, kind: "obstacle", id: this.boxes.length, obs: o, label: pl.T.name });
      this.boxes.push(b);
      this.obstacles.push(b);
      this.refreshDirt();
      return b;
    }

    removeObstacle(o) {
      this.boxes = this.boxes.filter(b => b.obs !== o);
      this.obstacles = this.obstacles.filter(b => b.obs !== o);
      this.boxes.forEach((b, i) => (b.id = i));
      this.refreshDirt();
    }

    // dirt only in the band between the two drive units, which the central
    // cleaning module can reach; nothing under the units' tracks
    buildDirt() {
      const cell = 0.04;
      this.dirt = [];
      this.treads.forEach((t, ti) => {
        if (t.kind !== "tread") return;
        const yLo = t.y0 + CLEAR + UNIT_W + 0.01, yHi = t.y1 - CLEAR - UNIT_W - 0.01;
        for (let x = t.x0 + cell / 2; x < t.x1 - 0.01; x += cell) {
          for (let y = yLo + cell / 2; y < yHi; y += cell) {
            this.dirt.push({ x, y, z: t.z, tread: ti, clean: false, at: -1 });
          }
        }
      });
      this.refreshDirt();
    }

    // cells under an obstacle cannot be cleaned and do not count towards coverage
    refreshDirt() {
      if (!this.dirt) return;
      this.dirt.forEach(c => {
        c.blocked = this.obstacles.some(o => c.x > o.min[0] - 0.02 && c.x < o.max[0] + 0.02 && c.y > o.min[1] - 0.02 && c.y < o.max[1] + 0.02);
      });
    }

    groundAt(x, y) {
      let g = 0;
      const B = this.boxes;
      for (let i = 0; i < B.length; i++) {
        const b = B[i];
        if (x >= b.min[0] && x <= b.max[0] && y >= b.min[1] && y <= b.max[1] && b.max[2] > g) g = b.max[2];
      }
      return g;
    }

    boxAt(x, y) {
      let g = -1, best = null;
      for (const b of this.boxes) {
        if (x >= b.min[0] && x <= b.max[0] && y >= b.min[1] && y <= b.max[1] && b.max[2] > g) { g = b.max[2]; best = b; }
      }
      return best;
    }

    surfaceAt(x, y) {
      const b = this.boxAt(x, y);
      if (b && b.surface) return b.surface;
      // ground floor before flight 1 shares flight 1's finish
      return this.P.surf1;
    }

    // returns {t, box, nz} or null
    raycast(o, dir, maxT) {
      let best = maxT, hit = null, nz = 0;
      if (dir[2] < -1e-9) {
        const t = -o[2] / dir[2];
        if (t > 0 && t < best) { best = t; hit = { kind: "floor", surface: this.P.surf1 }; nz = 1; }
      }
      const B = this.boxes;
      for (let i = 0; i < B.length; i++) {
        const b = B[i];
        let t0 = 0, t1 = best, axis = -1;
        let ok = true;
        for (let a = 0; a < 3; a++) {
          const da = dir[a], oa = o[a];
          if (Math.abs(da) < 1e-12) {
            if (oa < b.min[a] || oa > b.max[a]) { ok = false; break; }
          } else {
            let ta = (b.min[a] - oa) / da, tb = (b.max[a] - oa) / da;
            if (ta > tb) { const s = ta; ta = tb; tb = s; }
            if (ta > t0) { t0 = ta; axis = a; }
            if (tb < t1) t1 = tb;
            if (t0 > t1) { ok = false; break; }
          }
        }
        if (ok && t0 > 1e-6 && t0 < best) { best = t0; hit = b; nz = axis === 2 ? 1 : 0; }
      }
      return hit ? { t: best, box: hit, nz } : null;
    }
  }

  G.SCWorld = { World, SURFACES, OBSTACLES, DEFAULTS, CLEAR, UNIT_W, GAP, WALL_T, PARAPET, TALL };
})(typeof globalThis !== "undefined" ? globalThis : window);
