/*
 * Three.js rendering: staircase, robot model and sensor overlays.
 * Reads simulation state; never changes it.
 */
(function (G) {
  "use strict";
  const { R } = G.SCRobot;
  THREE.Object3D.DefaultUp.set(0, 0, 1);

  // ------------------------------------------------------------------ textures
  function canvasTex(size, draw, repeat) {
    const c = document.createElement("canvas");
    c.width = c.height = size;
    const g = c.getContext("2d");
    draw(g, size);
    const t = new THREE.CanvasTexture(c);
    t.wrapS = t.wrapT = THREE.RepeatWrapping;
    t.anisotropy = 4;
    t.encoding = THREE.sRGBEncoding;
    if (repeat) t.repeat.set(repeat, repeat);
    return t;
  }
  function rnd(seed) { let s = seed; return () => (s = (s * 16807) % 2147483647) / 2147483647; }
  function noise(g, n, alpha, colors, seed) {
    const r = rnd(seed || 7);
    for (let i = 0; i < 9000; i++) { g.fillStyle = colors[Math.floor(r() * colors.length)]; g.globalAlpha = alpha * r(); g.fillRect(r() * n, r() * n, 1 + r() * 2, 1 + r() * 2); }
    g.globalAlpha = 1;
  }
  const TEX = {};
  function makeTextures() {
    TEX.wood = canvasTex(512, (g, n) => {
      g.fillStyle = "#9b6a3d"; g.fillRect(0, 0, n, n);
      const r = rnd(3);
      for (let p = 0; p < 4; p++) {
        const y0 = p * n / 4;
        g.fillStyle = ["#9b6a3d", "#a5744a", "#8f6036", "#a06d40"][p]; g.fillRect(0, y0, n, n / 4);
        for (let k = 0; k < 26; k++) {
          g.strokeStyle = r() > 0.5 ? "rgba(70,40,15,0.25)" : "rgba(190,140,90,0.18)";
          g.lineWidth = 1 + r() * 1.5; g.beginPath();
          const yy = y0 + r() * n / 4;
          g.moveTo(0, yy);
          for (let x = 0; x <= n; x += 16) g.lineTo(x, yy + Math.sin(x / 60 + k) * 3 * r());
          g.stroke();
        }
        g.fillStyle = "rgba(40,20,5,0.55)"; g.fillRect(0, y0, n, 2);
      }
    });
    TEX.tile = canvasTex(512, (g, n) => {
      g.fillStyle = "#bdb7ad"; g.fillRect(0, 0, n, n);
      const r = rnd(5);
      for (let a = 0; a < 2; a++) for (let b = 0; b < 2; b++) {
        const v = 226 + Math.floor(r() * 12);
        g.fillStyle = `rgb(${v},${v - 3},${v - 9})`; g.fillRect(a * n / 2 + 3, b * n / 2 + 3, n / 2 - 6, n / 2 - 6);
      }
      noise(g, n, 0.12, ["#b0aa9f", "#ffffff"], 11);
    });
    TEX.marble = canvasTex(512, (g, n) => {
      g.fillStyle = "#efeeea"; g.fillRect(0, 0, n, n);
      const r = rnd(9);
      for (let k = 0; k < 14; k++) {
        g.strokeStyle = `rgba(120,120,128,${0.15 + r() * 0.35})`; g.lineWidth = 0.6 + r() * 2;
        g.beginPath(); let x = r() * n, y = r() * n; g.moveTo(x, y);
        for (let s = 0; s < 40; s++) { x += (r() - 0.3) * 22; y += (r() - 0.5) * 22; g.lineTo(x, y); }
        g.stroke();
      }
      noise(g, n, 0.08, ["#c9c8c4", "#ffffff"], 2);
    });
    TEX.carpet = canvasTex(256, (g, n) => {
      g.fillStyle = "#6e2230"; g.fillRect(0, 0, n, n);
      noise(g, n, 0.5, ["#4e1520", "#8c3445", "#5a1a27", "#a14a58"], 4);
    });
    TEX.plaster = canvasTex(256, (g, n) => { g.fillStyle = "#e7e3dc"; g.fillRect(0, 0, n, n); noise(g, n, 0.12, ["#d6d1c8", "#f6f3ee"], 8); });
    TEX.floor = canvasTex(512, (g, n) => {
      g.fillStyle = "#9c968b"; g.fillRect(0, 0, n, n);
      const r = rnd(12);
      for (let a = 0; a < 2; a++) for (let b = 0; b < 2; b++) { const v = 196 + Math.floor(r() * 10); g.fillStyle = `rgb(${v},${v - 4},${v - 12})`; g.fillRect(a * n / 2 + 2, b * n / 2 + 2, n / 2 - 4, n / 2 - 4); }
      noise(g, n, 0.1, ["#8f897f", "#ffffff"], 13);
    });
    TEX.fabric = canvasTex(128, (g, n) => { g.fillStyle = "#8fc3e6"; g.fillRect(0, 0, n, n); noise(g, n, 0.4, ["#6aa6d0", "#c4e2f5"], 6); });
  }
  const SURF_TEX = { wood: { tex: "wood", size: 0.9 }, tile: { tex: "tile", size: 0.6 }, marble: { tex: "marble", size: 0.8 }, carpet: { tex: "carpet", size: 0.35 } };

  // world-space UVs so textures keep their real size on every face
  function worldUV(geo, origin, size) {
    const pos = geo.attributes.position, nor = geo.attributes.normal, uv = geo.attributes.uv;
    for (let i = 0; i < pos.count; i++) {
      const x = pos.getX(i) + origin[0], y = pos.getY(i) + origin[1], z = pos.getZ(i) + origin[2];
      const nx = Math.abs(nor.getX(i)), ny = Math.abs(nor.getY(i));
      if (nx > 0.5) uv.setXY(i, y / size, z / size);
      else if (ny > 0.5) uv.setXY(i, x / size, z / size);
      else uv.setXY(i, x / size, y / size);
    }
    uv.needsUpdate = true;
  }

  function roundedBox(w, d, h, r, mat) {
    const s = new THREE.Shape(), x = -w / 2 + r, y = -d / 2 + r, W = w - 2 * r, D = d - 2 * r;
    s.moveTo(x, y - r); s.lineTo(x + W, y - r); s.absarc(x + W, y, r, -Math.PI / 2, 0); s.lineTo(x + W + r, y + D);
    s.absarc(x + W, y + D, r, 0, Math.PI / 2); s.lineTo(x, y + D + r); s.absarc(x, y + D, r, Math.PI / 2, Math.PI);
    s.lineTo(x - r, y); s.absarc(x, y, r, Math.PI, 1.5 * Math.PI);
    const bev = Math.min(r * 0.7, 0.006);
    const geo = new THREE.ExtrudeGeometry(s, { depth: h - 2 * bev, bevelEnabled: true, bevelThickness: bev, bevelSize: bev, bevelSegments: 3, curveSegments: 6 });
    geo.translate(0, 0, -(h - 2 * bev) / 2);
    const m = new THREE.Mesh(geo, mat);
    m.castShadow = m.receiveShadow = true;
    return m;
  }
  const cyl = (r, l, mat, seg) => { const g = new THREE.CylinderGeometry(r, r, l, seg || 20); const m = new THREE.Mesh(g, mat); m.castShadow = true; return m; };
  const box = (x, y, z, mat) => { const m = new THREE.Mesh(new THREE.BoxGeometry(x, y, z), mat); m.castShadow = true; m.receiveShadow = true; return m; };

  // ------------------------------------------------------------------ materials
  const M = {};
  function makeMaterials() {
    const std = (c, r, m, extra) => new THREE.MeshStandardMaterial(Object.assign({ color: c, roughness: r, metalness: m }, extra || {}));
    Object.assign(M, {
      shellDark: std(0x2a2e35, 0.5, 0.15), shellLight: std(0xd9dde2, 0.45, 0.05), top: std(0x3a3f47, 0.6, 0.1),
      blue: std(0x2347c9, 0.35, 0.2, { emissive: 0x0a1650, emissiveIntensity: 0.4 }), red: std(0xc8302a, 0.35, 0.15, { emissive: 0x40100c, emissiveIntensity: 0.4 }),
      alu: std(0xc2c7cc, 0.28, 0.9), chrome: std(0xe2e6ea, 0.12, 1.0), anod: std(0x31353c, 0.35, 0.7), steel: std(0x9aa0a6, 0.3, 0.85),
      rubber: std(0x15171a, 0.85, 0.0), black: std(0x0b0c0e, 0.3, 0.2), glass: std(0x0a0d12, 0.05, 0.6, { transparent: true, opacity: 0.92 }),
      lens: std(0x1b2b4a, 0.05, 0.9), orange: std(0xf08a24, 0.4, 0.2, { emissive: 0x6a2a00, emissiveIntensity: 0.6 }),
      white: std(0xf1f2f3, 0.4, 0.05), bin: new THREE.MeshPhysicalMaterial({ color: 0xd8e3ea, roughness: 0.05, metalness: 0, transparent: true, opacity: 0.35 }),
      tank: new THREE.MeshPhysicalMaterial({ color: 0x4aa3df, roughness: 0.1, transparent: true, opacity: 0.55 }),
      fabric: std(0xffffff, 0.95, 0, { map: TEX.fabric }), bristle: std(0x2a6fb0, 0.7, 0),
      led: new THREE.MeshBasicMaterial({ color: 0x35d07f }), mesh: std(0x8f969d, 0.6, 0.7),
      dust: std(0x7b5a3a, 0.9, 0),
    });
  }

  // ------------------------------------------------------------------ robot model
  function omniWheel() {
    const g = new THREE.Group();
    const hub = cyl(0.021, 0.026, M.alu, 24); g.add(hub);                              // CylinderGeometry axis = Y = axle
    const plateA = cyl(0.028, 0.003, M.anod, 24); plateA.position.y = 0.006; g.add(plateA);
    const plateB = plateA.clone(); plateB.position.y = -0.006; g.add(plateB);
    const rollerGeo = new THREE.CylinderGeometry(0.0065, 0.0065, 0.016, 10);
    for (let row = 0; row < 2; row++) {
      for (let k = 0; k < 9; k++) {
        const a = (k + row * 0.5) / 9 * Math.PI * 2;
        const rl = new THREE.Mesh(rollerGeo, M.rubber);
        rl.castShadow = true;
        rl.position.set(Math.cos(a) * 0.0285, row ? 0.0065 : -0.0065, Math.sin(a) * 0.0285);
        // roller axis tangent to the rim (in the wheel plane x-z)
        rl.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), new THREE.Vector3(-Math.sin(a), 0, Math.cos(a)));
        g.add(rl);
      }
    }
    return g;
  }

  function rack(len, teethSide) {
    const g = new THREE.Group();
    const bar = box(0.012, 0.016, len, M.alu); g.add(bar);
    const n = Math.floor(len / 0.008);
    const tg = new THREE.BoxGeometry(0.004, 0.012, 0.004);
    const im = new THREE.InstancedMesh(tg, M.steel, n);
    const m4 = new THREE.Matrix4();
    for (let i = 0; i < n; i++) { m4.makeTranslation(teethSide * 0.008, 0, -len / 2 + 0.004 + i * 0.008); im.setMatrixAt(i, m4); }
    g.add(im);
    return g;
  }

  function gear(r) {
    const g = new THREE.Group();
    const d = cyl(r, 0.01, M.steel, 24); g.add(d);
    const n = 14, tg = new THREE.BoxGeometry(0.005, 0.01, 0.005);
    for (let i = 0; i < n; i++) {
      const a = i / n * Math.PI * 2, t = new THREE.Mesh(tg, M.steel);
      t.position.set(Math.cos(a) * (r + 0.002), 0, Math.sin(a) * (r + 0.002)); t.rotation.y = -a; g.add(t);
    }
    const hub = cyl(r * 0.35, 0.014, M.anod, 12); g.add(hub);
    return g;
  }

  function ultrasonicPod() {
    const g = new THREE.Group();
    const ring = cyl(0.009, 0.006, M.alu, 18); ring.rotation.z = Math.PI / 2; g.add(ring);
    const mesh = cyl(0.0075, 0.007, M.mesh, 18); mesh.rotation.z = Math.PI / 2; mesh.position.x = 0.0005; g.add(mesh);
    return g;
  }

  class RobotView {
    constructor(scene) {
      this.root = new THREE.Group();
      scene.add(this.root);
      this.sides = {};
      const H = R.BODY_H;
      for (const s of [1, -1]) {
        const side = {};
        // ---- back unit (blue): battery, electronics, rods housing
        const back = new THREE.Group();
        const shell = roundedBox(R.BACK_LEN, R.UNIT_W, H, 0.018, M.shellDark); back.add(shell);
        const band = roundedBox(R.BACK_LEN + 0.004, R.UNIT_W + 0.004, 0.024, 0.02, M.blue); band.position.z = 0.085; back.add(band);
        const lid = roundedBox(R.BACK_LEN - 0.02, R.UNIT_W - 0.02, 0.01, 0.012, M.top); lid.position.z = H / 2 + 0.002; back.add(lid);
        for (let i = 0; i < 6; i++) { const v = box(0.11, 0.003, 0.005, M.black); v.position.set(-0.01, s * (R.UNIT_W / 2 + 0.001), -0.06 + i * 0.016); back.add(v); }
        const ledBar = box(0.004, 0.06, 0.008, M.led.clone()); ledBar.position.set(-R.BACK_LEN / 2 - 0.002, 0, 0.11); back.add(ledBar); side.led = ledBar;
        const sideUS = ultrasonicPod(); sideUS.rotation.z = s * Math.PI / 2; sideUS.position.set(0, s * (R.UNIT_W / 2 + 0.003), 0); back.add(sideUS);
        const pinHousing = box(0.018, 0.05, 0.05, M.anod); pinHousing.position.set(-R.BACK_LEN / 2 - 0.009, 0, -0.04); back.add(pinHousing);
        // front-lift rack (fixed to the back unit, inner side) - the front unit climbs it
        const fr = rack(H + R.RACK_TRAVEL, 1); fr.position.set(R.BACK_LEN / 2 - 0.01, -s * (R.UNIT_W / 2 + 0.01), R.RACK_TRAVEL / 2); back.add(fr);
        // wheels
        side.wheels = [];
        for (const wx of [-R.BACK_WHEEL_X, R.BACK_WHEEL_X]) {
          const w = omniWheel(); w.position.set(wx, s * (R.UNIT_W / 2 - 0.02), -H / 2 + R.WHEEL_R - R.CLEARANCE); back.add(w); side.wheels.push(w);
        }
        this.root.add(back); side.back = back;

        // ---- front unit (red): sensors, climbs first
        const front = new THREE.Group();
        const fshell = roundedBox(R.FRONT_LEN, R.UNIT_W, H, 0.016, M.shellLight); front.add(fshell);
        const fband = roundedBox(R.FRONT_LEN + 0.004, R.UNIT_W + 0.004, 0.024, 0.018, M.red); fband.position.z = 0.085; front.add(fband);
        const visor = roundedBox(0.012, R.UNIT_W - 0.03, 0.17, 0.005, M.glass); visor.position.set(R.FRONT_LEN / 2 + 0.002, 0, 0.0); front.add(visor);
        const usA = ultrasonicPod(); usA.position.set(R.FRONT_LEN / 2 + 0.008, 0.03, -0.05); front.add(usA);
        const usB = ultrasonicPod(); usB.position.set(R.FRONT_LEN / 2 + 0.008, -0.03, -0.05); front.add(usB);
        for (const yy of [0.045, -0.045]) { const ir = box(0.012, 0.01, 0.004, M.red); ir.position.set(R.FRONT_LEN / 2 - 0.012, yy, -H / 2 - 0.002); front.add(ir); }
        if (s > 0) {
          // 2D LiDAR (vertical fan) on the left front unit
          const lid2 = new THREE.Group();
          const housing = cyl(0.022, 0.03, M.black, 24); housing.rotation.x = Math.PI / 2; lid2.add(housing);
          const win = cyl(0.0225, 0.012, M.orange, 24); win.rotation.x = Math.PI / 2; lid2.add(win);
          const spin = box(0.004, 0.034, 0.03, M.anod); lid2.add(spin); side.lidarSpin = spin;
          lid2.position.set(R.FRONT_LEN / 2 + 0.016, 0, R.LIDAR_Z);
          front.add(lid2);
        } else {
          const cam = cyl(0.01, 0.01, M.lens, 18); cam.rotation.z = Math.PI / 2; cam.position.set(R.FRONT_LEN / 2 + 0.009, 0, R.LIDAR_Z); front.add(cam);
        }
        const pin = gear(0.012); pin.position.set(-R.FRONT_LEN / 2 + 0.002, -s * (R.UNIT_W / 2 + 0.01), 0); front.add(pin); side.pinion = pin;
        const fw = omniWheel(); fw.position.set(0, s * (R.UNIT_W / 2 - 0.02), -H / 2 + R.WHEEL_R - R.CLEARANCE); front.add(fw); side.wheels.push(fw);
        this.root.add(front); side.front = front;

        // ---- rear rack leg with rubber foot
        const leg = new THREE.Group();
        const lr = rack(0.34, -1); lr.position.z = 0.17; leg.add(lr);
        const foot = cyl(0.016, 0.012, M.rubber, 20); foot.rotation.x = Math.PI / 2; foot.position.z = 0.0; leg.add(foot);
        this.root.add(leg); side.leg = leg;
        this.sides[s] = side;
      }
      // ---- telescopic rods
      this.rods = [];
      for (const z of [0.08, 0.02]) {
        const outerL = cyl(0.016, 0.16, M.anod), outerR = cyl(0.016, 0.16, M.anod), inner = cyl(0.011, 1, M.chrome);
        [outerL, outerR, inner].forEach(m => this.root.add(m));
        this.rods.push({ z, outerL, outerR, inner });
      }
      // depth camera bar on the frame front
      const camBar = new THREE.Group();
      const cb = roundedBox(0.026, 0.09, 0.026, 0.006, M.black); camBar.add(cb);
      for (const yy of [-0.025, 0.025]) { const l = cyl(0.006, 0.006, M.lens, 16); l.rotation.z = Math.PI / 2; l.position.set(0.013, yy, 0); camBar.add(l); }
      const ir = cyl(0.004, 0.006, M.red, 12); ir.rotation.z = Math.PI / 2; ir.position.set(0.013, 0, 0); camBar.add(ir);
      const bracket = box(0.012, 0.012, 0.06, M.anod); bracket.position.set(-0.01, 0, -0.035); camBar.add(bracket);
      this.camBar = camBar; this.root.add(camBar);

      // ---- cleaning unit
      const cu = new THREE.Group();
      const body = roundedBox(R.CLEAN, R.CLEAN, 0.12, 0.02, M.white); body.position.z = 0.0; cu.add(body);
      const bin = roundedBox(0.07, 0.12, 0.09, 0.012, M.bin); bin.position.set(0.045, 0, 0.005); cu.add(bin);
      const dust = new THREE.InstancedMesh(new THREE.BoxGeometry(0.004, 0.004, 0.004), M.dust, 40);
      const m4 = new THREE.Matrix4(); const rr = rnd(21);
      for (let i = 0; i < 40; i++) { m4.makeTranslation(0.045 + (rr() - 0.5) * 0.05, (rr() - 0.5) * 0.09, -0.035 + rr() * 0.02); dust.setMatrixAt(i, m4); }
      cu.add(dust); this.dust = dust;
      const tank = roundedBox(0.06, 0.12, 0.035, 0.01, M.tank); tank.position.set(-0.035, 0, 0.07); cu.add(tank);
      const cap = cyl(0.01, 0.012, M.anod, 14); cap.rotation.x = Math.PI / 2; cap.position.set(-0.035, 0, 0.092); cu.add(cap);
      for (const yy of [-0.06, 0.06]) { const sl = box(0.05, 0.02, 0.022, M.anod); sl.position.set(0, yy, 0.05); cu.add(sl); }
      const status = box(0.004, 0.05, 0.006, M.led.clone()); status.position.set(R.CLEAN / 2 + 0.001, 0, 0.04); cu.add(status); this.cleanLed = status;
      const head = new THREE.Group();
      const pad = box(R.HEAD_D, R.HEAD_W, 0.014, M.fabric); pad.position.z = -0.007; head.add(pad); this.mopPad = pad;
      const plate = box(R.HEAD_D - 0.01, R.HEAD_W - 0.01, 0.012, M.anod); plate.position.z = 0.006; head.add(plate);
      const slot = box(0.012, R.HEAD_W - 0.03, 0.006, M.black); slot.position.set(R.HEAD_D / 2 - 0.02, 0, -0.012); head.add(slot);
      this.brushes = [];
      for (const yy of [-1, 1]) {
        const br = new THREE.Group();
        const disk = cyl(0.012, 0.006, M.anod, 16); br.add(disk);
        for (let k = 0; k < 8; k++) {
          const b = cyl(0.0012, 0.04, M.bristle, 4); const a = k / 8 * Math.PI * 2;
          b.rotation.z = Math.PI / 2; b.rotation.y = a; b.position.set(Math.cos(a) * 0.02, 0, -Math.sin(a) * 0.02); br.add(b);
        }
        br.rotation.x = Math.PI / 2; br.position.set(R.HEAD_D / 2 - 0.01, yy * (R.HEAD_W / 2 - 0.005), -0.012);
        head.add(br); this.brushes.push(br);
      }
      const mast = cyl(0.008, 0.06, M.chrome, 12); mast.rotation.x = Math.PI / 2; mast.position.z = 0.04; head.add(mast);
      cu.add(head); this.head = head; this.cleaner = cu;
      this.root.add(cu);
      this.root.traverse(o => { if (o.isMesh) o.castShadow = true; });
    }

    update(robot, state) {
      const r = robot, j = r.j;
      // pose: rotation Rz(yaw) Ry(-pitch) Rx(roll)
      const m = SCRobot.rotMat(r.yaw, r.pitch, r.roll);
      const mat = new THREE.Matrix4().set(m[0][0], m[0][1], m[0][2], r.x, m[1][0], m[1][1], m[1][2], r.y, m[2][0], m[2][1], m[2][2], r.z, 0, 0, 0, 1);
      this.root.matrixAutoUpdate = false; this.root.matrix.copy(mat); this.root.matrixWorldNeedsUpdate = true;
      for (const s of [1, -1]) {
        const sd = this.sides[s], py = s > 0 ? j.spanL : -j.spanR;
        sd.back.position.set(0, py, 0);
        sd.front.position.set(R.FRONT_WHEEL_X, py, j.fl);
        sd.leg.position.set(R.PAD_X, py, -R.BASE_Z + R.PAD_GAP + 0.004 - j.leg);
        sd.pinion.rotation.y = -j.fl / 0.012;
        sd.wheels.forEach(w => (w.rotation.y = r.wheelAngle));
        if (sd.lidarSpin) sd.lidarSpin.rotation.y = r.lidarAngle;
        const col = state === "SAFE_ABORT" ? 0xff3b30 : state === "OBSTACLE_WAIT" ? 0xffb020 : state === "DONE" ? 0x3aa0ff : 0x35d07f;
        sd.led.material.color.setHex(col);
      }
      const inner = j.spanL + j.spanR - R.UNIT_W;
      this.rods.forEach(rd => {
        rd.outerL.position.set(0, j.spanL - R.UNIT_W / 2 - 0.08, rd.z);
        rd.outerR.position.set(0, -j.spanR + R.UNIT_W / 2 + 0.08, rd.z);
        rd.inner.scale.y = Math.max(0.05, inner);
        rd.inner.position.set(0, (j.spanL - j.spanR) / 2, rd.z);
      });
      this.camBar.position.set(R.CAM[0], 0, R.CAM[2]);
      this.cleaner.position.set(j.reach, j.rail, 0.0);
      this.head.position.set(0, 0, -0.06 - 0.012 - j.head + 0.012);
      const carpet = state && state.carpet;
      this.mopPad.visible = !carpet;
      this.brushes.forEach((b, i) => (b.rotation.y = (i ? 1 : -1) * r.brushAngle));
      this.cleanLed.material.color.setHex(r.cleaning ? 0x35a8ff : 0x35d07f);
    }
  }

  // ------------------------------------------------------------------ stairs
  function buildStairs(world) {
    const g = new THREE.Group();
    const surfMat = {};
    const matFor = (surf, riser) => {
      const k = surf + (riser ? "R" : "");
      if (!surfMat[k]) {
        const spec = SURF_TEX[surf] || SURF_TEX.tile;
        surfMat[k] = new THREE.MeshStandardMaterial({ map: TEX[spec.tex], roughness: surf === "marble" ? 0.18 : surf === "carpet" ? 0.95 : 0.55,
          metalness: 0, color: riser ? 0xd8d4cc : 0xffffff });
      }
      return surfMat[k];
    };
    const plaster = new THREE.MeshStandardMaterial({ map: TEX.plaster, roughness: 0.9, color: 0xffffff });
    const plasterTall = new THREE.MeshStandardMaterial({ map: TEX.plaster, roughness: 0.9, color: 0xffffff, transparent: true, opacity: 0.35, depthWrite: false });
    world.boxes.forEach(b => {
      if (b.kind === "obstacle") return;
      const sx = b.max[0] - b.min[0], sy = b.max[1] - b.min[1], sz = b.max[2] - b.min[2];
      const geo = new THREE.BoxGeometry(sx, sy, sz);
      const c = [(b.min[0] + b.max[0]) / 2, (b.min[1] + b.max[1]) / 2, (b.min[2] + b.max[2]) / 2];
      let mesh;
      if (b.kind === "wall") {
        worldUV(geo, c, 1.2);
        mesh = new THREE.Mesh(geo, b.wallKind === "tall" ? plasterTall : plaster);
      } else {
        const spec = SURF_TEX[b.surface] || SURF_TEX.tile;
        worldUV(geo, c, spec.size);
        const side = matFor(b.surface, true), top = matFor(b.surface, false);
        mesh = new THREE.Mesh(geo, [side, side, side, side, top, side]);
        // rounded nosing on the riser edge
        const tr = world.treads.find(t => t.box === b);
        if (tr && tr.step > 0) {
          const nose = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, tr.y1 - tr.y0, 12), top);
          nose.position.set(tr.riser + (tr.dir > 0 ? 0.004 : -0.004), (tr.y0 + tr.y1) / 2, tr.z - 0.012);
          nose.receiveShadow = true; g.add(nose);
        }
      }
      mesh.position.set(...c);
      mesh.receiveShadow = true; mesh.castShadow = b.kind !== "wall" || b.wallKind !== "tall";
      g.add(mesh);
    });
    // handrails on posts along the outer walls
    const railMat = new THREE.MeshStandardMaterial({ color: 0x8d949b, roughness: 0.25, metalness: 0.9 });
    const addRail = pts => {
      for (let i = 0; i < pts.length - 1; i++) {
        const a = new THREE.Vector3(...pts[i]), b = new THREE.Vector3(...pts[i + 1]);
        const len = a.distanceTo(b); if (len < 1e-3) continue;
        const m = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, len, 12), railMat);
        m.position.copy(a.clone().add(b).multiplyScalar(0.5));
        m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), b.clone().sub(a).normalize());
        m.castShadow = true; g.add(m);
      }
      pts.forEach((p, i) => {
        if (i % 1) return;
        const post = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, 0.55, 8), railMat);
        post.rotation.x = Math.PI / 2; post.position.set(p[0], p[1], p[2] - 0.275); post.castShadow = true; g.add(post);
      });
    };
    const H = 0.9;
    const f1 = world.treads.filter(t => t.flight === 1 && t.step > 0);
    const y1 = -0.03;
    addRail([[-0.3, y1, H]].concat(f1.map(t => [t.riser + 0.15, y1, t.z + H])));
    if (world.uturn) {
      const f2 = world.treads.filter(t => t.flight === 2);
      const yo = world.totalW + 0.03, land = world.treads.find(t => t.kind === "landing");
      addRail([[land.x1 - 0.15, yo, land.z + H], [land.x0 + 0.1, yo, land.z + H]].concat(f2.map(t => [t.riser - 0.15, yo, t.z + H])));
      addRail([[land.x1 + 0.03, 0.1, land.z + H], [land.x1 + 0.03, world.totalW - 0.1, land.z + H]]);
    } else {
      addRail([[-0.3, world.width + 0.03, H]].concat(f1.map(t => [t.riser + 0.15, world.width + 0.03, t.z + H])));
    }
    return g;
  }

  // ------------------------------------------------------------------ scene
  class SceneView {
    constructor(container) {
      makeTextures(); makeMaterials();
      this.container = container;
      const renderer = this.renderer = new THREE.WebGLRenderer({ antialias: true });
      renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
      renderer.outputEncoding = THREE.sRGBEncoding;
      renderer.toneMapping = THREE.ACESFilmicToneMapping;
      renderer.toneMappingExposure = 1.05;
      renderer.shadowMap.enabled = true;
      renderer.shadowMap.type = THREE.PCFSoftShadowMap;
      container.prepend(renderer.domElement);
      const scene = this.scene = new THREE.Scene();
      // simple studio environment for reflections on metal
      const pm = new THREE.PMREMGenerator(renderer);
      const envScene = new THREE.Scene();
      envScene.background = new THREE.Color(0x8c939c);
      const lightBox = (c, p, s) => { const m = new THREE.Mesh(new THREE.BoxGeometry(...s), new THREE.MeshBasicMaterial({ color: c })); m.position.set(...p); envScene.add(m); };
      lightBox(0xffffff, [0, 0, 6], [8, 8, 0.1]); lightBox(0xdde6f0, [6, 0, 2], [0.1, 6, 3]); lightBox(0x3a3f47, [0, 0, -4], [12, 12, 0.1]);
      scene.environment = pm.fromScene(envScene, 0.04).texture;
      this.camera = new THREE.PerspectiveCamera(45, 1, 0.02, 60);
      this.controls = new THREE.OrbitControls(this.camera, renderer.domElement);
      this.controls.enableDamping = true; this.controls.dampingFactor = 0.1;
      scene.add(new THREE.HemisphereLight(0xffffff, 0x6d6457, 0.55));
      const sun = this.sun = new THREE.DirectionalLight(0xfff4e6, 1.1);
      sun.castShadow = true; sun.shadow.mapSize.set(2048, 2048); sun.shadow.bias = -0.0004;
      Object.assign(sun.shadow.camera, { left: -2.2, right: 2.2, top: 2.2, bottom: -2.2, near: 0.5, far: 20 });
      scene.add(sun, sun.target);
      const fill = new THREE.DirectionalLight(0xdfe8ff, 0.35); fill.position.set(4, 6, 3); scene.add(fill);
      const floor = new THREE.Mesh(new THREE.PlaneGeometry(14, 14), new THREE.MeshStandardMaterial({ map: TEX.floor, roughness: 0.6 }));
      TEX.floor.repeat.set(14 / 0.8, 14 / 0.8);
      floor.receiveShadow = true; floor.position.set(0.5, 1, 0); scene.add(floor);
      this.robotView = new RobotView(scene);
      this.overlays = new THREE.Group(); scene.add(this.overlays);
      this.buildOverlays();
      this.stairs = null; this.obstacleMeshes = new Map(); this.dirtMesh = null;
      this.followTarget = new THREE.Vector3(); this.mode = "follow";
      this.pipCam = new THREE.PerspectiveCamera(58, 1.5, 0.02, 20);
    }

    buildOverlays() {
      const O = this.overlays;
      const lineSeg = (n, color, opacity) => {
        const g = new THREE.BufferGeometry();
        g.setAttribute("position", new THREE.Float32BufferAttribute(new Float32Array(n * 6), 3));
        const l = new THREE.LineSegments(g, new THREE.LineBasicMaterial({ color, transparent: true, opacity }));
        l.frustumCulled = false; O.add(l); return l;
      };
      const points = (n, color, size) => {
        const g = new THREE.BufferGeometry();
        g.setAttribute("position", new THREE.Float32BufferAttribute(new Float32Array(n * 3), 3));
        const p = new THREE.Points(g, new THREE.PointsMaterial({ color, size, sizeAttenuation: true }));
        p.frustumCulled = false; O.add(p); return p;
      };
      this.ov = {
        lidar: lineSeg(160, 0xff3b30, 0.55), lidarPts: points(160, 0xff5a4e, 0.012),
        depth: points(1400, 0x23c4e8, 0.008), obst: points(1400, 0xffa31a, 0.016),
        us: lineSeg(4, 0x3fd28a, 0.9), cliff: lineSeg(4, 0xb06cff, 0.9), path: new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x22d36b })),
      };
      this.ov.path.frustumCulled = false; O.add(this.ov.path);
      this.usCones = [];
      for (let i = 0; i < 4; i++) {
        const c = new THREE.Mesh(new THREE.ConeGeometry(1, 1, 18, 1, true), new THREE.MeshBasicMaterial({ color: 0x3fd28a, transparent: true, opacity: 0.12, side: THREE.DoubleSide, depthWrite: false }));
        c.frustumCulled = false; O.add(c); this.usCones.push(c);
      }
      this.gridMesh = null;
      this.turnRing = new THREE.Mesh(new THREE.RingGeometry(0.98, 1, 64), new THREE.MeshBasicMaterial({ color: 0x22d36b, transparent: true, opacity: 0.6, side: THREE.DoubleSide }));
      O.add(this.turnRing); this.turnRing.visible = false;
    }

    setWorld(world) {
      if (this.stairs) this.scene.remove(this.stairs);
      this.stairs = buildStairs(world); this.scene.add(this.stairs);
      this.obstacleMeshes.forEach(m => this.scene.remove(m)); this.obstacleMeshes.clear();
      this.world = world;
      // dirt (instanced)
      if (this.dirtMesh) this.scene.remove(this.dirtMesh);
      const n = world.dirt.length;
      const dm = this.dirtMesh = new THREE.InstancedMesh(new THREE.BoxGeometry(0.03, 0.03, 0.0025), new THREE.MeshStandardMaterial({ color: 0x6b4423, roughness: 1 }), Math.max(1, n));
      const m4 = new THREE.Matrix4(); const r = rnd(5);
      this.dirtBase = [];
      world.dirt.forEach((c, i) => { const a = r() * 3; m4.makeRotationZ(a); m4.setPosition(c.x + (r() - 0.5) * 0.015, c.y + (r() - 0.5) * 0.015, c.z + 0.0015); dm.setMatrixAt(i, m4); this.dirtBase.push(m4.clone()); });
      dm.receiveShadow = true; this.scene.add(dm);
      this.syncObstacles();
      if (this.gridMesh) { this.overlays.remove(this.gridMesh); this.gridMesh = null; }
    }

    syncObstacles() {
      const w = this.world;
      const live = new Set(w.obstacles);
      this.obstacleMeshes.forEach((m, b) => { if (!live.has(b)) { this.scene.remove(m); this.obstacleMeshes.delete(b); } });
      w.obstacles.forEach(b => {
        if (this.obstacleMeshes.has(b)) return;
        const sx = b.max[0] - b.min[0], sy = b.max[1] - b.min[1], sz = b.max[2] - b.min[2];
        let m;
        const t = b.obs.type;
        if (t === "box") {
          m = new THREE.Group();
          const card = box(sx, sy, sz, new THREE.MeshStandardMaterial({ color: 0xb98d5a, roughness: 0.9 })); m.add(card);
          const tape = box(sx + 0.002, 0.05, sz + 0.002, new THREE.MeshStandardMaterial({ color: 0xc9b38c, roughness: 0.4 })); m.add(tape);
        } else if (t === "shoe") {
          m = new THREE.Group();
          const sole = box(sx, sy, 0.025, new THREE.MeshStandardMaterial({ color: 0xf2f2f2, roughness: 0.7 })); sole.position.z = -sz / 2 + 0.0125; m.add(sole);
          const up = roundedBox(sx * 0.85, sy * 0.95, sz - 0.025, 0.035, new THREE.MeshStandardMaterial({ color: 0x24438c, roughness: 0.6 })); up.position.set(-sx * 0.07, 0, 0.0125); m.add(up);
        } else {
          m = roundedBox(sx, sy, sz, 0.008, new THREE.MeshStandardMaterial({ color: 0xe8452c, roughness: 0.4 }));
        }
        m.position.set((b.min[0] + b.max[0]) / 2, (b.min[1] + b.max[1]) / 2, (b.min[2] + b.max[2]) / 2);
        m.traverse(o => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } });
        this.scene.add(m); this.obstacleMeshes.set(b, m);
      });
    }

    updateDirt() {
      const dm = this.dirtMesh, Z = new THREE.Matrix4().makeScale(0, 0, 0);
      let changed = false;
      this.world.dirt.forEach((c, i) => {
        const want = c.clean || c.blocked;
        if (c._shown !== !want) { dm.setMatrixAt(i, want ? Z : this.dirtBase[i]); c._shown = !want; changed = true; }
      });
      if (changed) dm.instanceMatrix.needsUpdate = true;
    }

    updateOverlays(mission, show) {
      this.overlays.visible = show;
      if (!show) return;
      const S = mission.sensors.last, ov = this.ov;
      // LiDAR
      const la = ov.lidar.geometry.attributes.position.array, pa = ov.lidarPts.geometry.attributes.position.array;
      let n = 0;
      if (S.lidar) S.lidar.hits.forEach(h => { if (!h) return; la.set([...S.lidar.o, ...h], n * 6); pa.set(h, n * 3); n++; });
      ov.lidar.geometry.setDrawRange(0, n * 2); ov.lidar.geometry.attributes.position.needsUpdate = true;
      ov.lidarPts.geometry.setDrawRange(0, n); ov.lidarPts.geometry.attributes.position.needsUpdate = true;
      // depth camera cloud, obstacle points highlighted
      const da = ov.depth.geometry.attributes.position.array, oa = ov.obst.geometry.attributes.position.array;
      let nd = 0, no = 0;
      const obsSet = new Set();
      (mission.obstacles.clusters || []).forEach(c => c.pts.forEach(p => obsSet.add(p)));
      (S.depth ? S.depth.pts : []).forEach(p => { if (nd < 1400) { da.set(p.w, nd * 3); nd++; } });
      obsSet.forEach(p => { if (no < 1400) { oa.set(p.w, no * 3); no++; } });
      ov.depth.geometry.setDrawRange(0, nd); ov.depth.geometry.attributes.position.needsUpdate = true;
      ov.obst.geometry.setDrawRange(0, no); ov.obst.geometry.attributes.position.needsUpdate = true;
      // ultrasonic cones
      const us = S.us ? [S.us.frontL, S.us.frontR, S.us.sideL, S.us.sideR] : [];
      this.usCones.forEach((c, i) => {
        const u = us[i];
        if (!u) { c.visible = false; return; }
        c.visible = true;
        const a = new THREE.Vector3(...u.o), b = new THREE.Vector3(...u.end);
        const len = Math.min(a.distanceTo(b), i < 2 ? 0.9 : 1.2);
        const dir = b.clone().sub(a).normalize();
        const spread = i < 2 ? 0.07 : 0.06;
        c.scale.set(Math.tan(spread) * len + 0.01, len, Math.tan(spread) * len + 0.01);
        c.position.copy(a.clone().add(dir.clone().multiplyScalar(len / 2)));
        c.quaternion.setFromUnitVectors(new THREE.Vector3(0, -1, 0), dir);
        const near = u.d != null && u.d < 0.25;
        c.material.color.setHex(near ? 0xffb020 : 0x3fd28a);
      });
      // cliff sensors
      const ca = ov.cliff.geometry.attributes.position.array;
      let nc = 0;
      if (S.cliff) Object.values(S.cliff).forEach(c => { ca.set([...c.o, c.o[0], c.o[1], c.o[2] - Math.min(c.d, 0.3)], nc * 6); nc++; });
      ov.cliff.geometry.setDrawRange(0, nc * 2); ov.cliff.geometry.attributes.position.needsUpdate = true;
      ov.cliff.material.color.setHex(S.cliff && Object.values(S.cliff).some(c => c.cliff) ? 0xff3b30 : 0xb06cff);
      // landing map + path
      const L = mission.landing;
      if (L && L.grid) {
        const g = L.grid, z = mission.world.zLanding + 0.004;
        if (!this.gridMesh || this.gridMesh.userData.version !== g.version || this.gridMesh.userData.grid !== g) {
          if (this.gridMesh) this.overlays.remove(this.gridMesh);
          const cells = [];
          for (let ix = 0; ix < g.nx; ix++) for (let iy = 0; iy < g.ny; iy++) if (g.occ[iy * g.nx + ix]) cells.push(g.center(ix, iy));
          const im = new THREE.InstancedMesh(new THREE.PlaneGeometry(g.res * 0.92, g.res * 0.92), new THREE.MeshBasicMaterial({ color: 0xff3b30, transparent: true, opacity: 0.35, depthWrite: false }), Math.max(1, cells.length));
          const m4 = new THREE.Matrix4();
          // grid is in odometry coordinates; draw it there (odometry ~ world, drift shows as offset)
          cells.forEach((c, i) => { m4.makeTranslation(c[0], c[1], z); im.setMatrixAt(i, m4); });
          im.count = cells.length;
          im.userData = { version: g.version, grid: g };
          this.gridMesh = im; this.overlays.add(im);
        }
        const pts = (L.path || []).map(p => new THREE.Vector3(p[0], p[1], z + 0.003));
        if (pts.length) this.ov.path.geometry.setFromPoints(pts);
        this.ov.path.visible = pts.length > 1;
        if (L.spot && mission.state !== "LANDING_ALIGN") {
          this.turnRing.visible = true;
          const rad = L.fp ? L.fp.radius : 0.4;
          this.turnRing.scale.set(rad, rad, 1); this.turnRing.position.set(L.spot[0], L.spot[1], z + 0.002);
        } else this.turnRing.visible = false;
      } else {
        if (this.gridMesh) { this.overlays.remove(this.gridMesh); this.gridMesh = null; }
        this.ov.path.visible = false; this.turnRing.visible = false;
      }
    }

    resize() {
      // shrink first so the container can report its own width, not the old canvas width
      this.renderer.domElement.style.width = "1px";
      const w = this.container.clientWidth, h = this.container.clientHeight;
      this.renderer.setSize(w, h, false);
      this.renderer.domElement.style.width = w + "px"; this.renderer.domElement.style.height = h + "px";
      this.camera.aspect = w / h; this.camera.updateProjectionMatrix();
    }

    setMode(mode, robot) {
      this.mode = mode;
      const p = new THREE.Vector3(robot.x, robot.y, robot.z);
      const c = Math.cos(robot.yaw), s = Math.sin(robot.yaw);
      if (mode === "overview") {
        const w = this.world;
        const cx = (w.X0 || 1) * 0.5 + 0.3, cy = w.totalW / 2;
        this.controls.target.set(cx, cy, w.zTop * 0.45);
        this.camera.position.set(cx - 3.2, cy - 3.4, w.zTop + 2.6);
      } else if (mode === "side") {
        this.controls.target.copy(p);
        this.camera.position.set(p.x - 0.2 * c, p.y - 1.9 * (c > 0 ? 1 : -1) + 0, p.z + 0.35);
      } else {
        this.controls.target.copy(p);
        this.camera.position.set(p.x - 1.1 * c + 0.9 * s, p.y - 1.1 * s - 0.9 * c, p.z + 0.85);
      }
      this.controls.update();
    }

    render(mission, opts) {
      const r = mission.robot;
      this.robotView.update(r, { carpet: mission.state === "CLEAN_TREAD" && mission.surfaceHere() === "carpet" });
      this.updateDirt();
      this.syncObstacles();
      this.updateOverlays(mission, opts.overlays);
      if (this.mode !== "overview" && opts.follow) {
        const goal = new THREE.Vector3(r.x + 0.15 * Math.cos(r.yaw), r.y + 0.15 * Math.sin(r.yaw), r.z - 0.05);
        const prev = this.controls.target.clone();
        this.controls.target.lerp(goal, 0.06);
        this.camera.position.add(this.controls.target.clone().sub(prev));
      }
      this.sun.position.set(r.x - 2.5, r.y - 3, r.z + 5); this.sun.target.position.set(r.x, r.y, r.z);
      this.controls.update();
      const R2 = this.renderer;
      const w = this.container.clientWidth, h = this.container.clientHeight;
      R2.setViewport(0, 0, w, h); R2.setScissorTest(false);
      this.scene.background = new THREE.Color(opts.bg || "#dfe4ea");
      R2.render(this.scene, this.camera);
      if (opts.pip) {
        // robot's depth-camera view, picture-in-picture
        const pw = Math.round(Math.min(300, w * 0.36)), ph = Math.round(pw / 1.5), px = 12, py = 12;
        const o = r.worldOf(R.CAM), M3 = r.rot();
        const fwd = SCRobot.mv(M3, [Math.cos(R.CAM_PITCH), 0, Math.sin(R.CAM_PITCH)]);
        const up = SCRobot.mv(M3, [-Math.sin(R.CAM_PITCH), 0, Math.cos(R.CAM_PITCH)]);
        this.pipCam.position.set(o[0] + fwd[0] * 0.02, o[1] + fwd[1] * 0.02, o[2] + fwd[2] * 0.02);
        this.pipCam.up.set(...up);
        this.pipCam.lookAt(o[0] + fwd[0], o[1] + fwd[1], o[2] + fwd[2]);
        this.pipCam.aspect = pw / ph; this.pipCam.updateProjectionMatrix();
        const robotVis = this.robotView.root.visible; this.robotView.root.visible = false;
        R2.setScissorTest(true); R2.setScissor(px, py, pw, ph); R2.setViewport(px, py, pw, ph);
        R2.render(this.scene, this.pipCam);
        R2.setScissorTest(false);
        this.robotView.root.visible = robotVis;
        this.pipRect = { x: px, y: h - py - ph, w: pw, h: ph };
      } else this.pipRect = null;
    }
  }

  G.SCScene = { SceneView };
})(window);
