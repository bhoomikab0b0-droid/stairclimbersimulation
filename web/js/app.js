/* UI wiring: inputs -> world/mission, simulation loop, live panels and charts. */
(function () {
  "use strict";
  const $ = id => document.getElementById(id);
  const { R } = SCRobot;
  const SURF = SCWorld.SURFACES;
  const DEG = 180 / Math.PI;
  const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();

  // ---------------------------------------------------------------- inputs
  const fmt = {
    h: v => `${v} mm`, d: v => `${v} mm`, W: v => `${v} mm`, n1: v => `${v}`, n2: v => `${v}`, L: v => `${v} mm`,
    jit: v => (+v ? `±${v} mm` : "none"), noise: v => `±${v} mm`, obFy: v => (v < 34 ? "right side" : v > 66 ? "left side" : "middle") + ` · ${v}%`,
  };
  Object.keys(fmt).forEach(k => {
    const el = $(k), out = $(k + "Out");
    const upd = () => { out.textContent = fmt[k](el.value); };
    el.addEventListener("input", () => { upd(); if (["h", "d", "W", "n1", "n2", "L", "jit"].includes(k)) markDirty(); if (k === "n1" || k === "n2") fillLocations(); });
    upd();
  });
  ["surf1", "surfL", "surf2"].forEach((id, i) => {
    $(id).innerHTML = Object.entries(SURF).map(([k, s]) => `<option value="${k}">${s.name}</option>`).join("");
    $(id).value = ["wood", "tile", "carpet"][i];
    $(id).addEventListener("change", markDirty);
  });
  $("layout").addEventListener("change", () => { layoutVisibility(); fillLocations(); markDirty(); });
  function layoutVisibility() {
    const u = $("layout").value === "uturn";
    ["n2Field", "LField", "surfLField", "surf2Field"].forEach(id => ($(id).hidden = !u));
  }
  function markDirty() { $("applyBtn").classList.add("flash"); }

  function readParams() {
    return {
      layout: $("layout").value, h: $("h").value / 1000, d: $("d").value / 1000, W: $("W").value / 1000,
      n1: +$("n1").value, n2: +$("n2").value, L: $("L").value / 1000, jitter: $("jit").value / 1000,
      seed: (Math.random() * 1e6) | 0, surf1: $("surf1").value, surfL: $("surfL").value, surf2: $("surf2").value,
    };
  }

  function fillLocations() {
    const u = $("layout").value === "uturn", n1 = +$("n1").value, n2 = +$("n2").value;
    const opts = [];
    for (let i = 1; i < (u ? n1 : n1); i++) opts.push([`f1:${i}`, `Flight 1 · step ${i}`]);
    if (u) { opts.push(["landing", "Landing"]); for (let j = 1; j < n2; j++) opts.push([`f2:${j}`, `Flight 2 · step ${j}`]); }
    const prev = $("obLoc").value;
    $("obLoc").innerHTML = opts.map(([v, t]) => `<option value="${v}">${t}</option>`).join("");
    if (opts.some(o => o[0] === prev)) $("obLoc").value = prev; else $("obLoc").value = opts[Math.min(2, opts.length - 1)][0];
  }

  // ---------------------------------------------------------------- obstacles
  let obstacles = [
    { type: "toy", loc: "f1", step: 2, fy: 0.5, fx: 0.5 },
    { type: "box", loc: "landing", fx: 0.72, fy: 0.3 },
  ];
  const OB = SCWorld.OBSTACLES;
  const locName = o => (o.loc === "landing" ? "landing" : `flight ${o.loc === "f2" ? 2 : 1}, step ${o.step}`);
  function renderObList() {
    $("obList").innerHTML = obstacles.map((o, i) => `<li><span>${OB[o.type].name} · ${locName(o)}</span><button data-i="${i}">Remove</button></li>`).join("")
      || `<li><span class="hint">No obstacles placed.</span></li>`;
    $("obList").querySelectorAll("button").forEach(b => b.addEventListener("click", () => removeObstacle(obstacles[+b.dataset.i])));
  }
  function removeObstacle(o) {
    obstacles = obstacles.filter(x => x !== o);
    if (world) world.removeObstacle(o);
    renderObList();
  }
  $("addObBtn").addEventListener("click", () => {
    const v = $("obLoc").value;
    const o = v === "landing" ? { type: $("obType").value, loc: "landing", fx: 0.5, fy: $("obFy").value / 100 }
      : { type: $("obType").value, loc: v.split(":")[0], step: +v.split(":")[1], fy: $("obFy").value / 100, fx: 0.5 };
    if (!world.uturn && o.loc !== "f1") return;
    obstacles.push(o);
    world.addObstacle(o);
    renderObList();
  });

  // ---------------------------------------------------------------- world + mission
  let world, mission, params, running = true, speed = 4, overlayTick = 0;
  const view = new SCScene.SceneView($("viewport"));
  function build(p) {
    params = p;
    world = new SCWorld.World(Object.assign({}, p, { obstacles: obstacles.slice() }));
    mission = new SCAutonomy.Mission(world, { seed: p.seed, lidarNoise: $("noise").value / 1000 });
    view.setWorld(world);
    view.setMode(view.mode === "overview" ? "overview" : "follow", mission.robot);
    $("applyBtn").classList.remove("flash");
    setRunning(true);
    lastLog = -1; lastReport = -1;
    renderObList();
  }
  $("applyBtn").addEventListener("click", () => build(readParams()));
  $("restartBtn").addEventListener("click", () => build(Object.assign({}, params, { seed: (Math.random() * 1e6) | 0 })));
  $("randomBtn").addEventListener("click", () => {
    const r = Math.random, pick = a => a[Math.floor(r() * a.length)];
    $("layout").value = r() < 0.75 ? "uturn" : "straight";
    $("h").value = 5 * Math.round((130 + r() * 75) / 5); $("d").value = 5 * Math.round((330 + r() * 70) / 5);
    $("W").value = 10 * Math.round((780 + r() * 500) / 10); $("n1").value = 3 + Math.floor(r() * 4); $("n2").value = 2 + Math.floor(r() * 4);
    $("L").value = 20 * Math.round((1000 + r() * 400) / 20); $("jit").value = Math.floor(r() * 16);
    ["surf1", "surfL", "surf2"].forEach(id => ($(id).value = pick(Object.keys(SURF))));
    ["h", "d", "W", "n1", "n2", "L", "jit"].forEach(k => $(k).dispatchEvent(new Event("input")));
    layoutVisibility(); fillLocations();
    obstacles = obstacles.filter(o => o.loc === "f1" && o.step < +$("n1").value);
    build(readParams());
  });
  $("noise").addEventListener("input", () => { if (mission) mission.sensors.opts.lidarNoise = $("noise").value / 1000; });

  function setRunning(on) {
    running = on;
    $("playBtn").textContent = on ? "❚❚ Pause" : "▶ Run";
  }
  $("playBtn").addEventListener("click", () => setRunning(!running));
  $("speedSel").addEventListener("change", e => (speed = +e.target.value));
  document.querySelectorAll(".camtools button").forEach(b => b.addEventListener("click", () => {
    document.querySelectorAll(".camtools button").forEach(x => x.setAttribute("aria-pressed", x === b));
    view.setMode(b.dataset.mode, mission.robot);
  }));
  $("alertBtn").addEventListener("click", () => {
    // remove the obstacle nearest the robot
    const r = mission.robot;
    let best = null, bd = Infinity;
    world.obstacles.forEach(b => { const d = Math.hypot((b.min[0] + b.max[0]) / 2 - r.x, (b.min[1] + b.max[1]) / 2 - r.y); if (d < bd) { bd = d; best = b; } });
    if (best) removeObstacle(best.obs);
  });

  // ---------------------------------------------------------------- panels
  const f1 = (v, d) => (v == null || Number.isNaN(v) ? "—" : v.toFixed(d == null ? 1 : d));
  const mm = v => (v == null ? "—" : `${(v * 1000).toFixed(0)} <small>mm</small>`);
  const dots = (arr) => `<span class="dots">${arr.map(([on, t, okc]) => `<i class="${on ? "on" : okc ? "ok" : ""}" title="${t}"></i>`).join("")}</span>`;

  function trueStepFor(rec) {
    const f = rec.flight;
    const idx = f === 1 ? rec.tread + 1 : rec.tread - world.P.n1 + 1;
    return world.treads.find(t => t.flight === f && t.step === idx);
  }

  function panels() {
    const m = mission, r = m.robot, S = m.sensors.last, est = m.est || {};
    // ---- sense
    const lid = S.lidar ? S.lidar.hits.filter(Boolean).length : 0;
    const dpts = S.depth ? S.depth.pts.length : 0;
    const obsPts = (m.obstacles.clusters || []).reduce((a, c) => a + c.n, 0);
    const us = m.us, cl = m.cl, t = r.touch;
    const usv = u => (u && u.d != null ? `${(u.d * 1000).toFixed(0)}` : "—");
    $("pSense").innerHTML = `
      <div class="sub">2D LiDAR · 160 beams</div><dt>returns</dt><dd>${lid}/160</dd>
      <dt>nearest riser</dt><dd>${m.per.lastObs && m.per.lastObs.riserDist != null ? mm(m.per.lastObs.riserDist) : "—"}</dd>
      <div class="sub">Depth camera</div><dt>points</dt><dd>${dpts}</dd><dt>above the stair model</dt><dd>${obsPts}</dd>
      <div class="sub">Ultrasonic (mm)</div><dt>front L / R</dt><dd>${usv(us.frontL)} / ${usv(us.frontR)}</dd><dt>side L / R</dt><dd>${usv(us.sideL)} / ${usv(us.sideR)}</dd>
      <div class="sub">IR cliff · bumpers</div><dt>cliff FL FR BL BR</dt><dd>${dots(["FL", "FR", "BL", "BR"].map(k => {
        // a unit that is lifted on its rack naturally sees a drop: shown grey, not as an alarm
        const lifted = k[0] === "F" ? r.j.fl > 0.01 : r.j.leg > 0.02;
        return [!lifted && cl[k] && cl[k].cliff, k + (lifted ? " (unit lifted)" : ""), !lifted];
      }))}</dd>
      <dt>bumpers F B pad</dt><dd>${dots([[t.front, "front"], [t.back, "back"], [t.pad, "pad"]].map(([a, b]) => [a, b, true]))}</dd>
      <div class="sub">IMU · encoders</div><dt>pitch / roll</dt><dd>${f1(m.pitch * DEG, 2)}° / ${f1(m.roll * DEG, 2)}°</dd>
      <dt>heading</dt><dd>${f1(((r.odo.yaw * DEG) % 360 + 360) % 360, 1)}°</dd>
      <dt>odometry</dt><dd>${f1(m.odomS(), 3)} m</dd>
      <dt>racks F / R</dt><dd>${f1(r.j.fl * 1000, 0)} / ${f1(r.j.leg * 1000, 0)} <small>mm</small></dd>`;
    // ---- perceive
    const tr = m.stepRecords.length ? trueStepFor(m.stepRecords[m.stepRecords.length - 1]) : null;
    const surf = m.surfaceByTread[m.tread + 1] || m.surfaceByTread[m.tread];
    const cls = m.obstacles.clusters || [];
    const L = m.landing;
    const drift = Math.hypot(r.odo.x - r.x, r.odo.y - r.y);
    $("pPerceive").innerHTML = `
      <div class="sub">Next step (Kalman)</div>
      <dt>riser height</dt><dd>${est.height != null ? `${(est.height * 1000).toFixed(1)} ± ${(est.hStd * 1000).toFixed(1)}` : "—"} <small>mm</small></dd>
      <dt>true height</dt><dd>${tr ? (tr.h * 1000).toFixed(1) : "—"} <small>mm</small></dd>
      <dt>tread depth</dt><dd>${est.depth != null ? (est.depth > 0.6 ? "long (landing)" : (est.depth * 1000).toFixed(0) + " <small>mm</small>") : "—"}</dd>
      <dt>stair width</dt><dd>${m.Wflight ? (m.Wflight * 1000).toFixed(0) + " <small>mm</small>" : "—"}</dd>
      <div class="sub">Camera</div>
      <dt>surface ahead</dt><dd>${surf ? `${SURF[surf.label].name} ${(surf.conf * 100).toFixed(0)}%` : "—"}</dd>
      <dt>obstacles</dt><dd>${cls.length ? cls.map(c => `${(c.top * 100).toFixed(0)} cm`).join(", ") : "none"}</dd>
      <div class="sub">Landing map</div>
      <dt>landing depth</dt><dd>${L && L.L ? (L.L * 1000).toFixed(0) + " <small>mm</small>" : "—"}</dd>
      <dt>occupied cells</dt><dd>${L && L.grid ? L.grid.occ.reduce((a, b) => a + b, 0) : "—"}</dd>
      <dt>path</dt><dd>${L && L.path ? `${L.path.length - 1} seg${L.path.length > 2 ? "s" : ""}` : "—"}</dd>
      <div class="sub">Truth check (simulator only)</div><dt>odometry error</dt><dd>${(drift * 1000).toFixed(0)} <small>mm</small></dd>`;
    // ---- decide
    const climb = ["SCAN", "ALIGN_WIDTH", "APPROACH", "CLEAN_TREAD", "FRONT_LIFT", "ADVANCE_FRONT", "PLACE_FRONT", "REAR_LIFT", "ADVANCE_REAR", "RETRACT_REAR"];
    const land = ["LANDING_ADVANCE", "LANDING_NARROW", "LANDING_PLAN", "LANDING_MOVE", "LANDING_TURN", "LANDING_ALIGN"];
    const end = ["OBSTACLE_WAIT", "DONE", "SAFE_ABORT"];
    const chips = a => `<div class="states">${a.map(s => `<span class="${s === m.state ? "on" : ""}">${s.replace("LANDING_", "L·")}</span>`).join("")}</div>`;
    const tiltMargin = 20 - Math.abs(m.pitch * DEG);
    $("pDecide").innerHTML = `<div class="label">Climb cycle</div>${chips(climb)}<div class="label">Landing</div>${chips(land)}${chips(end)}
      <dl class="kv"><dt>gait profile</dt><dd>${m.gait ? m.gait.name : "—"}</dd><dt>tilt margin</dt><dd>${tiltMargin.toFixed(1)}°</dd></dl>
      <div class="decision">${m.decision || SCAutonomy.STATE_INFO[m.state]}</div>`;
    // ---- act
    const sv = m.sv, sh = m.surfaceHere(), sm = SURF[sh] || SURF.tile;
    $("pAct").innerHTML = `
      <div class="sub">Omni wheels</div><dt>forward</dt><dd>${f1(r.cmd.vx * 1000, 0)} <small>mm/s</small></dd>
      <dt>sideways</dt><dd>${f1(r.cmd.vy * 1000, 0)} <small>mm/s</small></dd><dt>turn</dt><dd>${f1(r.cmd.wz * DEG, 1)} <small>°/s</small></dd>
      <div class="sub">Rack-and-pinion</div><dt>front set / at</dt><dd>${f1(sv.fl.target * 1000, 0)} / ${f1(r.j.fl * 1000, 0)} <small>mm</small></dd>
      <dt>rear set / at</dt><dd>${f1(sv.leg.target * 1000, 0)} / ${f1(r.j.leg * 1000, 0)} <small>mm</small></dd>
      <dt>levelling trim</dt><dd>${f1(m.trim * 1000, 1)} <small>mm</small></dd>
      <div class="sub">Frame &amp; cleaner</div><dt>frame width x</dt><dd>${f1(r.span * 1000, 0)} <small>mm</small></dd>
      <dt>head rail / reach</dt><dd>${f1(r.j.rail * 1000, 0)} / ${f1(r.j.reach * 1000, 0)} <small>mm</small></dd>
      <dt>head</dt><dd>${r.cleaning ? "down, cleaning" : r.j.head > 0.005 ? "moving" : "raised"}</dd>
      <dt>mode</dt><dd>${m.state === "CLEAN_TREAD" ? sm.mode : "idle"}</dd>
      <dt>suction</dt><dd>${m.state === "CLEAN_TREAD" ? sm.suction + "%" : "0%"}</dd>`;
    // ---- HUD
    const info = SCAutonomy.STATE_INFO[m.state] || "";
    const chip = $("stateChip");
    chip.innerHTML = `<b>${m.state.replace(/_/g, " ")}</b><small>${info}</small>`;
    chip.style.borderLeftColor = m.state === "SAFE_ABORT" ? css("--crit") : m.state === "OBSTACLE_WAIT" ? css("--warn") : m.state === "DONE" ? css("--accent") : m.state.startsWith("LANDING") ? css("--perceive") : css("--decide");
    const n1 = world.P.n1, n2 = world.uturn ? world.P.n2 : 0;
    let sq = "";
    for (let i = 1; i <= n1 + n2; i++) sq += `<i class="${i <= m.tread ? "done" : ""} ${world.uturn && i === n1 ? "land" : ""}" title="${world.uturn && i === n1 ? "landing" : "step " + i}"></i>`;
    $("progress").innerHTML = `<span><b>${m.tread}</b> of ${n1 + n2} climbed · ${(m.coverage() * 100).toFixed(0)}% cleaned · ${m.t.toFixed(0)} s</span><span class="steps">${sq}</span>`;
    const al = $("alert");
    if (m.alert && (m.state === "OBSTACLE_WAIT" || m.state === "SAFE_ABORT")) {
      al.hidden = false; al.classList.toggle("crit", m.alert.level === "crit");
      $("alertText").textContent = m.alert.text;
      $("alertBtn").hidden = !(m.state === "OBSTACLE_WAIT" && world.obstacles.length);
    } else al.hidden = true;
    const pill = $("statusPill");
    const st = m.state === "DONE" ? ["done", "Mission complete"] : m.state === "SAFE_ABORT" ? ["crit", "Stopped safely"] : m.state === "OBSTACLE_WAIT" ? ["warn", "Waiting"] : running ? ["", "Running"] : ["warn", "Paused"];
    pill.className = "pill " + st[0]; pill.textContent = st[1];
  }

  let lastLog = -1, lastReport = -1;
  function logAndReport() {
    const m = mission;
    if (m.events.length !== lastLog) {
      lastLog = m.events.length;
      $("log").innerHTML = m.events.slice(-120).reverse().map(e => `<li><span class="mono">${e.t.toFixed(1)} s</span><span><b>${e.state}</b>${e.note ? " · " + e.note : ""}</span></li>`).join("");
    }
    const key = m.stepRecords.length * 10 + (m.finished ? 1 : 0) + m.stepRecords.filter(r => r.hContact).length * 1000;
    if (key !== lastReport) {
      lastReport = key;
      const rows = m.stepRecords.filter(r => r.est.stepAhead && r.est.height != null).map(r => {
        const t = trueStepFor(r);
        const e = t ? (r.est.height - t.h) * 1000 : null;
        const surf = r.surface ? `${SURF[r.surface.label].name.split(" ").pop()} ${(r.surface.conf * 100).toFixed(0)}%` : "—";
        const nextT = world.treads.find(x => t && x.flight === t.flight && x.step === t.step);
        const trueSurf = nextT ? SURF[nextT.surface].name.split(" ").pop() : "";
        return `<tr><td>F${r.flight} · ${r.flight === 1 ? r.tread + 1 : r.tread - world.P.n1 + 1}</td><td>${r.gait || "—"}</td>
          <td class="num">${t ? (t.h * 1000).toFixed(1) : "—"}</td><td class="num">${(r.est.height * 1000).toFixed(1)}</td>
          <td class="num">${e == null ? "—" : (e >= 0 ? "+" : "") + e.toFixed(1)}</td><td class="num">${r.hContact ? (r.hContact * 1000).toFixed(1) : "—"}</td>
          <td class="num">${t && t.kind === "tread" ? ((t.x1 - t.x0) * 1000).toFixed(0) : "long"}</td><td class="num">${r.est.depth != null ? (r.est.depth > 0.6 ? "long" : (r.est.depth * 1000).toFixed(0)) : "—"}</td>
          <td>${surf}${trueSurf && r.surface && SURF[r.surface.label].name.split(" ").pop() !== trueSurf ? ` <small>(is ${trueSurf})</small>` : ""}</td></tr>`;
      }).join("");
      $("report").innerHTML = `<thead><tr><th>Step</th><th>Gait</th><th>h true</th><th>h est.</th><th>error</th><th>touch</th><th>d true</th><th>d est.</th><th>Surface (camera)</th></tr></thead><tbody>${rows || `<tr><td colspan="9" class="hint">No steps measured yet.</td></tr>`}</tbody>`;
      $("reportLead").textContent = m.finished
        ? `${m.result}. ${m.tread} steps in ${m.t.toFixed(0)} s, ${(m.coverage() * 100).toFixed(0)}% of the reachable tread area cleaned. Heights and depths in mm.`
        : "Filled in step by step: what the robot measured against the true staircase. Heights and depths in mm.";
    }
  }

  // ---------------------------------------------------------------- charts
  function setupCanvas(c) {
    const dpr = window.devicePixelRatio || 1, w = c.clientWidth, h = c.clientHeight;
    if (c.width !== Math.round(w * dpr) || c.height !== Math.round(h * dpr)) { c.width = Math.round(w * dpr); c.height = Math.round(h * dpr); }
    const g = c.getContext("2d"); g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, w, h);
    return [g, w, h];
  }
  function axes(g, w, h, pad, xr, yr, xt, yt, unitX, unitY) {
    const X = v => pad.l + (v - xr[0]) / (xr[1] - xr[0]) * (w - pad.l - pad.r);
    const Y = v => h - pad.b - (v - yr[0]) / (yr[1] - yr[0]) * (h - pad.t - pad.b);
    g.font = `11px ${css("--font-mono") || "monospace"}`; g.fillStyle = css("--ink-3"); g.strokeStyle = css("--line"); g.lineWidth = 1;
    yt.forEach(v => { g.beginPath(); g.moveTo(pad.l, Y(v)); g.lineTo(w - pad.r, Y(v)); g.stroke(); g.textAlign = "right"; g.fillText(v + unitY, pad.l - 4, Y(v) + 4); });
    xt.forEach(v => { g.textAlign = "center"; g.fillText(v + unitX, X(v), h - pad.b + 14); });
    return [X, Y];
  }
  function chartLidar() {
    const [g, w, h] = setupCanvas($("cLidar"));
    const pts = mission.per.lastPts, obs = mission.per.lastObs;
    if (!pts) return;
    const pad = { l: 40, r: 8, t: 8, b: 20 };
    const xr = [0, 1.6], yr = [-0.3, 0.55];
    const [X, Y] = axes(g, w, h, pad, xr, yr, [0, 0.4, 0.8, 1.2, 1.6], [-0.2, 0, 0.2, 0.4], "m", "");
    const lab = obs && obs.labels;
    pts.forEach((p, i) => {
      if (Number.isNaN(p[0]) || p[0] > xr[1] || p[1] < yr[0] || p[1] > yr[1]) return;
      g.fillStyle = lab && lab[i] === "H" ? css("--s1") : lab && lab[i] === "V" ? css("--s2") : css("--ink-3");
      g.beginPath(); g.arc(X(p[0]), Y(p[1]), 2.2, 0, Math.PI * 2); g.fill();
    });
    g.fillStyle = css("--s3"); g.fillRect(X(0) - 4, Y(0) - 4, 8, 8);
    if (obs && obs.stepAhead && obs.height != null) {
      const x = obs.riserDist, z0 = obs.groundZ;
      g.strokeStyle = css("--ink"); g.fillStyle = css("--ink"); g.lineWidth = 1;
      g.beginPath(); g.moveTo(X(x) - 6, Y(z0)); g.lineTo(X(x) - 6, Y(z0 + obs.height)); g.stroke();
      g.textAlign = "right"; g.font = `12px ${css("--font-mono")}`;
      g.fillText(`h ${(obs.height * 1000).toFixed(0)} mm`, X(x) - 10, Y(z0 + obs.height / 2) + 4);
      if (obs.depth && obs.depth < 0.6) {
        const yy = Y(z0 + obs.height) - 10;
        g.beginPath(); g.moveTo(X(x), yy); g.lineTo(X(x + obs.depth), yy); g.stroke();
        g.textAlign = "center"; g.fillText(`d ${(obs.depth * 1000).toFixed(0)} mm`, X(x + obs.depth / 2), yy - 5);
      }
    } else if (obs && obs.wall) {
      g.fillStyle = css("--ink"); g.textAlign = "left"; g.font = `12px ${css("--font-mono")}`;
      g.fillText(`no riser · wall at ${(obs.wallDist * 1000).toFixed(0)} mm`, pad.l + 6, pad.t + 14);
    }
  }
  function chartKalman() {
    const [g, w, h] = setupCanvas($("cKalman"));
    const recs = mission.stepRecords.filter(r => r.history.length && r.est.stepAhead);
    const hist = mission.state === "SCAN" && mission.per.history.length ? mission.per.history : recs.length ? recs[recs.length - 1].history : [];
    const rec = mission.state === "SCAN" ? null : recs[recs.length - 1];
    const vals = hist.map(x => x.raw).filter(v => v != null);
    if (!vals.length) return;
    const truth = rec ? trueStepFor(rec) : null;
    const all = vals.concat(hist.map(x => x.kf).filter(v => v != null)).concat(truth ? [truth.h] : []).map(v => v * 1000);
    let lo = Math.min(...all) - 2, hi = Math.max(...all) + 2;
    lo = Math.floor(lo); hi = Math.ceil(hi);
    const pad = { l: 46, r: 10, t: 8, b: 20 };
    const step = Math.max(1, Math.ceil((hi - lo) / 4));
    const yt = []; for (let v = Math.ceil(lo / step) * step; v <= hi; v += step) yt.push(v);
    const [X, Y] = axes(g, w, h, pad, [0.5, Math.max(12, hist.length) + 0.5], [lo, hi], [1, 4, 8, 12], yt, "", "");
    if (truth) { g.strokeStyle = css("--ink-3"); g.setLineDash([4, 4]); g.beginPath(); g.moveTo(pad.l, Y(truth.h * 1000)); g.lineTo(w - pad.r, Y(truth.h * 1000)); g.stroke(); g.setLineDash([]); }
    g.strokeStyle = css("--s1"); g.lineWidth = 2; g.beginPath();
    hist.forEach((x, i) => { if (x.kf == null) return; const px = X(i + 1), py = Y(x.kf * 1000); i ? g.lineTo(px, py) : g.moveTo(px, py); }); g.stroke();
    g.fillStyle = css("--s2");
    hist.forEach((x, i) => { if (x.raw == null) return; g.beginPath(); g.arc(X(i + 1), Y(x.raw * 1000), 3.2, 0, Math.PI * 2); g.fill(); });
  }
  function chartTime() {
    const [g, w, h] = setupCanvas($("cTime"));
    const H = mission.history;
    if (H.length < 2) return;
    const t1 = H[H.length - 1].t, t0 = Math.max(0, t1 - 90);
    const pad = { l: 40, r: 10, t: 8, b: 20 };
    const xt = []; for (let v = Math.ceil(t0 / 15) * 15; v <= t1; v += 15) xt.push(v);
    const [X, Y] = axes(g, w, h, pad, [t0, Math.max(t0 + 10, t1)], [-60, 260], xt, [0, 100, 200], "s", "");
    const line = (key, col, k) => {
      g.strokeStyle = col; g.lineWidth = 2; g.beginPath(); let first = true;
      H.forEach(p => { if (p.t < t0) return; const px = X(p.t), py = Y(Math.max(-60, Math.min(260, p[key] * k))); first ? g.moveTo(px, py) : g.lineTo(px, py); first = false; });
      g.stroke();
    };
    line("fl", css("--s2"), 1); line("leg", css("--s3"), 1); line("pitch", css("--s1"), 10);
  }

  // ---------------------------------------------------------------- loop
  let last = performance.now(), panelT = 0, chartT = 0, bg = css("--scene-bg");
  const themeChange = () => { bg = css("--scene-bg"); };
  window.matchMedia("(prefers-color-scheme: dark)").addEventListener("change", themeChange);
  new MutationObserver(themeChange).observe(document.documentElement, { attributes: true, attributeFilter: ["data-theme"] });

  function frame(now) {
    const dtReal = Math.min(0.1, (now - last) / 1000); last = now;
    if (running && !mission.finished) {
      const n = Math.min(80, Math.round(dtReal * speed * 120));
      for (let i = 0; i < n && !mission.finished; i++) {
        mission.tick(1 / 120);
        // keep the display sensors alive between scans (they do not affect decisions)
        if ($("ovChk").checked && ++overlayTick % 12 === 0 && mission.state !== "SCAN") {
          const imu = mission.sensors.imu();
          mission.sensors.lidar(imu);
          if (overlayTick % 36 === 0 && !mission.state.startsWith("LANDING")) mission.sensors.depthCamera(30, 18);
        }
      }
    }
    view.render(mission, { overlays: $("ovChk").checked, pip: $("pipChk").checked, follow: $("followChk").checked, bg });
    const pl = $("pipLabel");
    if (view.pipRect) {
      const surf = mission.surfaceByTread[mission.tread + 1] || mission.surfaceByTread[mission.tread];
      pl.hidden = false; pl.style.left = view.pipRect.x + 6 + "px"; pl.style.top = view.pipRect.y + 6 + "px";
      pl.textContent = `Robot depth camera${surf ? ` · ${SURF[surf.label].name} ${(surf.conf * 100).toFixed(0)}%` : ""}`;
    } else pl.hidden = true;
    if (now - panelT > 150) { panelT = now; panels(); logAndReport(); }
    if (now - chartT > 300) { chartT = now; chartLidar(); chartKalman(); chartTime(); }
    requestAnimationFrame(frame);
  }

  window.addEventListener("resize", () => view.resize());
  layoutVisibility(); fillLocations();
  build(readParams());
  // #skip120 fast-forwards the mission (handy for checking later phases)
  const skip = /^#skip(\d+)$/.exec(location.hash);
  if (skip) { while (mission.t < +skip[1] && !mission.finished) mission.tick(1 / 120); setRunning(false); view.setMode("follow", mission.robot); }
  view.resize();
  requestAnimationFrame(frame);
})();
