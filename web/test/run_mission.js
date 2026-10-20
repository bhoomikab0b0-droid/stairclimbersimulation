// Headless mission test: node web/test/run_mission.js '{"layout":"uturn"}'
const path = require("path");
for (const f of ["world.js", "robot.js", "autonomy.js"]) require(path.join(__dirname, "..", "js", f));
const params = JSON.parse(process.argv[2] || "{}");
const opts = JSON.parse(process.argv[3] || "{}");
const world = new SCWorld.World(params);
const m = new SCAutonomy.Mission(world, opts);
const dt = 1 / 120;
let last = null, maxPitch = 0;
const t0 = Date.now();
while (!m.finished && m.t < (opts.maxT || 900)) {
  m.tick(dt);
  maxPitch = Math.max(maxPitch, Math.abs(m.robot.pitch) * 180 / Math.PI);
  if (m.state !== last) {
    const e = m.events[m.events.length - 1];
    if (!opts.quiet) console.log(`[${m.t.toFixed(1).padStart(6)}] ${m.state.padEnd(15)} ${e.note}`);
    last = m.state;
  }
}
const steps = m.stepRecords.filter(r => r.est.stepAhead && r.est.height);
const truth = world.treads.filter(t => t.step > 0);
console.log(`RESULT: ${m.result || m.state} | tread ${m.tread} | t=${m.t.toFixed(0)}s | maxPitch ${maxPitch.toFixed(1)}° | coverage ${(m.coverage() * 100).toFixed(1)}% | wall ${(Date.now() - t0) / 1000}s`);
steps.forEach(r => {
  const tt = truth.find(t => t.flight === r.flight && t.step === (r.flight === 1 ? r.tread + 1 : r.tread - (world.P.n1) + 1));
  console.log(`  flight ${r.flight} tread ${r.tread}: h est ${(r.est.height * 1000).toFixed(1)} true ${tt ? (tt.h * 1000).toFixed(1) : "?"} touch ${r.hContact ? (r.hContact * 1000).toFixed(1) : "-"} d ${r.est.depth ? (r.est.depth * 1000).toFixed(0) : "-"} gait ${r.gait} surf ${r.surface ? r.surface.label + " " + (r.surface.conf * 100).toFixed(0) + "%" : "-"}`);
});
