const path = require("path");
for (const f of ["world.js", "robot.js", "autonomy.js"]) require(path.join(__dirname, "..", "js", f));
function run(name, params, hook) {
  const w = new SCWorld.World(params), m = new SCAutonomy.Mission(w, { seed: 5 });
  const notes = [];
  let last = "";
  while (!m.finished && m.t < 900) {
    m.tick(1 / 120);
    if (hook) hook(m, w);
    if (m.state !== last) { last = m.state; const e = m.events[m.events.length - 1]; if (/OBSTACLE|LANDING_PLAN|LANDING_MOVE|SAFE|DONE/.test(m.state) || /obstacle|small/.test(e.note)) notes.push(`${m.t.toFixed(0)}s ${m.state}: ${e.note}`); }
  }
  m.events.filter(e => /small obstacle/.test(e.note)).forEach(e => notes.push(`${e.t.toFixed(0)}s ${e.state}: ${e.note}`));
  console.log(`\n=== ${name}: ${m.result} (tread ${m.tread}, ${m.t.toFixed(0)} s, coverage ${(m.coverage() * 100).toFixed(0)}%)`);
  [...new Set(notes)].forEach(n => console.log("   " + n));
}
run("toy between the units", { obstacles: [{ type: "toy", loc: "f1", step: 2, fy: 0.5 }] });
let removed = false;
run("shoe in the left track, removed after 10 s", { obstacles: [{ type: "shoe", loc: "f1", step: 3, fy: 0.8 }] }, (m, w) => {
  if (!removed && m.state === "OBSTACLE_WAIT" && m.t - m.tEnter > 10) { w.removeObstacle(w.obstacles[0].obs); removed = true; }
});
run("parcel near the side of the landing", { obstacles: [{ type: "box", loc: "landing", fx: 0.72, fy: 0.3 }] });
let gone = false, firstWait = null;
run("parcel in the middle of the landing, removed 20 s after the robot starts waiting", { obstacles: [{ type: "box", loc: "landing", fx: 0.5, fy: 0.5 }] }, (m, w) => {
  if (firstWait == null && m.state === "OBSTACLE_WAIT") firstWait = m.t;
  if (!gone && firstWait != null && m.t - firstWait > 20) { w.removeObstacle(w.obstacles[0].obs); gone = true; }
});
run("too-high riser", { layout: "straight", n1: 4, h: 0.245 });
run("too-shallow treads", { d: 0.29 });
run("landing too small to turn", { L: 0.75 });
