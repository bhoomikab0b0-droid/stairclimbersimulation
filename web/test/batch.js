const path = require("path");
for (const f of ["world.js", "robot.js", "autonomy.js"]) require(path.join(__dirname, "..", "js", f));
function run(params, opts) {
  const w = new SCWorld.World(params), m = new SCAutonomy.Mission(w, opts || {});
  let maxP = 0;
  while (!m.finished && m.t < 700) {
    m.tick(1 / 120); maxP = Math.max(maxP, Math.abs(m.robot.pitch));
    if (m.state === "OBSTACLE_WAIT" && m.t - m.tEnter > 3) break;
  }
  return { m, w, maxP: maxP * 57.3 };
}
const surfs = ["tile", "marble", "wood", "carpet"];
let ok = 0, n = 0;
for (let s = 1; s <= 24; s++) {
  const r = k => ((Math.sin(s * 97.13 + k * 13.7) + 1) / 2);
  const P = { layout: s % 3 === 0 ? "straight" : "uturn", h: 0.13 + 0.07 * r(1), d: 0.33 + 0.06 * r(2), W: 0.75 + 0.55 * r(3),
    n1: 3 + Math.floor(4 * r(4)), n2: 2 + Math.floor(4 * r(5)), L: 1.0 + 0.4 * r(6), jitter: 0.015, seed: s,
    surf1: surfs[s % 4], surfL: surfs[(s + 1) % 4], surf2: surfs[(s + 2) % 4] };
  const { m, maxP } = run(P, { seed: s });
  const good = (m.result || "").startsWith("Top");
  ok += good; n++;
  console.log(`${String(s).padStart(2)} ${P.layout.padEnd(8)} h=${(P.h*1000).toFixed(0)} d=${(P.d*1000).toFixed(0)} W=${(P.W*1000).toFixed(0)} n=${P.n1}+${P.layout==="uturn"?P.n2:0} L=${(P.L*1000).toFixed(0)} ${good ? "OK  " : "FAIL"} ${m.state} ${m.result || ""} tread=${m.tread} t=${m.t.toFixed(0)} pitch=${maxP.toFixed(1)} cov=${(m.coverage()*100).toFixed(0)}%`);
}
console.log(`success ${ok}/${n}`);
const cases = [
  ["too high riser", { heights: null, h: 0.17 }, w => { w.P; }],
];
void cases;
