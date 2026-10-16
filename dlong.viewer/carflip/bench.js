/* bench.js — reproducible Monte Carlo evidence: node bench.js [N]   (default 100; seeds 1..N, same as the viewer's button)
   Prints the table and writes carflip_montecarlo_N<N>.csv next to this file. */
var C = require("./core.js"), fs = require("fs"), path = require("path");
var N = +(process.argv[2] || 100), rows = [], t0 = Date.now();
for (var s = 1; s <= N; s++) rows.push(C.evalSeed(s, {}));
var pct = function (x) { return (100 * x).toFixed(0) + "%"; };
console.log("Monte Carlo: " + N + " random 'berm' scenes x 4 configs, " + ((Date.now() - t0) / 1000).toFixed(1) + " s\n");
C.summarize(rows).forEach(function (r) {
  console.log((r.name + "                                              ").slice(0, 46) + " safe " + (r.safe + "/" + r.n).padStart(7) +
    "  CI95 " + (pct(r.ci[0]) + "-" + pct(r.ci[1])).padEnd(8) + "  hard " + String(r.hard).padStart(3) + "  flip " + String(r.flip).padStart(3) +
    (r.key === "none" ? "" : "  rescued " + r.rescue[0] + "/" + r.rescue[1]) + (r.estErr == null ? "" : "  att.err@touch " + r.estErr.toFixed(1) + " deg"));
});
var keys = Object.keys(C.RANGES), head = ["seed"].concat(keys);
C.CONFIGS.forEach(function (c) { ["outcome", "tilt", "sink", "estErr", "air", "fuel"].forEach(function (k) { head.push(c.key + "_" + k); }); });
var lines = [head.join(",")].concat(rows.map(function (r) {
  var a = [r.seed].concat(keys.map(function (k) { return r.params[k]; }));
  C.CONFIGS.forEach(function (c) { var o = r.out[c.key]; a.push(o.outcome, o.tilt == null ? "" : o.tilt.toFixed(1), o.sink == null ? "" : o.sink.toFixed(2), o.estErr == null ? "" : o.estErr.toFixed(2), o.air.toFixed(2), o.fuel.toFixed(2)); });
  return a.join(",");
}));
var out = path.join(__dirname, "carflip_montecarlo_N" + N + ".csv");
fs.writeFileSync(out, lines.join("\n"));
console.log("\nCSV: " + out);
