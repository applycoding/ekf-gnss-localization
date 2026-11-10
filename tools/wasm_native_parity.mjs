// SPDX-License-Identifier: Apache-2.0
import { createRequire } from "node:module";
import { spawnSync } from "node:child_process";
import { fileURLToPath } from "node:url";
import path from "node:path";

const root = path.join(path.dirname(fileURLToPath(import.meta.url)), "..");
const wasmJs = path.join(root, "web/public/ekf.js");
const wasmPath = path.join(root, "web/public/ekf.wasm");
const cli = path.join(root, "build/default/ekf_cli");

const nativeOut = spawnSync(cli, ["matched", "1", "--parity"], { encoding: "utf8" });
if (nativeOut.status !== 0) {
  console.error("native ekf_cli failed");
  process.exit(1);
}
const nativeRows = nativeOut.stdout
  .trim()
  .split("\n")
  .filter(Boolean)
  .map((line) => line.split(" ").map(Number));

const modText = await import("node:fs/promises").then((fs) => fs.readFile(wasmJs, "utf8"));
const createEkfModule = await (async () => {
  const require = createRequire(import.meta.url);
  const Module = { locateFile: (p) => path.join(root, "web/public", p) };
  const runner = new Function("Module", "require", `${modText}\nreturn createEkfModule;`);
  return runner(Module, require);
})();

const factory = await createEkfModule({
  locateFile: (p) => path.join(root, "web/public", p),
});
const out = factory.runScenario("matched", 1);
const wasmTicks = out.ticks.filter((t) => t.init);
let idx = 0;
for (const row of nativeRows) {
  const t = row[0];
  while (idx + 1 < wasmTicks.length && wasmTicks[idx].t < t - 1e-6) {
    idx++;
  }
  const w = wasmTicks[idx];
  if (Math.abs(w.ee - row[1]) > 1e-6 || Math.abs(w.en - row[2]) > 1e-6) {
    console.error("position mismatch at", t, w.ee, row[1], w.en, row[2]);
    process.exit(1);
  }
}
console.log("wasm_native_parity ok", wasmPath);
