// SPDX-License-Identifier: Apache-2.0
import vm from "node:vm";
import { createRequire } from "node:module";
import { readFile } from "node:fs/promises";
import { spawnSync } from "node:child_process";
import { fileURLToPath } from "node:url";
import path from "node:path";

const root = path.join(path.dirname(fileURLToPath(import.meta.url)), "..");
const wasmJs = path.join(root, "web/public/ekf.js");
const wasmPath = path.join(root, "web/public/ekf.wasm");
const cli = path.join(root, "build/default/ekf_cli");
const wasmDir = path.dirname(wasmJs);

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

const require = createRequire(wasmJs);
const glue = await readFile(wasmJs, "utf8");
const sandbox = {
  module: { exports: {} },
  exports: {},
  require,
  __dirname: wasmDir,
  __filename: wasmJs,
  console,
  process,
  Buffer,
  setImmediate,
  clearImmediate,
  URL,
  TextDecoder: globalThis.TextDecoder,
  TextEncoder: globalThis.TextEncoder,
  WebAssembly,
  crypto: globalThis.crypto,
  fetch: globalThis.fetch,
  atob: globalThis.atob,
  btoa: globalThis.btoa,
};
sandbox.global = sandbox;
sandbox.globalThis = sandbox;
vm.runInNewContext(glue, sandbox, { filename: wasmJs });
const createEkfModule = sandbox.module.exports.default ?? sandbox.module.exports;
if (typeof createEkfModule !== "function") {
  console.error("failed to load createEkfModule from", wasmJs);
  process.exit(1);
}

const factory = await createEkfModule({
  locateFile: (p) => path.join(wasmDir, p),
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
