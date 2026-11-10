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

const CASES = [
  ["matched", 1],
  ["matched", 2],
  ["matched", 3],
  ["matched", 4],
  ["matched", 5],
  ["urban", 1],
];

function parseNative(text) {
  const lines = text.trim().split("\n");
  const out = {
    rmse: NaN,
    gnssAccepted: -1,
    gnssGated: -1,
    gnss: [],
    state: [],
  };
  for (const line of lines) {
    if (line.startsWith("RMSE ")) {
      out.rmse = Number(line.slice(5));
    } else if (line.startsWith("GNSS_ACCEPTED ")) {
      out.gnssAccepted = Number(line.slice(14));
    } else if (line.startsWith("GNSS_GATED ")) {
      out.gnssGated = Number(line.slice(11));
    } else if (line.startsWith("G ")) {
      const [, t, acc, status] = line.split(" ");
      out.gnss.push({ t: Number(t), accepted: Number(acc), status: Number(status) });
    } else if (line.startsWith("S ")) {
      const p = line.split(" ");
      out.state.push({
        t: Number(p[1]),
        ee: Number(p[2]),
        en: Number(p[3]),
        epsi: Number(p[4]),
        ev: Number(p[5]),
        ebg: Number(p[6]),
        ek: Number(p[7]),
      });
    }
  }
  return out;
}

function fail(msg) {
  console.error(msg);
  process.exit(1);
}

async function loadWasmFactory() {
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
    fail(`failed to load createEkfModule from ${wasmJs}`);
  }
  return createEkfModule;
}

function compareScenario(scenario, seed, native, wasmOut) {
  if (Math.abs(wasmOut.rmse - native.rmse) > 1e-6) {
    fail(`${scenario} seed ${seed}: RMSE ${wasmOut.rmse} vs native ${native.rmse}`);
  }
  if (wasmOut.gnssAccepted !== native.gnssAccepted || wasmOut.gnssGated !== native.gnssGated) {
    fail(
      `${scenario} seed ${seed}: gnss counts wasm ${wasmOut.gnssAccepted}/${wasmOut.gnssGated} native ${native.gnssAccepted}/${native.gnssGated}`,
    );
  }
  const wasmGnss = wasmOut.gnss.filter((g) => g.emitted);
  if (wasmGnss.length !== native.gnss.length) {
    fail(`${scenario} seed ${seed}: gnss epoch count ${wasmGnss.length} vs ${native.gnss.length}`);
  }
  for (let i = 0; i < native.gnss.length; i++) {
    const n = native.gnss[i];
    const w = wasmGnss[i];
    const wAcc = w.accepted ? 1 : 0;
    if (Math.abs(w.t - n.t) > 1e-9 || wAcc !== n.accepted || w.status !== n.status) {
      fail(`${scenario} seed ${seed}: gnss gate mismatch at ${n.t}`);
    }
  }
  const wasmTicks = wasmOut.ticks.filter((t) => t.init);
  if (wasmTicks.length !== native.state.length) {
    fail(`${scenario} seed ${seed}: state tick count ${wasmTicks.length} vs ${native.state.length}`);
  }
  for (let i = 0; i < native.state.length; i++) {
    const n = native.state[i];
    const w = wasmTicks[i];
    const fields = [
      ["ee", w.ee, n.ee],
      ["en", w.en, n.en],
      ["epsi", w.epsi, n.epsi],
      ["ev", w.ev, n.ev],
      ["ebg", w.ebg, n.ebg],
      ["ek", w.ek, n.ek],
    ];
    for (const [name, a, b] of fields) {
      if (Math.abs(a - b) > 1e-6) {
        fail(`${scenario} seed ${seed}: state ${name} mismatch at t=${n.t}: wasm ${a} native ${b}`);
      }
    }
    if (Math.abs(w.t - n.t) > 1e-9) {
      fail(`${scenario} seed ${seed}: state time mismatch at index ${i}`);
    }
  }
}

const createEkfModule = await loadWasmFactory();
const factory = await createEkfModule({
  locateFile: (p) => path.join(wasmDir, p),
});

for (const [scenario, seed] of CASES) {
  const nativeOut = spawnSync(cli, [scenario, String(seed), "--parity-full"], {
    encoding: "utf8",
    maxBuffer: 64 * 1024 * 1024,
  });
  if (nativeOut.status !== 0) {
    fail(`native ekf_cli failed for ${scenario} seed ${seed}`);
  }
  const native = parseNative(nativeOut.stdout);
  const wasmRaw = factory.runScenario(scenario, seed);
  const wasmOut = {
    rmse: wasmRaw.rmse,
    gnssAccepted: wasmRaw.gnssAccepted,
    gnssGated: wasmRaw.gnssGated,
    gnss: wasmRaw.gnss,
    ticks: wasmRaw.ticks,
  };
  compareScenario(scenario, seed, native, wasmOut);
  console.log(`ok ${scenario} seed ${seed}`);
}

console.log("wasm_native_parity ok", wasmPath);
