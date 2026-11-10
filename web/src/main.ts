// SPDX-License-Identifier: Apache-2.0
import { neesSummaryLabel } from "./neesDisplay";

type WasmModule = {
  runScenario: (name: string, seed: number) => {
    ticks: Array<{ t: number; te: number; tn: number; ee: number; en: number; env: number; init: boolean }>;
    gnss: Array<{ t: number; me: number; mn: number; accepted: boolean; nees: number }>;
    initTime: number;
    rmse: number;
    rmseGnss: number;
    rmseDr: number;
    rmseGateOff: number;
  };
};

declare global {
  interface Window {
    createEkfModule: (opts: { locateFile: (p: string) => string }) => Promise<WasmModule>;
  }
}

const app = document.getElementById("app")!;
const runBtn = document.getElementById("run") as HTMLButtonElement;
const errEl = document.getElementById("load-error") as HTMLParagraphElement;

function setState(s: string) {
  app.dataset.state = s;
}

function drawMap(canvas: HTMLCanvasElement, ticks: Array<{ te: number; tn: number; ee: number; en: number; init: boolean }>) {
  const ctx = canvas.getContext("2d");
  if (!ctx) return;
  ctx.fillStyle = "#1a1f29";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  const pts = ticks.filter((t) => t.init);
  if (pts.length < 2) return;
  let minE = pts[0].te;
  let maxE = pts[0].te;
  let minN = pts[0].tn;
  let maxN = pts[0].tn;
  for (const p of pts) {
    minE = Math.min(minE, p.te, p.ee);
    maxE = Math.max(maxE, p.te, p.ee);
    minN = Math.min(minN, p.tn, p.en);
    maxN = Math.max(maxN, p.tn, p.en);
  }
  const pad = 10;
  const sx = (canvas.width - 2 * pad) / Math.max(maxE - minE, 1);
  const sy = (canvas.height - 2 * pad) / Math.max(maxN - minN, 1);
  const tx = (e: number) => pad + (e - minE) * sx;
  const ty = (n: number) => canvas.height - pad - (n - minN) * sy;
  ctx.strokeStyle = "#7aa2ff";
  ctx.beginPath();
  for (let i = 0; i < pts.length; i++) {
    const p = pts[i];
    if (i === 0) ctx.moveTo(tx(p.te), ty(p.tn));
    else ctx.lineTo(tx(p.te), ty(p.tn));
  }
  ctx.stroke();
  ctx.strokeStyle = "#5cf29c";
  ctx.beginPath();
  for (let i = 0; i < pts.length; i++) {
    const p = pts[i];
    if (i === 0) ctx.moveTo(tx(p.ee), ty(p.en));
    else ctx.lineTo(tx(p.ee), ty(p.en));
  }
  ctx.stroke();
}

async function loadWasm(): Promise<WasmModule> {
  if (!window.createEkfModule) {
    await new Promise<void>((resolve, reject) => {
      const s = document.createElement("script");
      s.src = `${import.meta.env.BASE_URL}ekf.js`;
      s.onload = () => resolve();
      s.onerror = () => reject(new Error("failed to load ekf.js"));
      document.head.appendChild(s);
    });
  }
  const factory = window.createEkfModule;
  if (!factory) {
    throw new Error("WASM module factory missing");
  }
  return factory({
    locateFile: (p) => `${import.meta.env.BASE_URL}${p}`,
  });
}

async function runSimulation(mod: WasmModule) {
  const scenario = (document.getElementById("scenario") as HTMLSelectElement).value;
  const seed = Number((document.getElementById("seed") as HTMLInputElement).value) || 1;
  const out = mod.runScenario(scenario, seed);
  (document.getElementById("rmse-ekf")!).textContent = out.rmse.toFixed(3);
  (document.getElementById("rmse-gnss")!).textContent = out.rmseGnss.toFixed(3);
  (document.getElementById("rmse-dr")!).textContent = out.rmseDr.toFixed(3);
  (document.getElementById("rmse-off")!).textContent = out.rmseGateOff.toFixed(3);
  const neesVals = out.gnss.filter((g) => g.accepted && g.nees > 0).map((g) => g.nees);
  const meanNe = neesVals.length
    ? neesVals.reduce((a, b) => a + b, 0) / neesVals.length
    : Number.NaN;
  const label = neesSummaryLabel(scenario, meanNe);
  (document.getElementById("nees-label")!).textContent = label;
  const map = document.getElementById("map") as HTMLCanvasElement;
  const errCanvas = document.getElementById("error") as HTMLCanvasElement;
  const neesCanvas = document.getElementById("nees") as HTMLCanvasElement;
  drawMap(map, out.ticks);
  const pts = out.ticks.filter((t) => t.init);
  const ectx = errCanvas.getContext("2d");
  if (ectx && pts.length > 1) {
    ectx.fillStyle = "#1a1f29";
    ectx.fillRect(0, 0, errCanvas.width, errCanvas.height);
    ectx.strokeStyle = "#ffb347";
    ectx.lineWidth = 2;
    ectx.fillStyle = "rgba(255,179,71,0.35)";
    ectx.beginPath();
    for (let i = 0; i < pts.length; i++) {
      const err = Math.hypot(pts[i].ee - pts[i].te, pts[i].en - pts[i].tn);
      const x = (pts[i].t / 125) * errCanvas.width;
      const y = errCanvas.height - Math.min(err, 20) * (errCanvas.height / 20);
      if (i === 0) ectx.moveTo(x, y);
      else ectx.lineTo(x, y);
    }
    ectx.lineTo(errCanvas.width, errCanvas.height);
    ectx.lineTo(0, errCanvas.height);
    ectx.closePath();
    ectx.fill();
    ectx.stroke();
  }
  const nctx = neesCanvas.getContext("2d");
  if (nctx && out.gnss.length > 1) {
    nctx.fillStyle = "#1a1f29";
    nctx.fillRect(0, 0, neesCanvas.width, neesCanvas.height);
    nctx.strokeStyle = "#c792ea";
    nctx.lineWidth = 2;
    nctx.fillStyle = "rgba(199,146,234,0.35)";
    const accepted = out.gnss.filter((g) => g.accepted && g.nees > 0);
    nctx.beginPath();
    for (let i = 0; i < accepted.length; i++) {
      const x = (accepted[i].t / 125) * neesCanvas.width;
      const y = neesCanvas.height - Math.min(accepted[i].nees, 15) * (neesCanvas.height / 15);
      if (i === 0) nctx.moveTo(x, y);
      else nctx.lineTo(x, y);
    }
    nctx.lineTo(neesCanvas.width, neesCanvas.height);
    nctx.lineTo(0, neesCanvas.height);
    nctx.closePath();
    nctx.fill();
    nctx.stroke();
  }
  setState("done");
}

loadWasm()
  .then((mod) => {
    runBtn.disabled = false;
    runBtn.addEventListener("click", () => {
      setState("running");
      void runSimulation(mod);
    });
    setState("ready");
  })
  .catch((e: unknown) => {
    errEl.hidden = false;
    errEl.textContent = String(e);
    setState("error");
  });
