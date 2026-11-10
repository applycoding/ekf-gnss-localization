import { test, expect } from "@playwright/test";
import { mkdirSync, writeFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const repoRoot = join(dirname(fileURLToPath(import.meta.url)), "../..");

test("demo screenshot", async ({ page }) => {
  const errors: string[] = [];
  page.on("pageerror", (e) => errors.push(String(e)));
  page.on("console", (m) => {
    if (m.type() === "error") errors.push(m.text());
  });
  await page.goto("./");
  await expect(page).toHaveURL(/ekf-gnss-localization/);
  const run = page.locator("#run");
  await expect(run).toBeEnabled({ timeout: 120_000 });
  await run.click();
  await expect(page.locator("#app")).toHaveAttribute("data-state", "done", { timeout: 120_000 });
  const rmse = await page.locator("#rmse-ekf").textContent();
  expect(Number.isFinite(Number(rmse))).toBe(true);
  for (const id of ["map", "error", "nees"]) {
    const nonBg = await page.evaluate((canvasId) => {
      const c = document.getElementById(canvasId) as HTMLCanvasElement;
      const ctx = c.getContext("2d")!;
      const d = ctx.getImageData(0, 0, c.width, c.height).data;
      let count = 0;
      for (let i = 0; i < d.length; i += 4) {
        if (!(d[i] === 26 && d[i + 1] === 31 && d[i + 2] === 41)) count++;
      }
      return count / (d.length / 4);
    }, id);
    expect(nonBg).toBeGreaterThan(0.02);
  }
  expect(errors).toEqual([]);
  mkdirSync(join(repoRoot, "docs"), { recursive: true });
  await page.screenshot({ path: join(repoRoot, "docs/demo.png"), fullPage: true });
});
