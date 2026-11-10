// SPDX-License-Identifier: Apache-2.0
import { describe, expect, it } from "vitest";
import { GNSS_RATE_HZ, SIM_DURATION_S, SIM_DT_S, SIM_NUM_TICKS } from "../src/simConstants";

describe("simulation constants", () => {
  it("matches C++ scheduler footprint", () => {
    expect(SIM_DURATION_S).toBe(125);
    expect(SIM_DT_S).toBe(0.01);
    expect(SIM_NUM_TICKS).toBe(SIM_DURATION_S / SIM_DT_S);
    expect(GNSS_RATE_HZ * SIM_DURATION_S + 1).toBeGreaterThan(1000);
  });
});

describe("paths", () => {
  it("uses GitHub Pages base", () => {
    expect(import.meta.env.BASE_URL).toContain("ekf-gnss-localization");
  });
});
