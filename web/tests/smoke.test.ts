// SPDX-License-Identifier: Apache-2.0
import { describe, expect, it } from "vitest";
import { GNSS_RATE_HZ, SIM_DURATION_S, SIM_DT_S, SIM_NUM_TICKS } from "../src/simConstants";
import { POS_NEES_HIGH, POS_NEES_LOW, neesSummaryLabel } from "../src/neesDisplay";

describe("simulation constants", () => {
  it("matches C++ scheduler footprint", () => {
    expect(SIM_DURATION_S).toBe(125);
    expect(SIM_DT_S).toBe(0.01);
    expect(SIM_NUM_TICKS).toBe(SIM_DURATION_S / SIM_DT_S);
    expect(GNSS_RATE_HZ * SIM_DURATION_S + 1).toBeGreaterThan(1000);
  });
});

describe("NEES summary", () => {
  it("uses the white-model diagnostic label", () => {
    expect(neesSummaryLabel("urban_white_model", 12.3)).toBe("diagnostic: model mismatch");
  });

  it("formats matched NEES against PLAN reference band", () => {
    const label = neesSummaryLabel("matched", 1.05);
    expect(label).toContain("NEES mean 1.05");
    expect(label).toContain(POS_NEES_LOW.toFixed(2));
    expect(label).toContain(POS_NEES_HIGH.toFixed(2));
  });
});
