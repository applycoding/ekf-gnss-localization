import { describe, expect, it } from "vitest";

describe("paths", () => {
  it("uses GitHub Pages base", () => {
    expect("/ekf-gnss-localization/").toContain("ekf-gnss-localization");
  });
});
