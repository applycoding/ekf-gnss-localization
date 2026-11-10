// SPDX-License-Identifier: Apache-2.0
import { defineConfig } from "vite";

export default defineConfig({
  base: "/ekf-gnss-localization/",
  publicDir: "public",
  build: {
    outDir: "dist",
  },
  test: {
    environment: "node",
    exclude: ["e2e/**", "node_modules/**"],
  },
});
