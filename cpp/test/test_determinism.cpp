// SPDX-License-Identifier: Apache-2.0
#include "ekf_sim/scenario.hpp"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

TEST(Determinism, SameSeedSameRmse) {
  const auto a = ekf::sim::RunScenario("matched", 7U);
  const auto b = ekf::sim::RunScenario("matched", 7U);
  EXPECT_DOUBLE_EQ(a.metrics.rmse_horizontal_m, b.metrics.rmse_horizontal_m);
}

TEST(Determinism, SameSeedBitIdenticalTrace) {
  const auto a = ekf::sim::RunScenario("matched", 11U);
  const auto b = ekf::sim::RunScenario("matched", 11U);
  ASSERT_EQ(a.trace.ticks.size(), b.trace.ticks.size());
  for (std::size_t i = 0; i < a.trace.ticks.size(); ++i) {
    EXPECT_DOUBLE_EQ(a.trace.ticks[i].est_e_m, b.trace.ticks[i].est_e_m);
    EXPECT_DOUBLE_EQ(a.trace.ticks[i].est_n_m, b.trace.ticks[i].est_n_m);
    EXPECT_EQ(a.trace.gnss_gated, b.trace.gnss_gated);
    EXPECT_EQ(a.trace.gnss_accepted, b.trace.gnss_accepted);
  }
}

TEST(Determinism, WasmMatchesNativeParity) {
  std::ifstream wasm("web/public/ekf.wasm", std::ios::binary);
  std::ifstream js("web/public/ekf.js");
  if (!wasm.good() || !js.good()) {
    GTEST_SKIP() << "prebuilt WASM artifacts missing";
  }
  const int code = std::system("node tools/wasm_native_parity.mjs >/dev/null");
  ASSERT_EQ(code, 0);
}
