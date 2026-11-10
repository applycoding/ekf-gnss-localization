// SPDX-License-Identifier: Apache-2.0
#include "ekf_sim/scenario.hpp"

#include <cstdlib>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#ifndef EKF_SOURCE_DIR
#define EKF_SOURCE_DIR "."
#endif

namespace {

std::string wasm_js_path() {
  return std::string(EKF_SOURCE_DIR) + "/web/public/ekf.js";
}

std::string wasm_binary_path() {
  return std::string(EKF_SOURCE_DIR) + "/web/public/ekf.wasm";
}

bool require_wasm_parity() {
  const char* flag = std::getenv("EKF_REQUIRE_WASM_PARITY");
  return flag != nullptr && flag[0] == '1';
}

}  // namespace

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
  std::ifstream wasm(wasm_binary_path(), std::ios::binary);
  std::ifstream js(wasm_js_path());
  if (!wasm.good() || !js.good()) {
    if (require_wasm_parity()) {
      FAIL() << "WASM artifacts missing at " << wasm_binary_path();
    }
    GTEST_SKIP() << "prebuilt WASM artifacts missing";
  }
  const std::string cmd =
      "node " + std::string(EKF_SOURCE_DIR) + "/tools/wasm_native_parity.mjs >/dev/null";
  const int code = std::system(cmd.c_str());
  ASSERT_EQ(code, 0);
}
