// SPDX-License-Identifier: Apache-2.0
#include "ekf_sim/scenario.hpp"

#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

namespace {

emscripten::val trace_to_val(const ekf::sim::Trace& trace, const ekf::sim::RunMetrics& metrics) {
  emscripten::val ticks = emscripten::val::array();
  for (const auto& t : trace.ticks) {
    emscripten::val o = emscripten::val::object();
    o.set("t", t.t_s);
    o.set("te", t.truth_e_m);
    o.set("tn", t.truth_n_m);
    o.set("ee", t.est_e_m);
    o.set("en", t.est_n_m);
    o.set("epsi", t.est_psi_rad);
    o.set("ev", t.est_v_m_s);
    o.set("ebg", t.est_b_g_rad_s);
    o.set("ek", t.est_k);
    o.set("env", t.envelope_m);
    o.set("init", t.filter_initialized);
    ticks.call<void>("push", o);
  }
  emscripten::val gnss = emscripten::val::array();
  for (const auto& g : trace.gnss) {
    emscripten::val o = emscripten::val::object();
    o.set("t", g.t_s);
    o.set("me", g.meas_e_m);
    o.set("mn", g.meas_n_m);
    o.set("accepted", g.accepted);
    o.set("emitted", g.emitted);
    o.set("status", g.update_status);
    o.set("nees", g.nees_pos);
    gnss.call<void>("push", o);
  }
  emscripten::val out = emscripten::val::object();
  out.set("ticks", ticks);
  out.set("gnss", gnss);
  out.set("initTime", metrics.init_time_s);
  out.set("rmse", metrics.rmse_horizontal_m);
  out.set("rmseGnss", metrics.baseline_gnss_only.rmse_horizontal_m);
  out.set("rmseDr", metrics.baseline_dead_reckoning.rmse_horizontal_m);
  out.set("rmseGateOff", metrics.baseline_ekf_gate_off.rmse_horizontal_m);
  out.set("gnssAccepted", trace.gnss_accepted);
  out.set("gnssGated", trace.gnss_gated);
  return out;
}

}  // namespace

emscripten::val run_scenario(const std::string& name, std::uint32_t seed) {
  const ekf::sim::RunResult result = ekf::sim::RunScenario(name.c_str(), seed);
  return trace_to_val(result.trace, result.metrics);
}

EMSCRIPTEN_BINDINGS(ekf_module) {
  emscripten::function("runScenario", &run_scenario);
}
