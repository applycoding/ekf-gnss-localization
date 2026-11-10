// SPDX-License-Identifier: Apache-2.0
#include "ekf_sim/scenario.hpp"

#include "ekf_sim/scheduler.hpp"

#include <nlohmann/json.hpp>

#include <cstring>
#include <fstream>
#include <stdexcept>

namespace ekf::sim {

ekf::Config DefaultFilterConfig() {
  ekf::Config cfg{};
  cfg.q_gyro = 0.002 * 0.002;
  cfg.q_accel = 0.05 * 0.05;
  cfg.q_bias = 1e-4 * 1e-4;
  cfg.q_scale = 1e-5 * 1e-5;
  cfg.r_wheel = 0.05 * 0.05;
  cfg.gnss_sigma_m = kGnssWhiteSigmaM;
  cfg.gnss_bias_sigma_m = kGnssBiasSigmaM;
  cfg.gnss_bias_tau_s = kGnssBiasTauS;
  cfg.model_gnss_bias = true;
  cfg.estimate_wheel_scale = true;
  cfg.gate = true;
  return cfg;
}

namespace {

void apply_filter_json(ekf::Config& cfg, const nlohmann::json& j) {
  if (j.contains("model_gnss_bias")) {
    cfg.model_gnss_bias = j.at("model_gnss_bias").get<bool>();
  }
  if (j.contains("estimate_wheel_scale")) {
    cfg.estimate_wheel_scale = j.at("estimate_wheel_scale").get<bool>();
  }
  if (j.contains("gate")) {
    cfg.gate = j.at("gate").get<bool>();
  }
  if (j.contains("gnss_sigma_m")) {
    cfg.gnss_sigma_m = j.at("gnss_sigma_m").get<double>();
  }
}

ScenarioSpec parse_json_spec(const nlohmann::json& root) {
  ScenarioSpec spec{};
  if (root.contains("name")) {
    spec.name = root.at("name").get<std::string>();
  }
  spec.filter = DefaultFilterConfig();
  if (root.contains("filter")) {
    apply_filter_json(spec.filter, root.at("filter"));
  }
  if (root.contains("truth")) {
    const auto& truth = root.at("truth");
    if (truth.contains("wheel_scale")) {
      spec.wheel_scale_true = truth.at("wheel_scale").get<double>();
    }
    if (truth.contains("gyro_bias_rad_s")) {
      spec.gyro_bias_rad_s = truth.at("gyro_bias_rad_s").get<double>();
    }
  }
  if (root.contains("gnss_sim")) {
    const auto& g = root.at("gnss_sim");
    if (g.contains("gauss_markov")) {
      spec.gnss_gauss_markov = g.at("gauss_markov").get<bool>();
    }
    if (g.contains("dropout_start_s")) {
      spec.dropout_start_s = g.at("dropout_start_s").get<double>();
    }
    if (g.contains("dropout_end_s")) {
      spec.dropout_end_s = g.at("dropout_end_s").get<double>();
    }
    if (g.contains("multipath_probability")) {
      spec.multipath_probability = g.at("multipath_probability").get<double>();
    }
  }
  if (root.contains("wheel_sim") && root.at("wheel_sim").contains("slip_windows")) {
    const auto& slips = root.at("wheel_sim").at("slip_windows");
    spec.slip_window_count = 0;
    for (const auto& w : slips) {
      if (spec.slip_window_count >= 2) {
        break;
      }
      spec.slip_windows[spec.slip_window_count].start_s = w.at("start_s").get<double>();
      spec.slip_windows[spec.slip_window_count].end_s = w.at("end_s").get<double>();
      ++spec.slip_window_count;
    }
  }
  return spec;
}

}  // namespace

ScenarioSpec ScenarioMatched() {
  ScenarioSpec spec{};
  spec.name = "matched";
  spec.filter = DefaultFilterConfig();
  spec.filter.model_gnss_bias = false;
  spec.filter.gnss_sigma_m = kGnssWhiteSigmaM;
  spec.filter.gate = true;
  spec.filter.estimate_wheel_scale = true;
  spec.wheel_scale_true = 1.0;
  spec.gyro_bias_rad_s = kDefaultGyroBiasRadS;
  spec.gnss_gauss_markov = false;
  spec.dropout_start_s = -1.0;
  spec.multipath_probability = 0.0;
  spec.slip_window_count = 0;
  return spec;
}

ScenarioSpec ScenarioUrban() {
  ScenarioSpec spec{};
  spec.name = "urban";
  spec.filter = DefaultFilterConfig();
  spec.filter.model_gnss_bias = true;
  spec.filter.gate = true;
  spec.wheel_scale_true = 1.02;
  spec.gnss_gauss_markov = true;
  spec.dropout_start_s = kDropoutStartS;
  spec.dropout_end_s = kDropoutEndS;
  spec.multipath_probability = kMultipathProb;
  spec.slip_window_count = 2;
  spec.slip_windows[0] = {40.0, 41.0};
  spec.slip_windows[1] = {90.0, 91.0};
  return spec;
}

ScenarioSpec ScenarioUrbanWhiteModel() {
  ScenarioSpec spec = ScenarioUrban();
  spec.name = "urban_white_model";
  spec.filter.model_gnss_bias = false;
  spec.filter.gnss_sigma_m = 2.5;
  return spec;
}

ScenarioSpec LoadScenarioFile(const char* path) {
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("failed to open scenario file");
  }
  nlohmann::json root = nlohmann::json::parse(in);
  return parse_json_spec(root);
}

ScenarioSpec LoadScenarioByName(const char* name) {
  if (name == nullptr) {
    return ScenarioMatched();
  }
  const std::string n(name);
  if (n == "matched") {
    return ScenarioMatched();
  }
  if (n == "urban") {
    return ScenarioUrban();
  }
  if (n == "urban_white_model") {
    return ScenarioUrbanWhiteModel();
  }
  std::string path = std::string("scenarios/") + n + ".json";
  return LoadScenarioFile(path.c_str());
}

RunResult RunScenario(const ScenarioSpec& spec, std::uint64_t seed) {
  RunResult out{};
  MeasurementLog log{};
  Trace trace_on = RunSimulation(spec, seed, spec.filter.gate, &log);

  ScenarioSpec spec_off = spec;
  spec_off.filter.gate = false;
  Trace trace_off = RunSimulation(spec_off, seed, false, nullptr);

  out.trace = trace_on;
  out.metrics = ComputeMetrics(spec, trace_on, trace_off, spec.gyro_bias_rad_s);
  return out;
}

RunResult RunScenario(const char* scenario_name, std::uint64_t seed) {
  const ScenarioSpec spec = LoadScenarioByName(scenario_name);
  return RunScenario(spec, seed);
}

}  // namespace ekf::sim
