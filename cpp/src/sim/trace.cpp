// SPDX-License-Identifier: Apache-2.0
#include "ekf_sim/constants.hpp"
#include "ekf_sim/trace.hpp"

namespace ekf::sim {

void TraceClear(Trace& trace) {
  trace.ticks.clear();
  trace.gnss.clear();
  trace.wheel.clear();
  trace.init_time_s = -1.0;
  trace.seed = 0;
  trace.scenario_name[0] = '\0';
}

void TraceReserve(Trace& trace) {
  trace.ticks.reserve(static_cast<std::size_t>(kNumTicks));
  trace.gnss.reserve(kNumTicks / 10 + 1);
  trace.wheel.reserve(kNumTicks / 2 + 1);
}

}  // namespace ekf::sim
