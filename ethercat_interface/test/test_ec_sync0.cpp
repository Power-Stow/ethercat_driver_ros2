// Copyright 2026 Power Stow
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <cstdint>

#include "ethercat_interface/ec_diagnostics.hpp"

namespace
{

using ethercat_interface::sync0_distance_ns;
using ethercat_interface::sync0_margin_ns;
using ethercat_interface::sync0_phase_ns;
using ethercat_interface::sync0_index;

constexpr uint32_t CYCLE_NS = 10000000;  // 10 ms

// An application time far beyond 2^32 ns, so the 32-bit unwrapping is exercised.
constexpr uint64_t APP_TIME_NS = 1234567ULL * CYCLE_NS + 4000000;  // phase 4 ms

TEST(Sync0, phase_from_a_64_bit_system_time)
{
  EXPECT_EQ(sync0_phase_ns(APP_TIME_NS + 3250000, APP_TIME_NS, CYCLE_NS), 7250000);
}

TEST(Sync0, phase_from_a_32_bit_system_time_matches_the_64_bit_one)
{
  const uint64_t next_sync0_ns = APP_TIME_NS + 3250000;
  EXPECT_EQ(sync0_phase_ns(next_sync0_ns & 0xFFFFFFFFULL, APP_TIME_NS, CYCLE_NS), 7250000);
}

TEST(Sync0, phase_from_a_sync0_that_has_already_passed)
{
  // The register is read a few cycles before the application time it is unwrapped around.
  EXPECT_EQ(sync0_phase_ns(APP_TIME_NS - 26750000, APP_TIME_NS, CYCLE_NS), 7250000);
}

TEST(Sync0, margin_wraps_into_the_cycle)
{
  const int64_t phase_ns = 3000000;
  EXPECT_EQ(sync0_margin_ns(APP_TIME_NS - 1200000, phase_ns, CYCLE_NS), 200000);
  EXPECT_EQ(sync0_margin_ns(APP_TIME_NS - 900000, phase_ns, CYCLE_NS), 9900000);
}

TEST(Sync0, distance_is_to_the_nearest_sync0)
{
  EXPECT_EQ(sync0_distance_ns(200000, CYCLE_NS), 200000);
  EXPECT_EQ(sync0_distance_ns(9900000, CYCLE_NS), 100000);
}

TEST(Sync0, phase_from_a_32_bit_system_time_across_its_wrap)
{
  // The low half of the application time is just below 2^32, and the next SYNC0's is just past it.
  const uint64_t app_time_ns = (1ULL << 40) - 1000;
  const uint64_t next_sync0_ns = app_time_ns + 3250000;
  EXPECT_EQ(
    sync0_phase_ns(next_sync0_ns & 0xFFFFFFFFULL, app_time_ns, CYCLE_NS),
    static_cast<int64_t>(next_sync0_ns % CYCLE_NS));
}

TEST(Sync0, consecutive_frames_reach_consecutive_sync0_events_unless_they_slip)
{
  const int64_t phase_ns = 3000000;
  const uint64_t sync0_ns = 1234567ULL * CYCLE_NS + 3000000;  // a SYNC0 at that phase
  const uint64_t send_ns = sync0_ns - 100000;  // 100 us before it
  const uint64_t index = sync0_index(send_ns, phase_ns, CYCLE_NS);

  // Jitter that stays on the same side of SYNC0.
  EXPECT_EQ(sync0_index(send_ns + CYCLE_NS + 50000, phase_ns, CYCLE_NS), index + 1);
  // Sent 300 us late, past the next SYNC0, so that SYNC0 latches the previous frame again.
  EXPECT_EQ(sync0_index(send_ns + CYCLE_NS + 300000, phase_ns, CYCLE_NS), index + 2);
  // A cycle that sent no frame, with the margin back where it was.
  EXPECT_EQ(sync0_index(send_ns + 2 * CYCLE_NS, phase_ns, CYCLE_NS), index + 2);
  // A frame sent just after a SYNC0, then one sent 300 us early, before the next SYNC0: both reach
  // the same SYNC0, so the first one's outputs are never latched.
  const uint64_t late_send_ns = sync0_ns + 100000;
  EXPECT_EQ(
    sync0_index(late_send_ns + CYCLE_NS - 300000, phase_ns, CYCLE_NS),
    sync0_index(late_send_ns, phase_ns, CYCLE_NS));
}

}  // namespace
