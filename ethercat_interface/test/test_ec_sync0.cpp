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
using ethercat_interface::sync0_slipped;

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

TEST(Sync0, slip_only_when_consecutive_frames_straddle_sync0)
{
  EXPECT_FALSE(sync0_slipped(200000, 400000, CYCLE_NS));
  EXPECT_FALSE(sync0_slipped(4800000, 5200000, CYCLE_NS));
  EXPECT_TRUE(sync0_slipped(200000, 9900000, CYCLE_NS));
  EXPECT_TRUE(sync0_slipped(9900000, 100000, CYCLE_NS));
}

}  // namespace
