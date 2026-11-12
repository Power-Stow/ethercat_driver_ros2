// Copyright 2026 Power Stow A/S
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

#include "ethercat_interface/ec_diagnostics.hpp"

#include <chrono>
#include <mutex>
#include <thread>

namespace ethercat_interface
{

std::unique_lock<std::mutex> lock_without_waiting(std::mutex & mutex)
{
  constexpr std::chrono::microseconds retry_period{100};
  std::unique_lock<std::mutex> lock(mutex, std::defer_lock);
  while (!lock.try_lock()) {
    std::this_thread::sleep_for(retry_period);
  }
  return lock;
}

// Out-of-line key function, so the vtable and type_info are emitted once in this library,
// which keeps the dynamic_cast from EcSlave reliable across dynamically loaded plugins.
Cia402DiagnosticsProvider::~Cia402DiagnosticsProvider() = default;
Sync0MarginConsumer::~Sync0MarginConsumer() = default;

}  // namespace ethercat_interface
