/*
 *Copyright 2022, Tatsuhiro Ikebe.
 *
 *Licensed under the Apache License, Version 2.0 (the "License");
 *you may not use this file except in compliance with the License.
 *You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *Unless required by applicable law or agreed to in writing, software
 *distributed under the License is distributed on an "AS IS" BASIS,
 *WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *See the License for the specific language governing permissions and
 *limitations under the License.
 */

#include <ros/ros.h>

namespace raspicat_speak
{
class speak_list
{
 public:
  std::string topic;
  std::string sentence;
  std::string voice_model;
  double speak_interval;
  std::string stop_trigger;
  uint16_t priority;
};

class voice_config
{
 public:
  double additional_half_tone;
  double all_pass_constant;
  double speech_speed_rate;
  std::string voice_model;
};
}  // namespace raspicat_speak