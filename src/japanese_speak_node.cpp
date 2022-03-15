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

#include "raspicat_speak/japanese_speak.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "japanese_speak");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  raspicat_speak::japanese_speak jsk(nh, pnh);

  // If there is only one thread, the timer callback may not work well.
  ros::AsyncSpinner spinner(pnh.param("num_callback_threads", 4));
  spinner.start();
  ros::waitForShutdown();
  return 0;
}