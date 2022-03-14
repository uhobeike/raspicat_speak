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

#include <topic_tools/shape_shifter.h>

#include <boost/regex.hpp>
#include <mutex>

namespace raspicat_speak {

class japanese_speak {
private:
  bool createSubscriber();
  bool checkSubscribeTopics(std::string const &topic);
  void callback(ros::MessageEvent<topic_tools::ShapeShifter const> msg_event,
                std::string const &topic,
                std::shared_ptr<ros::Subscriber> subscriber);
  std::shared_ptr<ros::Subscriber> subscribe(std::string const &topic);
  bool isSubscribed(std::string const &topic) const;
  void speak();
  void speakControl();
  void checkPriority(std::set<std::string>);
  void run();

  void getSpeakList();

  XmlRpc::XmlRpcValue speak_list;

  u_int16_t num_subscribers_;
  std::set<std::string> currently_registered_topics;
  std::set<std::string> speak_now;
  std::vector<std::string> speak_list_topics;
  bool regex;

  std::mutex mtx;

public:
  japanese_speak();
  ~japanese_speak();
};
} // namespace raspicat_speak