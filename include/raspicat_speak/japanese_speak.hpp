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

#include "raspicat_speak/speak_list.hpp"

#include <ros/ros.h>

#include <topic_tools/shape_shifter.h>

#include <boost/regex.hpp>
#include <mutex>

namespace raspicat_speak {

class japanese_speak {
private:
  void createSubscriber();
  bool checkSubscribeTopics(std::string const &topic);
  void callback(ros::MessageEvent<topic_tools::ShapeShifter const> msg_event,
                std::string const &topic,
                std::shared_ptr<ros::Subscriber> subscriber);
  std::shared_ptr<ros::Subscriber> subscribe(std::string const &topic);
  bool isSubscribed(std::string const &topic) const;
  void speak(std::string const &topic);
  void speakControl();
  void checkPriority(std::set<std::string>);
  void run();

  void getSpeakList();
  void getVoiceConfig();

  ros::NodeHandle &nh_, &pnh_;
  ros::Timer check_master_timer_;

  XmlRpc::XmlRpcValue speak_list_param_;
  XmlRpc::XmlRpcValue voice_config_param_;
  std::map<std::string, speak_list> speak_list_map_;
  voice_config voc_;

  std::set<std::string> currently_registered_topics_;
  std::set<std::string> speak_now_;
  std::vector<std::string> speak_list_topics_;
  bool regex_;

  std::mutex mtx_;

public:
  japanese_speak(ros::NodeHandle &nodeHandle,
                 ros::NodeHandle &private_nodeHandle);
  ~japanese_speak();
};
} // namespace raspicat_speak