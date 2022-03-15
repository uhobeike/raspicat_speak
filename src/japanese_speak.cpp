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

#include <ros/package.h>

#include <thread>

namespace raspicat_speak {

japanese_speak::japanese_speak(ros::NodeHandle &nodeHandle,
                               ros::NodeHandle &private_nodeHandle)
    : nh_(nodeHandle), pnh_(private_nodeHandle) {
  getSpeakList();
  getVoiceConfig();
  run();
}

japanese_speak::~japanese_speak() {}

std::shared_ptr<ros::Subscriber>
japanese_speak::subscribe(std::string const &topic) {
  std::shared_ptr<ros::Subscriber> sub(new ros::Subscriber);
  *sub = nh_.subscribe<topic_tools::ShapeShifter>(
      topic, 100, boost::bind(&japanese_speak::callback, this, _1, topic, sub));
  currently_registered_topics_.insert(topic);
  return sub;
}

bool japanese_speak::createSubscriber() {
  ros::master::V_TopicInfo topics;
  if (ros::master::getTopics(topics)) {
    for (ros::master::TopicInfo const &t : topics) {
      if (checkSubscribeTopics(t.name))
        subscribe(t.name);
    }
  }
}

bool japanese_speak::isSubscribed(std::string const &topic) const {
  return currently_registered_topics_.find(topic) !=
         currently_registered_topics_.end();
}

bool japanese_speak::checkSubscribeTopics(std::string const &topic) {
  if (isSubscribed(topic)) {
    return false;
  }

  // if (regex) {
  //   for (std::string const &regex_str : speak_list_topics) {
  //     boost::regex e(regex_str);
  //     boost::smatch what;
  //     if (boost::regex_match(topic, what, e, boost::match_extra))
  //       return true;
  //   }
  // }

  for (auto const &slm : speak_list_map_)
    if (slm.second.topic == topic)
      return true;

  return false;
}

void japanese_speak::callback(
    ros::MessageEvent<topic_tools::ShapeShifter const> msg_event,
    std::string const &topic, std::shared_ptr<ros::Subscriber> subscriber) {
  ROS_INFO("Callback function: %s", topic.c_str());
  speak(topic);
}

void japanese_speak::getSpeakList() {
  pnh_.getParam("topics", speak_list_param_);
  // pnh.getParam("regex_topics", speak_list);
  ROS_ASSERT(speak_list_param_.getType() == XmlRpc::XmlRpcValue::TypeArray);

  for (auto i = 0; i < speak_list_param_.size(); ++i) {
    ROS_INFO("ROS Param Load speak_list: %s",
             static_cast<std::string>(speak_list_param_[i]["topic"]).c_str());
    speak_list spl;
    spl.topic = static_cast<std::string>(speak_list_param_[i]["topic"]);
    spl.sentence = static_cast<std::string>(speak_list_param_[i]["sentence"]);
    spl.priority = static_cast<int>(speak_list_param_[i]["priority"]);
    speak_list_map_.insert(std::make_pair(
        static_cast<std::string>(speak_list_param_[i]["topic"]), spl));
  }
}

void japanese_speak::getVoiceConfig() {
  pnh_.getParam("voice_config", voice_config_param_);
  ROS_ASSERT(voice_config_param_.getType() == XmlRpc::XmlRpcValue::TypeStruct);

  voc_.additional_half_tone =
      static_cast<double>(voice_config_param_["additional_half_tone"]);
  voc_.all_pass_constant =
      static_cast<double>(voice_config_param_["all_pass_constant"]);
  voc_.speech_speed_rate =
      static_cast<double>(voice_config_param_["speech_speed_rate"]);
  voc_.voice_interval =
      static_cast<double>(voice_config_param_["voice_interval"]);
  voc_.voice_model =
      static_cast<std::string>(voice_config_param_["voice_model"]);
}

void japanese_speak::speak(std::string const &topic) {
  std::string open_jtalk =
      "echo " + speak_list_map_[topic].sentence + " | open_jtalk -x " +
      "/var/lib/mecab/dic/open-jtalk/naist-jdic -m " +
      ros::package::getPath("raspicat_speak") + "/voice_model/" +
      voc_.voice_model + " -r " + std::to_string(voc_.speech_speed_rate) +
      "-fm" + std::to_string(voc_.additional_half_tone) + "-a" +
      std::to_string(voc_.all_pass_constant) + " -ow  /dev/stdout | mpv - & ";

  if (system(open_jtalk.c_str())) {
    ROS_ERROR("shell is not available on the system!");
  }
}

void japanese_speak::run() {
  // std::thread record_thread;
  // record_thread = std::thread(boost::bind(&japanese_speak::speakControl,
  // this));

  check_master_timer_ = nh_.createTimer(
      ros::Duration(1.0), boost::bind(&japanese_speak::createSubscriber, this));

  // record_thread.join();
}

void japanese_speak::checkPriority(std::set<std::string>) {}
} // namespace raspicat_speak