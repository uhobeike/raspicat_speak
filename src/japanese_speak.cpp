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

japanese_speak::japanese_speak() {
  getSpeakList();
  getVoiceConfig();
  speak();
  // run();
}

japanese_speak::~japanese_speak() {}

std::shared_ptr<ros::Subscriber>
japanese_speak::subscribe(std::string const &topic) {
  ros::NodeHandle nh;
  std::shared_ptr<ros::Subscriber> sub(new ros::Subscriber);
  *sub = nh.subscribe<topic_tools::ShapeShifter>(
      topic, 100, boost::bind(&japanese_speak::callback, this, _1, topic, sub));
  currently_registered_topics.insert(topic);
  num_subscribers_++;

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

void japanese_speak::callback(
    ros::MessageEvent<topic_tools::ShapeShifter const> msg_event,
    std::string const &topic, std::shared_ptr<ros::Subscriber> subscriber) {}

bool japanese_speak::isSubscribed(std::string const &topic) const {
  return currently_registered_topics.find(topic) !=
         currently_registered_topics.end();
}
bool japanese_speak::checkSubscribeTopics(std::string const &topic) {
  if (isSubscribed(topic)) {
    return false;
  }

  if (regex) {
    for (std::string const &regex_str : speak_list_topics) {
      boost::regex e(regex_str);
      boost::smatch what;
      if (boost::regex_match(topic, what, e, boost::match_extra))
        return true;
    }
  } else {
    for (std::string const &t : speak_list_topics)
      if (t == topic)
        return true;
  }

  return false;
}

void japanese_speak::getSpeakList() {
  ros::NodeHandle pnh("~");
  pnh.getParam("topics", speak_list_param);
  // pnh.getParam("regex_topics", speak_list);
  ROS_ASSERT(speak_list_param.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_INFO("speak_list param size: %i", (int)speak_list_param.size());

  for (auto i = 0; i < speak_list_param.size(); ++i) {
    ROS_INFO("ROS Param Load speak_list: %s",
             static_cast<std::string>(speak_list_param[i]["topic"]).c_str());
    speak_list spl;
    spl.topic = static_cast<std::string>(speak_list_param[i]["topic"]);
    spl.sentence = static_cast<std::string>(speak_list_param[i]["sentence"]);
    spl.priority = static_cast<int>(speak_list_param[i]["priority"]);
    speak_list_map.insert(std::make_pair(
        static_cast<std::string>(speak_list_param[i]["topic"]), spl));
  }
}

void japanese_speak::getVoiceConfig() {
  ros::NodeHandle pnh("~");
  pnh.getParam("voice_config", voice_config_param);
  ROS_ASSERT(speak_list_param.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_INFO("voice_config param size: %i", (int)voice_config_param.size());

  voc.additional_half_tone =
      static_cast<double>(voice_config_param["additional_half_tone"]);
  voc.all_pass_constant =
      static_cast<double>(voice_config_param["all_pass_constant"]);
  voc.speech_speed_rate =
      static_cast<double>(voice_config_param["speech_speed_rate"]);
  voc.voice_interval =
      static_cast<double>(voice_config_param["voice_interval"]);
  voc.voice_model = static_cast<std::string>(voice_config_param["voice_model"]);
}
void japanese_speak::run() {
  if (regex) {
    for (std::string const &topic : speak_list_topics)
      subscribe(topic);
  }

  std::thread record_thread;
  record_thread = std::thread(boost::bind(&japanese_speak::speakControl, this));

  ros::Timer check_master_timer;

  ros::NodeHandle nh;
  // check_master_timer = nh.createTimer(
  //     ros::Duration(1.0),
  //     boost::bind(&japanese_speak::doCheckMaster, this, _1,
  //     boost::ref(nh)));

  record_thread.join();
}

void japanese_speak::speakControl() {
  std::lock_guard<std::mutex> lock(mtx);
  thread_local int current_id = 0;
  checkPriority(speak_now);

  // for
  if (true)
    speak();
}

void japanese_speak::speak() {
  auto ret = system(nullptr);
  if (ret != 0)
    ROS_INFO("shell is available on the system!");
  else {
    ROS_ERROR("shell is not available on the system!");
    exit(1);
  }

  std::string open_jtalk =
      "echo " + speak_list_map["/hoge3"].sentence + " | open_jtalk -x " +
      "/var/lib/mecab/dic/open-jtalk/naist-jdic -m " +
      ros::package::getPath("raspicat_speak") + "/voice_model/" +
      voc.voice_model + " -r " + std::to_string(voc.speech_speed_rate) +
      " -ow  /dev/stdout | mpv - & ";

  std::cout << open_jtalk << "\n";

  if (!system(open_jtalk.c_str())) {
  }
}

void japanese_speak::checkPriority(std::set<std::string>) {}

} // namespace raspicat_speak