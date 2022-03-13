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

//     foreach (string const &topic, options_.topics) subscribe(topic);

// #include <algorithm>

// #include "XmlRpc.h"

#include "raspicat_speak/japanese_speak.hpp"

#include <mutex>
#include <thread.hpp>

namespace raspicat_speak {

japanese_speak::japanese_speak() { run(); }

std::shared_ptr<ros::Subscriber>
japanese_speak::subscribe(std::string const &topic) {
  ros::NodeHandle nh;
  std::shared_ptr<ros::Subscriber> sub(new ros::Subscriber);
  *sub = nh.subscribe<topic_tools::ShapeShifter>(
      topic, 100, boost::bind(&japanese_speak::doSpeak, this, _1, topic, sub));
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
    std::string const &topic, std::shared_ptr<ros::Subscriber> subscriber) {
  std::lock_guard<std::mutex> lock(mtx);
  thread_local int current_id = 0;
}

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
  //     boost::bind(&japanese_speak::doCheckMaster, this, _1, boost::ref(nh)));

  record_thread.join();
}

void japanese_speak::speak() {}

} // namespace raspicat_speak