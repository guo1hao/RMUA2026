#include <ros/ros.h>
#include <rmua_msgs/SensorHubStatus.h>
#include <topic_tools/shape_shifter.h>

#include <boost/bind.hpp>

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

struct RelaySpec {
  std::string key;
  std::string source_topic;
  std::string target_topic;
  bool latch;
};

std::string TrimSlashes(const std::string& input) {
  if (input.empty()) {
    return std::string();
  }

  std::size_t begin = 0;
  while (begin < input.size() && input[begin] == '/') {
    ++begin;
  }

  std::size_t end = input.size();
  while (end > begin && input[end - 1] == '/') {
    --end;
  }

  return input.substr(begin, end - begin);
}

std::string JoinTopic(const std::string& left, const std::string& right) {
  const std::string left_trimmed = TrimSlashes(left);
  const std::string right_trimmed = TrimSlashes(right);

  if (left_trimmed.empty() && right_trimmed.empty()) {
    return std::string("/");
  }

  if (left_trimmed.empty()) {
    return std::string("/") + right_trimmed;
  }

  if (right_trimmed.empty()) {
    return std::string("/") + left_trimmed;
  }

  return std::string("/") + left_trimmed + "/" + right_trimmed;
}

class TopicRelay {
 public:
  TopicRelay(ros::NodeHandle& nh,
             const RelaySpec& spec,
             int queue_size,
             std::function<void(const std::string&, const ros::Time&)> on_message)
      : nh_(nh),
        spec_(spec),
        queue_size_(queue_size),
        on_message_(std::move(on_message)),
        advertised_(false) {
    subscriber_ = nh_.subscribe<topic_tools::ShapeShifter>(
        spec_.source_topic,
        queue_size_,
        boost::bind(&TopicRelay::MessageCallback, this, _1));
  }

 private:
  void MessageCallback(const topic_tools::ShapeShifter::ConstPtr& message) {
    if (!advertised_) {
      AdvertiseFromMessage(*message);
    }

    publisher_.publish(message);

    if (on_message_) {
      on_message_(spec_.key, ros::Time::now());
    }
  }

  void AdvertiseFromMessage(const topic_tools::ShapeShifter& message) {
    ros::AdvertiseOptions options(
        spec_.target_topic,
        static_cast<uint32_t>(queue_size_),
        message.getMD5Sum(),
        message.getDataType(),
        message.getMessageDefinition());
    options.latch = spec_.latch;

    publisher_ = nh_.advertise(options);
    advertised_ = true;

    ROS_INFO_STREAM("Relay ready: " << spec_.source_topic
                    << " -> " << spec_.target_topic
                    << " [" << message.getDataType() << "]");
  }

  ros::NodeHandle& nh_;
  RelaySpec spec_;
  int queue_size_;
  std::function<void(const std::string&, const ros::Time&)> on_message_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  bool advertised_;
};

class SensorHubNode {
 public:
  SensorHubNode() : nh_(), pnh_("~") {
    pnh_.param<std::string>("drone_name", drone_name_, "drone_1");
    pnh_.param<std::string>("source_root", source_root_, "/airsim_node");
    pnh_.param<std::string>("relay_root", relay_root_, "/rmua/sensors");
    pnh_.param("queue_size", queue_size_, 10);
    pnh_.param("status_publish_rate", status_publish_rate_hz_, 2.0);

    if (queue_size_ < 1) {
      queue_size_ = 10;
    }
    if (status_publish_rate_hz_ <= 0.0) {
      status_publish_rate_hz_ = 2.0;
    }

    relay_drone_root_ = JoinTopic(relay_root_, drone_name_);
    status_publisher_ = nh_.advertise<rmua_msgs::SensorHubStatus>(
        JoinTopic(relay_drone_root_, "status"), 1, true);

    const std::vector<RelaySpec> relay_specs = BuildRelaySpecs();
    relay_specs_ = relay_specs;

    for (const RelaySpec& spec : relay_specs_) {
      last_received_[spec.key] = ros::Time();
      relays_.emplace_back(new TopicRelay(
          nh_,
          spec,
          queue_size_,
          [this](const std::string& key, const ros::Time& stamp) {
            RecordReception(key, stamp);
          }));
    }

    status_timer_ = nh_.createTimer(
        ros::Duration(1.0 / status_publish_rate_hz_),
        &SensorHubNode::PublishStatus,
        this);

    PublishStatus(ros::TimerEvent());

    ROS_INFO_STREAM("Sensor hub is relaying topics into " << relay_drone_root_);
  }

 private:
  std::vector<RelaySpec> BuildRelaySpecs() const {
    const std::string drone_source_root = JoinTopic(source_root_, drone_name_);

    std::vector<RelaySpec> specs;
    specs.push_back({"front_left_scene", JoinTopic(drone_source_root, "front_left/Scene"), JoinTopic(relay_drone_root_, "front_left/scene"), false});
    specs.push_back({"front_right_scene", JoinTopic(drone_source_root, "front_right/Scene"), JoinTopic(relay_drone_root_, "front_right/scene"), false});
    specs.push_back({"back_left_scene", JoinTopic(drone_source_root, "back_left/Scene"), JoinTopic(relay_drone_root_, "back_left/scene"), false});
    specs.push_back({"back_right_scene", JoinTopic(drone_source_root, "back_right/Scene"), JoinTopic(relay_drone_root_, "back_right/scene"), false});
    specs.push_back({"imu", JoinTopic(drone_source_root, "imu/imu"), JoinTopic(relay_drone_root_, "imu"), false});
    specs.push_back({"lidar", JoinTopic(drone_source_root, "lidar"), JoinTopic(relay_drone_root_, "lidar"), false});
    specs.push_back({"pose_gt", JoinTopic(drone_source_root, "debug/pose_gt"), JoinTopic(relay_drone_root_, "pose_gt"), false});
    specs.push_back({"gps", JoinTopic(drone_source_root, "gps"), JoinTopic(relay_drone_root_, "gps"), false});
    specs.push_back({"wind", JoinTopic(drone_source_root, "debug/wind"), JoinTopic(relay_drone_root_, "debug/wind"), false});
    specs.push_back({"rotor_pwm", JoinTopic(drone_source_root, "debug/rotor_pwm"), JoinTopic(relay_drone_root_, "debug/rotor_pwm"), false});
    specs.push_back({"initial_pose", JoinTopic(source_root_, "initial_pose"), JoinTopic(relay_drone_root_, "initial_pose"), true});
    specs.push_back({"end_goal", JoinTopic(source_root_, "end_goal"), JoinTopic(relay_drone_root_, "end_goal"), true});
    return specs;
  }

  void RecordReception(const std::string& key, const ros::Time& stamp) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    last_received_[key] = stamp;
  }

  bool IsReady(const std::string& key) const {
    const std::unordered_map<std::string, ros::Time>::const_iterator it = last_received_.find(key);
    return it != last_received_.end() && !it->second.isZero();
  }

  ros::Time LastStamp(const std::string& key) const {
    const std::unordered_map<std::string, ros::Time>::const_iterator it = last_received_.find(key);
    if (it == last_received_.end()) {
      return ros::Time();
    }
    return it->second;
  }

  void PublishStatus(const ros::TimerEvent&) {
    rmua_msgs::SensorHubStatus status;
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      status.header.stamp = ros::Time::now();
      status.drone_name = drone_name_;

      status.front_left_scene_ready = IsReady("front_left_scene");
      status.front_right_scene_ready = IsReady("front_right_scene");
      status.back_left_scene_ready = IsReady("back_left_scene");
      status.back_right_scene_ready = IsReady("back_right_scene");
      status.imu_ready = IsReady("imu");
      status.lidar_ready = IsReady("lidar");
      status.pose_gt_ready = IsReady("pose_gt");
      status.gps_ready = IsReady("gps");
      status.wind_ready = IsReady("wind");
      status.rotor_pwm_ready = IsReady("rotor_pwm");
      status.initial_pose_ready = IsReady("initial_pose");
      status.end_goal_ready = IsReady("end_goal");

      status.front_left_scene_last_received = LastStamp("front_left_scene");
      status.front_right_scene_last_received = LastStamp("front_right_scene");
      status.back_left_scene_last_received = LastStamp("back_left_scene");
      status.back_right_scene_last_received = LastStamp("back_right_scene");
      status.imu_last_received = LastStamp("imu");
      status.lidar_last_received = LastStamp("lidar");
      status.pose_gt_last_received = LastStamp("pose_gt");
      status.gps_last_received = LastStamp("gps");
      status.wind_last_received = LastStamp("wind");
      status.rotor_pwm_last_received = LastStamp("rotor_pwm");
      status.initial_pose_last_received = LastStamp("initial_pose");
      status.end_goal_last_received = LastStamp("end_goal");
    }

    status_publisher_.publish(status);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher status_publisher_;
  ros::Timer status_timer_;
  std::string drone_name_;
  std::string source_root_;
  std::string relay_root_;
  std::string relay_drone_root_;
  int queue_size_;
  double status_publish_rate_hz_;
  std::vector<RelaySpec> relay_specs_;
  std::vector<std::unique_ptr<TopicRelay>> relays_;
  std::unordered_map<std::string, ros::Time> last_received_;
  mutable std::mutex status_mutex_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "sensor_hub_node");
  SensorHubNode node;
  ros::spin();
  return 0;
}
