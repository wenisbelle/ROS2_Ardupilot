// clock_relay: Gazebo clock -> ROS /clock at a lower rate.
//
// Gazebo publishes its clock at every physics step (1 kHz with the 1 ms step
// that ArduPilot lock-step needs). Every ROS node with use_sim_time subscribes
// to /clock, and MAVROS has one node per plugin: at 1 kHz this alone costs about
// one CPU core per drone. 100 Hz (10 ms resolution) is plenty for 20 Hz control,
// timers, TF and sensor stamps (the sensors carry their exact sim time stamps).
//
//   ros2 run robot_gazebo clock_relay --ros-args -p rate:=100.0
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <gz/msgs/clock.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

class ClockRelay : public rclcpp::Node
{
public:
  ClockRelay()
  : Node("clock_relay")
  {
    const double rate = declare_parameter("rate", 100.0);
    const std::string gz_topic = declare_parameter("gz_topic", std::string("/clock"));
    period_ns_ = rate > 0.0 ? static_cast<int64_t>(1e9 / rate) : 0;
    pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    if (!gz_node_.Subscribe(gz_topic, &ClockRelay::on_clock, this)) {
      RCLCPP_FATAL(get_logger(), "cannot subscribe to the Gazebo topic %s", gz_topic.c_str());
      throw std::runtime_error("gz subscribe failed");
    }
    RCLCPP_INFO(get_logger(), "Gazebo %s -> ROS /clock at %.1f Hz", gz_topic.c_str(), rate);
  }

private:
  void on_clock(const gz::msgs::Clock & msg)
  {
    const int64_t t = static_cast<int64_t>(msg.sim().sec()) * 1000000000LL + msg.sim().nsec();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (t < last_ns_) {
        last_ns_ = -1;  // simulation reset: time went backwards
      }
      if (last_ns_ >= 0 && t - last_ns_ < period_ns_) {
        return;
      }
      last_ns_ = t;
    }
    rosgraph_msgs::msg::Clock out;
    out.clock.sec = static_cast<int32_t>(msg.sim().sec());
    out.clock.nanosec = static_cast<uint32_t>(msg.sim().nsec());
    pub_->publish(out);
  }

  gz::transport::Node gz_node_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pub_;
  std::mutex mutex_;
  int64_t period_ns_{0};
  int64_t last_ns_{-1};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClockRelay>());
  rclcpp::shutdown();
  return 0;
}
