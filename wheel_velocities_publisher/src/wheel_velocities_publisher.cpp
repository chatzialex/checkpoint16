#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

using namespace std::chrono_literals;

class WheelVelocitiesPublisher : public rclcpp::Node {
public:
  WheelVelocitiesPublisher(
      const std::string &node_name = kNodeName,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions{})
      : Node{node_name, options},
        publisher_{this->create_publisher<std_msgs::msg::Float32MultiArray>(
            kWheelSpeedTopicName, 1)},
        timer_{this->create_wall_timer(
            kTimerPeriod,
            std::bind(&WheelVelocitiesPublisher::timer_cb, this))} {}

  void setWheelVelocities(std_msgs::msg::Float32MultiArray wheel_velocities);

private:
  constexpr static char kNodeName[]{"wheel_velocities_publisher"};
  constexpr static char kWheelSpeedTopicName[]{"wheel_speed"};
  constexpr static auto kTimerPeriod{100ms};

  void timer_cb();

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std_msgs::msg::Float32MultiArray wheel_velocities_;
  mutable std::mutex wheel_velocities_mutex_;
};

void WheelVelocitiesPublisher::timer_cb() {
  std::lock_guard<std::mutex> lock{wheel_velocities_mutex_};
  publisher_->publish(wheel_velocities_);
}

void WheelVelocitiesPublisher::setWheelVelocities(
    std_msgs::msg::Float32MultiArray wheel_velocities) {
  std::lock_guard<std::mutex> lock(wheel_velocities_mutex_);
  wheel_velocities_ = std::move(wheel_velocities);
}

int main(int argc, char **argv) {
  // initialize

  rclcpp::init(argc, argv);
  auto node{std::make_shared<WheelVelocitiesPublisher>()};
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto executor_thread{std::thread([&executor]() { executor.spin(); })};
  std_msgs::msg::Float32MultiArray wheel_velocities;

  // move forward
  RCLCPP_INFO(node->get_logger(), "Moving forward.");
  wheel_velocities.data = {1.0, 1.0, 1.0, 1.0};
  node->setWheelVelocities(wheel_velocities);
  std::this_thread::sleep_for(3s);

  // move backward
  RCLCPP_INFO(node->get_logger(), "Moving backward.");
  wheel_velocities.data = {-1.0, -1.0, -1.0, -1.0};
  node->setWheelVelocities(wheel_velocities);
  std::this_thread::sleep_for(3s);

  // move sideways to the left
  RCLCPP_INFO(node->get_logger(), "Moving sideways to the left.");
  wheel_velocities.data = {-1, 1, -1, 1};
  node->setWheelVelocities(wheel_velocities);
  std::this_thread::sleep_for(3s);

  // move sideways to the right
  RCLCPP_INFO(node->get_logger(), "Moving sideways to the right.");
  wheel_velocities.data = {1, -1, 1, -1};
  node->setWheelVelocities(wheel_velocities);
  std::this_thread::sleep_for(3s);

  // turn clockwise
  RCLCPP_INFO(node->get_logger(), "Turning clockwise.");
  wheel_velocities.data = {1, -1, -1, 1};
  node->setWheelVelocities(wheel_velocities);
  std::this_thread::sleep_for(3s);

  // turn counter-clockwise
  RCLCPP_INFO(node->get_logger(), "Turning counter-clockwise.");
  wheel_velocities.data = {-1, 1, 1, -1};
  node->setWheelVelocities(wheel_velocities);
  std::this_thread::sleep_for(3s);

  // stop
  RCLCPP_INFO(node->get_logger(), "Stopping.");
  wheel_velocities.data = {0.0, 0.0, 0.0, 0.0};
  node->setWheelVelocities(wheel_velocities);

  if (executor_thread.joinable()) {
    executor_thread.join();
  }
  rclcpp::shutdown();
}