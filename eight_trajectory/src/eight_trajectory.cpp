#include "geometry_msgs/msg/detail/pose__struct.hpp"
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

using Pose = geometry_msgs::msg::Pose;
using PoseArray = geometry_msgs::msg::PoseArray;
using Quaternion = geometry_msgs::msg::Quaternion;
using Odometry = nav_msgs::msg::Odometry;
using Float32MultiArray = std_msgs::msg::Float32MultiArray;
using Twist = geometry_msgs::msg::Twist;
using namespace std::chrono_literals;

namespace EightTrajectory {

constexpr double kPi{3.1436};

double getYaw(const Quaternion &quaternion) {
  tf2::Quaternion q{quaternion.x, quaternion.y, quaternion.z, quaternion.w};
  double pitch{};
  double roll{};
  double yaw{};
  tf2::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
  return yaw;
}

class EightTrajectory : public rclcpp::Node {
public:
  EightTrajectory(const std::string &node_name = kNodeName,
                  const rclcpp::NodeOptions &options = rclcpp::NodeOptions{})
      : Node{node_name, options}, odom_sub_{this->create_subscription<Odometry>(
                                      kOdometryTopicName, 1,
                                      std::bind(&EightTrajectory::odomSubCb,
                                                this, std::placeholders::_1))},
        wheel_speed_pub_{this->create_publisher<Float32MultiArray>(
            kWheelSpeedTopicName, 1)} {

    RCLCPP_INFO(this->get_logger(), "%s node started.", node_name.c_str());
  }

  bool followTrajectory(const PoseArray &goal_poses);

private:
  constexpr static char kWheelSpeedTopicName[]{"wheel_speed"};
  constexpr static char kNodeName[]{"eight_trajectory"};
  constexpr static char kOdometryTopicName[]{"/odom"};

  constexpr static double kWheelRadius{0.050};       // [m]
  constexpr static double kHalfTrackWidth{0.10709};  // [m]
  constexpr static double kHalfWheelDistance{0.085}; // [m]

  constexpr static double kPositionTolerance{0.1}; // [m]
  constexpr static double kAngleTolerance{0.1};    // [rad]
  constexpr static auto kControlCycle{100ms};
  constexpr static double kMaxLinearVelocity{0.2};  // [m/s]
  constexpr static double kMaxAngularVelocity{0.2}; // [rad/s]

  void odomSubCb(const std::shared_ptr<const Odometry> msg);

  Float32MultiArray twistToWheels(const Twist &twist);
  Twist scaleTwist(const Twist &twist, double /*ds*/, double /*dtheta*/);
  Twist computeTwist(double dx, double dy, double dtheta);
  bool goToPose(const Pose &goal_pose_relative);

  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_{};
  rclcpp::Publisher<Float32MultiArray>::SharedPtr wheel_speed_pub_{};

  double x_cur_{};
  double y_cur_{};
  double theta_cur_{};
};

void EightTrajectory::odomSubCb(const std::shared_ptr<const Odometry> msg) {
  x_cur_ = msg->pose.pose.position.x;
  y_cur_ = msg->pose.pose.position.y;
  theta_cur_ = getYaw(msg->pose.pose.orientation);
}

Float32MultiArray EightTrajectory::twistToWheels(const Twist &twist) {
  const auto w_z{twist.angular.z};
  const auto v_x{twist.linear.x};
  const auto v_y{twist.linear.y};

  Float32MultiArray wheel_velocities{};

  wheel_velocities.data = {
      static_cast<float>(
          ((-kHalfWheelDistance - kHalfTrackWidth) * w_z + v_x - v_y) /
          kWheelRadius),
      static_cast<float>(
          ((kHalfWheelDistance + kHalfTrackWidth) * w_z + v_x + v_y) /
          kWheelRadius),
      static_cast<float>(
          ((kHalfWheelDistance + kHalfTrackWidth) * w_z + v_x - v_y) /
          kWheelRadius),
      static_cast<float>(
          ((-kHalfWheelDistance - kHalfTrackWidth) * w_z + v_x + v_y) /
          kWheelRadius),
  };

  return wheel_velocities;
}

Twist EightTrajectory::scaleTwist(const Twist &twist, double /*ds*/,
                                  double /*dtheta*/) {
  const auto v_norm{std::sqrt(twist.linear.x * twist.linear.x +
                              twist.linear.y * twist.linear.y)};
  const auto v_scale_factor{kMaxLinearVelocity / v_norm};
  const auto w_scale_factor{std::abs(kMaxAngularVelocity / twist.angular.z)};

  Twist twist_scaled;
  twist_scaled.linear.x = v_scale_factor * twist.linear.x;
  twist_scaled.linear.y = v_scale_factor * twist.linear.y;
  twist_scaled.angular.z = w_scale_factor * twist.angular.z;

  return twist_scaled;
}

Twist EightTrajectory::computeTwist(double dx, double dy, double dtheta) {
  const auto ds{std::sqrt(dx * dx + dy * dy)};

  Twist twist{};
  twist.linear.x = dx;
  twist.linear.y = dy;
  twist.angular.z = dtheta;

  return scaleTwist(twist, ds, dtheta);
}

bool EightTrajectory::goToPose(const Pose &goal_pose_relative) {
  const auto x_goal_relative{goal_pose_relative.position.x};
  const auto y_goal_relative{goal_pose_relative.position.y};
  const auto theta_goal_relative{getYaw(goal_pose_relative.orientation)};

  RCLCPP_INFO(this->get_logger(),
              "Received new relative waypoint: dx=%f dy=%f dtheta=%f",
              x_goal_relative, y_goal_relative, theta_goal_relative);

  const auto x_goal{x_goal_relative + x_cur_};
  const auto y_goal{y_goal_relative + y_cur_};
  const auto theta_goal{theta_goal_relative + theta_cur_};

  double dx{}, dy{}, dtheta{};
  for (;;) {
    dx = x_goal - x_cur_;
    dy = y_goal - y_cur_;
    dtheta = std::atan2(std::sin(theta_goal - theta_cur_),
                        std::cos(theta_goal - theta_cur_));

    if (std::abs(dtheta) < kAngleTolerance &&
        std::sqrt(dx * dx + dy * dy) < kPositionTolerance) {
      wheel_speed_pub_->publish(twistToWheels(Twist{}));
      RCLCPP_INFO(this->get_logger(), "Reached waypoint.");
      return true;
    }

    wheel_speed_pub_->publish(twistToWheels(computeTwist(dx, dy, dtheta)));
    rclcpp::sleep_for(kControlCycle);
  }
}

bool EightTrajectory::followTrajectory(const PoseArray &goal_poses) {
  RCLCPP_INFO(this->get_logger(), "Received new goal trajectory.");

  for (const auto &goal_pose : goal_poses.poses) {
    if (!goToPose(goal_pose)) {
      RCLCPP_WARN(this->get_logger(), "Failed to follow goal trajectory.");
      return false;
    }
  }
  RCLCPP_INFO(this->get_logger(), "Reached goal.");
  return true;
}

} // namespace EightTrajectory

PoseArray createGoal() {
  PoseArray goal_poses;
  Pose goal_pose;

  // w1
  goal_pose.orientation.x = 0.0;
  goal_pose.orientation.y = 0.0;
  goal_pose.orientation.z = 0.0;
  goal_pose.orientation.w = 1.0;
  goal_pose.position.x = 1;
  goal_pose.position.y = -1;
  goal_poses.poses.push_back(goal_pose);

  // w2
  // goal_pose.orientation same as before
  goal_pose.position.x = 1;
  goal_pose.position.y = 1;
  goal_poses.poses.push_back(goal_pose);

  // w3
  // goal_pose.orientation same as before
  goal_pose.position.x = 1;
  goal_pose.position.y = 1;
  goal_poses.poses.push_back(goal_pose);

  // w4
  goal_pose.orientation.z = 0.7070727;
  goal_pose.orientation.w = 0.7071408;
  goal_pose.position.x = 1;
  goal_pose.position.y = -1;
  goal_poses.poses.push_back(goal_pose);

  // w5
  goal_pose.orientation.z = -1.0;
  goal_pose.orientation.w = 0.0;
  goal_pose.position.x = -1;
  goal_pose.position.y = -1;
  goal_poses.poses.push_back(goal_pose);

  // w6
  goal_pose.orientation.z = 0.0;
  goal_pose.orientation.w = 1.0;
  goal_pose.position.x = -1;
  goal_pose.position.y = 1;
  goal_poses.poses.push_back(goal_pose);

  // w7
  // goal pose orientation as before
  goal_pose.position.x = -1;
  goal_pose.position.y = 1;
  goal_poses.poses.push_back(goal_pose);

  // w8
  // goal pose orientation as before
  goal_pose.position.x = -1;
  goal_pose.position.y = -1;
  goal_poses.poses.push_back(goal_pose);

  return goal_poses;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node{std::make_shared<EightTrajectory::EightTrajectory>()};
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto executor_thread{std::thread([&executor]() { executor.spin(); })};

  auto goal_poses{createGoal()};
  node->followTrajectory(goal_poses);

  if (executor_thread.joinable()) {
    executor_thread.join();
  }
  rclcpp::shutdown();
}