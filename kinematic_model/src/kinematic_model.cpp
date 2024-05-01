#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "std_msgs/msg/detail/float32_multi_array__struct.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <memory>

class KinematicModel : public rclcpp::Node {
public:
  KinematicModel(const std::string &node_name = kNodeName,
                 const rclcpp::NodeOptions &options = rclcpp::NodeOptions{})
      : Node{node_name, options},
        subscriber_{this->create_subscription<std_msgs::msg::Float32MultiArray>(
            kWheelSpeedTopicName, 1,
            std::bind(&KinematicModel::subscriber_cb, this,
                      std::placeholders::_1))},
        publisher_{this->create_publisher<geometry_msgs::msg::Twist>(
            kCommandVelocityTopicName, 1)} {}

private:
  constexpr static char kNodeName[]{"kinematic_model"};
  constexpr static char kWheelSpeedTopicName[]{"/wheel_speed"};
  constexpr static char kCommandVelocityTopicName[]{"cmd_vel"};

  constexpr static double kWheelRadius{0.050};       // [m]
  constexpr static double kHalfTrackWidth{0.10709};  // [m]
  constexpr static double kHalfWheelDistance{0.085}; // [m]
  Eigen::Matrix<double, 4, 3> twist_to_wheels_ =
      (1.0 / kWheelRadius) *
      (Eigen::Matrix<double, 4, 3>() << -kHalfWheelDistance - kHalfTrackWidth,
       1.0, -1.0, kHalfWheelDistance + kHalfTrackWidth, 1.0, 1.0,
       kHalfWheelDistance + kHalfTrackWidth, 1.0, -1.0,
       -kHalfWheelDistance - kHalfTrackWidth, 1.0, 1.0)
          .finished();

  void subscriber_cb(
      const std::shared_ptr<const std_msgs::msg::Float32MultiArray> msg);
  geometry_msgs::msg::Twist
  toTwist(const std_msgs::msg::Float32MultiArray &wheel_velocities) const;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      subscriber_{};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{};
};

void KinematicModel::subscriber_cb(
    const std::shared_ptr<const std_msgs::msg::Float32MultiArray> msg) {
  if (msg->data.size() != 4) {
    RCLCPP_WARN(this->get_logger(),
                "Received wheel velocities message with %zu values, expected "
                "%d. Ignoring.",
                msg->data.size(), 4);
    return;
  }
  publisher_->publish(toTwist(*msg));
}

geometry_msgs::msg::Twist KinematicModel::toTwist(
    const std_msgs::msg::Float32MultiArray &wheel_velocities) const {
  geometry_msgs::msg::Twist twist;

  // solving the linear system Ax = b instead of directly using the first three
  // wheel velocities to minimize the effect of numerical error

  Eigen::Vector4d u{wheel_velocities.data[0], wheel_velocities.data[1],
                    wheel_velocities.data[2], wheel_velocities.data[3]};
  Eigen::Vector3d v;
  v = twist_to_wheels_.colPivHouseholderQr().solve(u);
  twist.angular.z = v(0);
  twist.linear.x = v(1);
  twist.linear.y = v(2);

  return twist;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<KinematicModel>());

  rclcpp::shutdown();
}