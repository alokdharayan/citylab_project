#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node"), turning_(false) {
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&Patrol::laserCallback, this, std::placeholders::_1));

    cmd_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ =
        this->create_wall_timer(100ms, std::bind(&Patrol::controlLoop, this));
  }

private:
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    int front = msg->ranges.size() / 2;

    // Trigger turn early
    if (!turning_ && msg->ranges[front] < 0.6) {
      turning_ = true;
      turn_start_time_ = this->now();
    }
  }

  void controlLoop() {
    geometry_msgs::msg::Twist cmd;

    if (turning_) {
      // TURN for a FIXED duration
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.8;

      // Stop turning after 1 second
      if ((this->now() - turn_start_time_).seconds() > 1.0) {
        turning_ = false;
      }
    } else {
      // Move forward
      cmd.linear.x = 0.12;
      cmd.angular.z = 0.0;
    }

    cmd_pub_->publish(cmd);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool turning_;
  rclcpp::Time turn_start_time_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}
