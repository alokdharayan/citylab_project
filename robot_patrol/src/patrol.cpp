#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

class Patrol : public rclcpp::Node
{
public:
    Patrol() : Node("patrol_node"), direction_(0.0)
    {
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&Patrol::laserCallback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Patrol::controlLoop, this));
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        int start = msg->ranges.size() / 4;
        int end   = 3 * msg->ranges.size() / 4;

        int front = msg->ranges.size() / 2;

        if (msg->ranges[front] < 0.35)
        {
            double max_dist = 0.0;
            int max_index = front;

            for (int i = start; i <= end; i++)
            {
                if (std::isfinite(msg->ranges[i]) && msg->ranges[i] > max_dist)
                {
                    max_dist = msg->ranges[i];
                    max_index = i;
                }
            }

            direction_ = msg->angle_min +
                         max_index * msg->angle_increment;
        }
        else
        {
            direction_ = 0.0;
        }
    }

    void controlLoop()
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.1;
        cmd.angular.z = direction_ / 2.0;
        cmd_pub_->publish(cmd);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double direction_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Patrol>());
    rclcpp::shutdown();
    return 0;
}
