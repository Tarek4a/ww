#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

using namespace std::chrono_literals;

enum class RobotState { MOVING_3M, TURNING_180, MOVING_5M, STOPPED };

class RobotMotion : public rclcpp::Node {
public:
    RobotMotion() : Node("robot_motion_node"), current_state_(RobotState::MOVING_3M) {
        // تأكد من استخدام نفس الـ Topic اللي جربته في الـ Terminal
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/diff_drive_base_controller/cmd_vel", 10);
        
        // جربنا هنا QoS متوافق مع أغلب محركات المحاكاة
       subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
         "/diff_drive_base_controller/odom", 10, 
           std::bind(&RobotMotion::odomCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(100ms, std::bind(&RobotMotion::controlLoop, this));
        
        first_run_ = true;
        RCLCPP_INFO(this->get_logger(), "Node Started. Waiting for Odometry...");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (first_run_) {
            RCLCPP_INFO(this->get_logger(), "Odometry received! Starting Movement...");
            start_pose_ = msg->pose.pose;
            first_run_ = false;
        }
        current_pose_ = msg->pose.pose;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double r, p;
        m.getRPY(r, p, current_yaw_);
    }

    void controlLoop() {
        // لو مفيش اودومتري وصل، مش هيتحرك
        if (first_run_) {
        // السطر ده هيعرفنا لو النود لسه مستنية بيانات الاودومتري
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for Odom data...");
        return; 
    
        }
        // السطر ده هيطبع لنا المسافة الحالية في الـ Terminal عشان نشوفها بتزيد ولا لا
         RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Current Distance: %f", getDistance());

        auto message = geometry_msgs::msg::TwistStamped();
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "base_link"; // نفس اللي نفع في الـ pub

        switch (current_state_) {
            case RobotState::MOVING_3M:
                if (getDistance() < 3.0) {
                    message.twist.linear.x = 0.3; 
                } else {
                    RCLCPP_INFO(this->get_logger(), "3 Meters reached. Turning...");
                    resetReference();
                    current_state_ = RobotState::TURNING_180;
                }
                break;

             case RobotState::TURNING_180:
                       RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Current Yaw Diff: %f", getAngleDiff());
                  // نلف لحد ما الفرق يوصل لـ 3.0 راديان (حوالي 172 درجة) لضمان الاستقرار
                  if (std::abs(getAngleDiff()) < 3.0) {
                       message.twist.angular.z = 0.4; // سرعة دوران هادية
                         } else {
                      RCLCPP_INFO(this->get_logger(), "Turn finished! Current Yaw Diff: %f. Moving 5m...", getAngleDiff());
                      resetReference();
                    current_state_ = RobotState::MOVING_5M;
                  }
                break;

            case RobotState::MOVING_5M:
                if (getDistance() < 5.0) {
                    message.twist.linear.x = 0.3;
                } else {
                    RCLCPP_INFO(this->get_logger(), "5 Meters reached. Stopping.");
                    current_state_ = RobotState::STOPPED;
                }
                break;

            case RobotState::STOPPED:
                message.twist.linear.x = 0.0;
                message.twist.angular.z = 0.0;
                break;
        }
        publisher_->publish(message);
    }

    double getDistance() {
        return std::sqrt(std::pow(current_pose_.position.x - start_pose_.position.x, 2) +
                         std::pow(current_pose_.position.y - start_pose_.position.y, 2));
    }

    double getAngleDiff() {
      double diff = current_yaw_ - start_yaw_;
      // التأكد إن الفرق دائماً في نطاق [-PI, PI]
      while (diff > M_PI) diff -= 2.0 * M_PI;
      while (diff < -M_PI) diff += 2.0 * M_PI;
      return diff;
    }

    void resetReference() {
        start_pose_ = current_pose_;
        start_yaw_ = current_yaw_;
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Pose current_pose_, start_pose_;
    double current_yaw_, start_yaw_ = 0.0;
    bool first_run_;
    RobotState current_state_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotMotion>());
    rclcpp::shutdown();
    return 0;
}