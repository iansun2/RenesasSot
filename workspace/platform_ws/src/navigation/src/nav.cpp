#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include "movement_platform_if/srv/goal_request.hpp"
#include "movement_platform_if/srv/goal_status.hpp"

#include <cmath>
#include <chrono>
#include <algorithm>
#include <memory>
#include <mutex>

// Helper function to normalize angles to the range [-PI, PI]
double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

using namespace movement_platform_if::srv;
using namespace geometry_msgs::msg;
using namespace sensor_msgs::msg;

enum class NavState { IDLE, NAVIGATING, HEADING };

class SimpleNavigator : public rclcpp::Node
{
public:
    SimpleNavigator() : Node("simple_navigator")
    {
        // Declare and get parameters
        this->declare_parameter<double>("kp_linear", 1.0); // "Proportional gain for linear velocity."
        this->declare_parameter<double>("kp_angular", 2.5); // "Proportional gain for angular velocity."
        this->declare_parameter<double>("kp_avoidance", 0.01); // "Proportional gain for obstacle avoidance."
        this->declare_parameter<double>("max_linear_vel", 0.2); // "Maximum linear velocity."
        this->declare_parameter<double>("max_angular_vel", 0.8); // "Maximum angular velocity."
        this->declare_parameter<double>("max_linear_accel", 0.01); // "Maximum linear acceleration (m/s^2)."
        this->declare_parameter<double>("max_angular_accel", 0.01); // "Maximum angular acceleration (rad/s^2)."
        this->declare_parameter<double>("goal_dist_tolerance", 0.05); // "Distance tolerance to consider the goal reached."
        this->declare_parameter<double>("lidar_fov_deg", 120.0); // "Field of view for lidar-based avoidance in degrees."
        this->declare_parameter<double>("lidar_head_deg", 90.0);
        this->declare_parameter<double>("avoidance_activation_dist", 0.5); //"Distance at which to start obstacle avoidance."
        this->declare_parameter<double>("final_approach_dist", 0.3);

        kp_linear_ = this->get_parameter("kp_linear").as_double();
        kp_angular_ = this->get_parameter("kp_angular").as_double();
        kp_avoidance_ = this->get_parameter("kp_avoidance").as_double();
        max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
        max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
        max_linear_accel_ = this->get_parameter("max_linear_accel").as_double();
        max_angular_accel_ = this->get_parameter("max_angular_accel").as_double();
        goal_dist_tolerance_ = this->get_parameter("goal_dist_tolerance").as_double();
        lidar_fov_rad_ = this->get_parameter("lidar_fov_deg").as_double() * M_PI / 180.0;
        lidar_head_rad_ = this->get_parameter("lidar_head_deg").as_double() * M_PI / 180.0;
        avoidance_activation_dist_ = this->get_parameter("avoidance_activation_dist").as_double();
        final_approach_dist = this->get_parameter("final_approach_dist").as_double();

        // TF listener and buffer
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Publishers and Subscribers
        cmd_vel_pub_ = this->create_publisher<Twist>("cmd_vel_test", 10);
        scan_sub_ = this->create_subscription<LaserScan>(
            "scan_filted", 10, std::bind(&SimpleNavigator::scan_callback, this, std::placeholders::_1));
        // Service Servers
        goal_request_service_ = this->create_service<GoalRequest>(
            "goal_request", std::bind(&SimpleNavigator::set_goal_callback, this, std::placeholders::_1, std::placeholders::_2));
        goal_status_service_ = this->create_service<GoalStatus>(
            "goal_status", std::bind(&SimpleNavigator::goal_status_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Control loop timer
        control_rate_ = 20.0;
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / control_rate_),
            std::bind(&SimpleNavigator::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Simple Navigator node initialized.");
    }

private:
    // Service callback to set a new goal
    void set_goal_callback(
        const std::shared_ptr<GoalRequest::Request> request,
        std::shared_ptr<GoalRequest::Response> response)
    {
        goal_pose_ = request->goal_pose;
        state_ = NavState::NAVIGATING;
        response->success = true;
        response->message = "New goal has been set.";
        RCLCPP_INFO(this->get_logger(), "New goal received via service: (%.2f, %.2f)", 
                    goal_pose_.position.x, goal_pose_.position.y);
    }

    // Service callback to check the goal status
    void goal_status_callback(
        const std::shared_ptr<GoalStatus::Request> request,
        std::shared_ptr<GoalStatus::Response> response)
    {
        (void)request; // Unused parameter
        response->status = static_cast<int>(state_);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        latest_scan_ = msg;
    }

    void control_loop()
    {
        if (state_ == NavState::IDLE)
        {
            stop_robot();
            return;
        }

        // Get current robot pose from TF
        geometry_msgs::msg::Pose current_pose;
        try {
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            current_pose.position.x = t.transform.translation.x;
            current_pose.position.y = t.transform.translation.y;
            current_pose.orientation = t.transform.rotation;
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform from map to base_link: %s", ex.what());
            stop_robot();
            return;
        }
        
        // Calculate position and angle errors
        double dist_error = std::hypot(goal_pose_.position.x - current_pose.position.x,
                                       goal_pose_.position.y - current_pose.position.y);
        
        double current_yaw = tf2::getYaw(current_pose.orientation);
        double angle_to_goal = std::atan2(goal_pose_.position.y - current_pose.position.y,
                                          goal_pose_.position.x - current_pose.position.x);
        double angle_error = 0;
        if (state_ == NavState::NAVIGATING) {
            angle_error = normalize_angle(angle_to_goal - current_yaw);
        }else if (state_ == NavState::HEADING) {
            double target_yaw = tf2::getYaw(goal_pose_.orientation);
            angle_error = normalize_angle(target_yaw - current_yaw);
        }
        RCLCPP_INFO(this->get_logger(), "pos_err(x,y): (%lf, %lf), angle_to_goal: %lf, current_yaw: %lf, angle_error: %lf", 
            goal_pose_.position.x - current_pose.position.x, goal_pose_.position.y - current_pose.position.y, angle_to_goal, current_yaw, angle_error);

        // Switch avoidance by dist_error
        bool avoidance_en = true;
        if (dist_error < final_approach_dist) {
            avoidance_en = false;
        }

        // Check if goal is reached
        if (state_ == NavState::NAVIGATING && dist_error < goal_dist_tolerance_)
        {
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            state_ = NavState::HEADING;
            // stop_robot();
            // return;
        // Check if head reached
        } else if (state_ == NavState::HEADING && std::abs(angle_error) < 0.1) {
            RCLCPP_INFO(this->get_logger(), "Head reached!");
            state_ = NavState::IDLE;
            stop_robot();
            return;
        }

        // Calculate avoidance velocity from Lidar
        double avoidance_angular_vel = 0.0;
        if (avoidance_en) {
            std::lock_guard<std::mutex> lock(scan_mutex_);
            if (latest_scan_)
            {
                for (size_t i = 0; i < latest_scan_->ranges.size(); ++i)
                {
                    // Relative to head
                    double angle = latest_scan_->angle_min + i * latest_scan_->angle_increment - lidar_head_rad_;
                    // Only consider points within the specified Field of View
                    if (std::abs(angle) > lidar_fov_rad_ / 2.0) continue;

                    double range = latest_scan_->ranges[i];
                    // Check for valid range and if it's close enough to trigger avoidance
                    if (!std::isinf(range) && !std::isnan(range) && range < avoidance_activation_dist_)
                    {
                        // Closer points create a stronger repulsive force.
                        // The -sin(angle) term makes the robot turn away from the obstacle.
                        auto sign = (angle > 0)? 1.0 : -1.0;
                        avoidance_angular_vel += sign * kp_avoidance_ * cos(angle) / (range * range);
                    }
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "p linear: %lf, p augular: %lf, avoidance angular: %lf", 
            kp_linear_ * dist_error, kp_angular_ * angle_error, avoidance_angular_vel);

        // Calculate target velocities based on behavior (avoidance or goal-seeking)
        double target_linear_vel = 0.0;
        double target_angular_vel = 0.0;

        // Proportional control for goal seeking
        // Only move forward if facing the goal to prevent sideways motion
        if (state_ == NavState::NAVIGATING && std::abs(angle_error) < M_PI / 4.0) { // e.g., within 45 degrees
            target_linear_vel = kp_linear_ * dist_error;
        // Turning first
        } else if (state_ == NavState::NAVIGATING) {
            target_linear_vel = 0.01; // Focus on turning first
        // Heading mode
        } else if (state_ == NavState::HEADING) {
            target_linear_vel = 0;
        }
        target_angular_vel = kp_angular_ * angle_error + avoidance_angular_vel;
        
        RCLCPP_INFO(this->get_logger(), "[No Limit] linear: %lf, angular: %lf", target_linear_vel, target_angular_vel);

        // Apply velocity and acceleration limits
        auto cmd_vel = std::make_unique<Twist>();
        double dt = 1.0 / control_rate_;

        // Clamp to maximum velocities
        target_linear_vel = std::clamp(target_linear_vel, -max_linear_vel_, max_linear_vel_);
        target_angular_vel = std::clamp(target_angular_vel, -max_angular_vel_, max_angular_vel_);
        
        // Apply acceleration limits
        double dv_linear = target_linear_vel - last_cmd_vel_.linear.x;
        cmd_vel->linear.x = last_cmd_vel_.linear.x + std::clamp(dv_linear, -max_linear_accel_ * dt, max_linear_accel_ * dt);

        double dv_angular = target_angular_vel - last_cmd_vel_.angular.z;
        cmd_vel->angular.z = last_cmd_vel_.angular.z + std::clamp(dv_angular, -max_angular_accel_ * dt, max_angular_accel_ * dt);

        RCLCPP_INFO(this->get_logger(), "[Final] linear: %lf, angular: %lf", cmd_vel->linear.x, cmd_vel->angular.z);

        // Publish and store the command
        cmd_vel_pub_->publish(std::move(*cmd_vel));
        last_cmd_vel_.linear.x = cmd_vel->linear.x;
        last_cmd_vel_.angular.z = cmd_vel->angular.z;
    }

    void stop_robot()
    {
        if (last_cmd_vel_.linear.x != 0.0 || last_cmd_vel_.angular.z != 0.0) {
            auto cmd_vel = std::make_unique<Twist>();
            cmd_vel->linear.x = 0;
            cmd_vel->angular.z = 0;
            cmd_vel_pub_->publish(std::move(*cmd_vel));
            last_cmd_vel_ = *cmd_vel; // Reset to zero
        }
    }

    // ROS 2 components
    rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Service<GoalRequest>::SharedPtr goal_request_service_;
    rclcpp::Service<GoalStatus>::SharedPtr goal_status_service_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // State
    NavState state_{NavState::IDLE};
    geometry_msgs::msg::Pose goal_pose_;
    Twist last_cmd_vel_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    std::mutex scan_mutex_;

    // Parameters
    double kp_linear_, kp_angular_, kp_avoidance_;
    double max_linear_vel_, max_angular_vel_;
    double max_linear_accel_, max_angular_accel_;
    double goal_dist_tolerance_, lidar_fov_rad_, lidar_head_rad_, avoidance_activation_dist_, final_approach_dist;
    double control_rate_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleNavigator>());
    rclcpp::shutdown();
    return 0;
}
