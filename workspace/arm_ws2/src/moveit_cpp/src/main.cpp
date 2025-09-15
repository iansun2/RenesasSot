#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Include the service headers
#include "python_moveit_interface/srv/pose_request.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <memory>
#include <string>
#include <vector>
#include <atomic>

// Define shorter aliases for convenience
using PoseRequest = python_moveit_interface::srv::PoseRequest;
using Trigger = std_srvs::srv::Trigger;
using namespace std::placeholders; // for _1, _2

class NamedGoalService : public rclcpp::Node
{
public:
    NamedGoalService() : Node("named_goal_service_cpp")
    {
        // We need to spin up a separate thread for the MoveGroupInterface to work
        // See: https://moveit.ros.org/master/doc/examples/move_group_interface/move_group_interface_tutorial.html
        rclcpp::NodeOptions node_options;
        node_options.automatically_declare_parameters_from_overrides(true);
        auto moveit_node = std::make_shared<rclcpp::Node>(
            "moveit_interface_node", node_options);

        // Run a spinner in a separate thread for MoveIt
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(moveit_node);
        executor_thread_ = std::thread([this]() { this->executor_->spin(); });

        // Initialize MoveGroupInterfaces for arm and gripper
        while(1) {
          try {
            move_group_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(moveit_node, "small_arm");
            move_group_gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(moveit_node, "gripper");
            break;
          } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Waiting for MoveGroupInterface... (%s)", e.what());
          }
        }
        // Set planning parameters if needed (optional)
        // move_group_arm_->setPlanningTime(10.0);
        // move_group_arm_->setMaxVelocityScalingFactor(1.0);
        // move_group_arm_->setMaxAccelerationScalingFactor(1.0);

        // Create the services
        goal_service_ = this->create_service<PoseRequest>(
            "arm_goal_pose",
            std::bind(&NamedGoalService::handle_goal_request, this, _1, _2));

        finish_service_ = this->create_service<Trigger>(
            "arm_goal_finish",
            std::bind(&NamedGoalService::handle_goal_finish, this, _1, _2));
        
        is_moving_.store(false); // Initialize atomic flag

        RCLCPP_INFO(this->get_logger(), "Named goal service is ready.");
    }

    // When the node is destroyed, stop the executor thread
    ~NamedGoalService() {
        executor_->cancel();
        if (executor_thread_.joinable()) {
            executor_thread_.join();
        }
    }


private:
    void handle_goal_request(const std::shared_ptr<PoseRequest::Request> request,
                             std::shared_ptr<PoseRequest::Response> response)
    {
        if (is_moving_.load()) {
            RCLCPP_WARN(this->get_logger(), "Robot is already moving. Ignoring new goal.");
            response->success = false;
            response->message = "Robot is already executing a goal.";
            return;
        }

        is_moving_.store(true);
        moveit::core::MoveItErrorCode move_result;

        // Check if the request is a named target or a pose target
        if (!request->message.empty())
        {
            std::string target_name = request->message;
            RCLCPP_INFO(this->get_logger(), "Received named target: '%s'", target_name.c_str());

            // Choose the correct move group
            if (target_name == "gripper_open" || target_name == "gripper_close")
            {
                move_group_gripper_->setNamedTarget(target_name);
                move_result = move_group_gripper_->move();
            }
            else
            {
                move_group_arm_->setNamedTarget(target_name);
                move_result = move_group_arm_->move();
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Received pose target.");
            move_group_arm_->setPoseTarget(request->target_pose);
            move_result = move_group_arm_->move();
        }

        // Set response based on the result of the move() command
        if (move_result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            response->success = true;
            response->message = "Execution successful.";
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "MoveIt execution failed with error code: %d", move_result.val);
            response->success = false;
            response->message = "Execution failed.";
        }
        
        is_moving_.store(false);
    }

    void handle_goal_finish(const std::shared_ptr<Trigger::Request> request,
                            std::shared_ptr<Trigger::Response> response)
    {
        // With the C++ interface, the move() call is blocking.
        // This service checks our atomic flag to see if a goal is in progress.
        if (is_moving_.load()) {
            response->success = false;
            response->message = "Goal is still running.";
            RCLCPP_INFO(this->get_logger(), "Finish check: Goal is running.");
        } else {
            response->success = true;
            response->message = "Robot is idle.";
            RCLCPP_INFO(this->get_logger(), "Finish check: Robot is idle.");
        }
    }

    rclcpp::Service<PoseRequest>::SharedPtr goal_service_;
    rclcpp::Service<Trigger>::SharedPtr finish_service_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper_;

    // Executor and thread for running MoveIt background tasks
    rclcpp::Executor::SharedPtr executor_;
    std::thread executor_thread_;
    std::atomic<bool> is_moving_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NamedGoalService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
