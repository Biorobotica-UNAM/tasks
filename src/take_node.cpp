#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <string>
#include <vector>
#include <thread>

#include "take_interfaces/srv/take.hpp"
#include "manipulation_tools/Manipulation.hpp"

using Take = take_interfaces::srv::Take;

enum class State
{
    IDLE,
    PLAN_REACH,
    EXECUTE_REACH,
    CLOSE_GRIPPER,
    PLAN_RETRACT,
    EXECUTE_RETRACT,
    DONE,
    ERROR
};

class TakeNode : public rclcpp::Node
{
public:
    TakeNode() : Node("take_node")
    {
        service_ = this->create_service<Take>(
            "/take",
            std::bind(&TakeNode::takeCallback, this,
                      std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "Take node ready");
    }

    void init()
    {
        manipulation_ = std::make_shared<Manipulation>(this->shared_from_this());
    }

private:
    rclcpp::Service<Take>::SharedPtr service_;
    std::shared_ptr<Manipulation> manipulation_;

    // FSM
    State state_ = State::IDLE;

    // Request data
    geometry_msgs::msg::PoseStamped target_pose_;
    std::string planning_group_;

    // Trajectory buffers
    std::vector<std::string> joint_names_;
    std::vector<double> waypoints_;
    std::string message_;

    bool busy_ = false;

    // ---------------------------------------------------------
    // Service callback
    // ---------------------------------------------------------
    void takeCallback(
        const std::shared_ptr<Take::Request> request,
        std::shared_ptr<Take::Response> response)
    {
        if (busy_)
        {
            response->success = false;
            response->message = "Take already running";
            return;
        }

        target_pose_ = request->target_pose;

        if (request->arm == "right")
            planning_group_ = "right_arm_group";
        else if (request->arm == "left")
            planning_group_ = "left_arm_group";
        else
        {
            response->success = false;
            response->message = "Invalid arm";
            return;
        }

        busy_ = true;
        state_ = State::PLAN_REACH;

        // 🔥 correr SM en hilo separado
        std::thread(&TakeNode::runStateMachine, this).detach();

        response->success = true;
        response->message = "Take started";
    }

    // ---------------------------------------------------------
    // State Machine Thread
    // ---------------------------------------------------------
    void runStateMachine()
    {
        while (rclcpp::ok() && busy_)
        {
            switch (state_)
            {
                case State::PLAN_REACH:
                {
                    RCLCPP_INFO(this->get_logger(), "STATE: PLAN_REACH");

                    if (!manipulation_->PlanTrajectory(
                            target_pose_,
                            planning_group_,
                            joint_names_,
                            waypoints_,
                            message_))
                    {
                        error("Plan reach failed");
                        return;
                    }

                    state_ = State::EXECUTE_REACH;
                    break;
                }

                case State::EXECUTE_REACH:
                {
                    RCLCPP_INFO(this->get_logger(), "STATE: EXECUTE_REACH");

                    if (!manipulation_->FollowTrajectory(
                            planning_group_,
                            joint_names_,
                            waypoints_,
                            message_))
                    {
                        error("Execute reach failed");
                        return;
                    }

                    state_ = State::CLOSE_GRIPPER;
                    break;
                }

                case State::CLOSE_GRIPPER:
                {
                    RCLCPP_INFO(this->get_logger(), "STATE: CLOSE_GRIPPER");
                    RCLCPP_INFO(this->get_logger(), "Closing gripper (fake)");

                    std::this_thread::sleep_for(std::chrono::seconds(1));

                    state_ = State::PLAN_RETRACT;
                    break;
                }

                case State::PLAN_RETRACT:
                {
                    RCLCPP_INFO(this->get_logger(), "STATE: PLAN_RETRACT");

                    geometry_msgs::msg::PoseStamped home_pose;
                    home_pose.header.frame_id = "base_link";
                    home_pose.pose.position.x = 0.3;
                    home_pose.pose.position.y = 0.0;
                    home_pose.pose.position.z = 0.8;
                    home_pose.pose.orientation.w = 1.0;

                    if (!manipulation_->PlanTrajectory(
                            home_pose,
                            planning_group_,
                            joint_names_,
                            waypoints_,
                            message_))
                    {
                        error("Plan retract failed");
                        return;
                    }

                    state_ = State::EXECUTE_RETRACT;
                    break;
                }

                case State::EXECUTE_RETRACT:
                {
                    RCLCPP_INFO(this->get_logger(), "STATE: EXECUTE_RETRACT");

                    if (!manipulation_->FollowTrajectory(
                            planning_group_,
                            joint_names_,
                            waypoints_,
                            message_))
                    {
                        error("Execute retract failed");
                        return;
                    }

                    state_ = State::DONE;
                    break;
                }

                case State::DONE:
                {
                    RCLCPP_INFO(this->get_logger(), "STATE: DONE");
                    busy_ = false;
                    return;
                }

                case State::ERROR:
                {
                    RCLCPP_ERROR(this->get_logger(), "STATE: ERROR");
                    busy_ = false;
                    return;
                }

                default:
                    break;
            }
        }
    }

    void error(const std::string& msg)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        state_ = State::ERROR;
    }
};

// ---------------------------------------------------------
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TakeNode>();
    node->init();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
