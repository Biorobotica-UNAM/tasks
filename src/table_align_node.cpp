#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <atomic>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "vision_tools/Vision.hpp"

using namespace std::chrono_literals;

enum class State { IDLE, START_TASK, MOVING_HEAD, WAITING_HEAD, GET_ALIGNMENT, ROTATE, STABILIZING, SUCCESS, FAILURE };

class TableAlignSM {
public:
  TableAlignSM(rclcpp::Node::SharedPtr node) : node_(node) {
    cb_group_ =
        node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = cb_group_;

    vision_ = std::make_shared<Vision>(node_);
    pub_cmd_vel_ = node_->create_publisher<geometry_msgs::msg::Twist>(
        "/hardware/base/cmd_vel", rclcpp::QoS(10));

    // Head control topics
    head_gp_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/hardware/head/goal_pose", 1);
    head_gr_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/hardware/head/goal_reached", 1,
        std::bind(&TableAlignSM::head_goal_reached_cb, this, std::placeholders::_1),
        sub_options);

    task_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "/tasks/table_align",
        std::bind(&TableAlignSM::handle_task_request, this,
                  std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, cb_group_);

    state_ = State::IDLE;
    timer_ = node_->create_wall_timer(
        66ms, // ~15 Hz
        std::bind(&TableAlignSM::control_loop, this), cb_group_);

    RCLCPP_INFO(node_->get_logger(), "Table Align service ready.");
  }

private:
  void handle_task_request(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (state_ != State::IDLE) {
      RCLCPP_INFO(node_->get_logger(),
                  "Alignment request received but already running; waiting for current run to finish...");
      while (!end_of_task_ && rclcpp::ok()) {
        std::this_thread::sleep_for(100ms);
      }

      response->success = last_success_.load();
      response->message = response->success ? "Alignment completed"
                                            : "Alignment failed or aborted";
      RCLCPP_INFO(node_->get_logger(), "Alignment finished, response sent to waiting client");
      return;
    }

    RCLCPP_INFO(node_->get_logger(), "Alignment request received. Starting...");
    state_ = State::START_TASK;

    rclcpp::Rate loop_rate(10);
    end_of_task_ = false;
    while (!end_of_task_ && rclcpp::ok()) {
      std::this_thread::sleep_for(100ms);
    }

    response->success = (state_ == State::SUCCESS);
    response->message = response->success ? "Alignment completed"
                                          : "Alignment failed or aborted";

    state_ = State::IDLE;
    RCLCPP_INFO(node_->get_logger(), "Alignment task completed, response sent");
  }

  void publish_stop() {
    geometry_msgs::msg::Twist cmd;
    pub_cmd_vel_->publish(cmd);
  }

  void publish_head_goal(double pan, double tilt) {
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = {pan, tilt};
    head_gp_pub_->publish(msg);
  }

  void head_goal_reached_cb(const std_msgs::msg::Bool::SharedPtr msg) {
    if (state_ == State::WAITING_HEAD && msg->data) {
      state_ = State::GET_ALIGNMENT;
    }
  }

  void control_loop() {
    switch (state_) {
    case State::IDLE:
      break;

    case State::START_TASK:
      RCLCPP_INFO(node_->get_logger(), "[START]");
      //publish_stop();
      start_time_ = std::chrono::steady_clock::now();
      state_ = State::MOVING_HEAD;
      break;

    case State::MOVING_HEAD:
      RCLCPP_INFO(node_->get_logger(), "[MOVING_HEAD] Moving to pan=0.0, tilt=-0.6");
      publish_head_goal(0.0, -0.6);
      state_ = State::WAITING_HEAD;
      break;

    case State::WAITING_HEAD:
      // Waiting for head to reach goal_reached callback will transition to GET_ALIGNMENT
      break;

    case State::GET_ALIGNMENT:
      angle_ = vision_->GetTableAlignment();
      RCLCPP_INFO(node_->get_logger(), "[ALIGNMENT] angle=%f", angle_);
      if (std::fabs(angle_) <= kAngleTolerance_) {
        state_ = State::SUCCESS;
      } else {
        //stabilizing_stop_sent_ = false;
        state_ = State::ROTATE;
      }
      break;

    case State::ROTATE: {
      /*if (std::chrono::steady_clock::now() - start_time_ > kMaxDuration_) {
        RCLCPP_WARN(node_->get_logger(), "[TIMEOUT]");
        state_ = State::FAILURE;
        break;  
      }*/
      
      angle_ = vision_->GetTableAlignment();
      //RCLCPP_INFO(node_->get_logger(), "[ROTATE] angle=%f", angle_);

      if (std::fabs(angle_) <= kAngleTolerance_) {
        state_ = State::SUCCESS;
        break;
      }

      geometry_msgs::msg::Twist cmd;
      if (angle_ > 0.0) 
        cmd.angular.z = -0.2;
      else
        cmd.angular.z = 0.2;
      //cmd.angular.z = (angle_ > 0.0) ? -0.2 : 0,2;
      RCLCPP_INFO(node_->get_logger(), "[ANGLE] angle=%f", angle_ );
      RCLCPP_INFO(node_->get_logger(), "[ROTATE] angle=%f", cmd.angular.z );
      pub_cmd_vel_->publish(cmd);
      if (++wait_count > 10000) { 
            wait_count = 0;
            state_ = State::GET_ALIGNMENT;
        }
      //state_ = State::STABILIZING;
      break;
    }

    case State::STABILIZING: {
        if (++wait_count > 1000) { 
            wait_count = 0;
            state_ = State::GET_ALIGNMENT;
        }
        //publish_stop(); 
        break;
    }

    case State::SUCCESS:
      RCLCPP_INFO(node_->get_logger(), "[FINISH]");
      //publish_stop();
      last_success_ = true;
      end_of_task_ = true;
      break;

    case State::FAILURE:
      RCLCPP_WARN(node_->get_logger(), "[ABORT]");
      //publish_stop();
      last_success_ = false;
      end_of_task_ = true;
      break;
    }
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  std::shared_ptr<Vision> vision_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr head_gp_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr head_gr_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr task_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  State state_;
  std::atomic<bool> end_of_task_{false};
  std::atomic<bool> last_success_{false};
  double angle_ = 0.0;
  int wait_count = 0;
  std::chrono::time_point<std::chrono::steady_clock> start_time_;

  const double kAngularSpeed_ = 0.1;
  const double kAngleTolerance_ = 2.0;
  const std::chrono::seconds kMaxDuration_{15};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("table_align");
  auto sm = std::make_shared<TableAlignSM>(node);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    4);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}