#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "vision_tools/Vision.hpp"

using namespace std::chrono_literals;

enum class State { IDLE, START_TASK, GET_ALIGNMENT, ROTATE, STABILIZING, SUCCESS, FAILURE };

class TableAlignSM {
public:
  TableAlignSM(rclcpp::Node::SharedPtr node) : node_(node) {
    cb_group_ =
        node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    vision_ = std::make_shared<Vision>(node_);
    pub_cmd_vel_ = node_->create_publisher<geometry_msgs::msg::Twist>(
        "/hardware/base/cmd_vel", rclcpp::QoS(10));

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
      response->success = false;
      response->message = "Already aligning.";
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

  void control_loop() {
    switch (state_) {
    case State::IDLE:
      break;

    case State::START_TASK:
      RCLCPP_INFO(node_->get_logger(), "[START]");
      publish_stop();
      start_time_ = std::chrono::steady_clock::now();
      state_ = State::GET_ALIGNMENT;
      break;

    case State::GET_ALIGNMENT:
      angle_ = vision_->getTableAlignment();
      RCLCPP_INFO(node_->get_logger(), "[ALIGNMENT] angle=%f", angle_);
      if (std::fabs(angle_) <= kAngleTolerance_) {
        state_ = State::SUCCESS;
      } else {
        state_ = State::ROTATE;
      }
      break;

    case State::ROTATE: {
      if (std::chrono::steady_clock::now() - start_time_ > kMaxDuration_) {
        RCLCPP_WARN(node_->get_logger(), "[TIMEOUT]");
        state_ = State::FAILURE;
        break;
      }
      geometry_msgs::msg::Twist cmd;
      cmd.angular.z = (angle_ > 0.0) ? -kAngularSpeed_ : kAngularSpeed_;
      pub_cmd_vel_->publish(cmd);
      state_ = State::STABILIZING;
      break;
    }

    case State::STABILIZING: {
        publish_stop(); 
        if (++wait_count > 5) { 
            wait_count = 0;
            state_ = State::GET_ALIGNMENT;
        }
        break;
    }

    case State::SUCCESS:
      RCLCPP_INFO(node_->get_logger(), "[FINISH]");
      publish_stop();
      end_of_task_ = true;
      break;

    case State::FAILURE:
      RCLCPP_WARN(node_->get_logger(), "[ABORT]");
      publish_stop();
      end_of_task_ = true;
      break;
    }
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  std::shared_ptr<Vision> vision_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr task_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  State state_;
  std::atomic<bool> end_of_task_{false};
  double angle_ = 0.0;
  int wait_count = 0;
  std::chrono::time_point<std::chrono::steady_clock> start_time_;

  const double kAngularSpeed_ = 0.1;
  const double kAngleTolerance_ = 0.02;
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