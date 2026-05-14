#include <chrono>
#include <memory>
#include <vector>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "vision_interfaces/srv/find_seat.hpp"

using namespace std::chrono_literals;

enum class State {
    IDLE,
    ENABLE_DETECTION,
    MOVING_HEAD,
    WAITING_HEAD,
    STABILIZING,
    CALLING_DETECTION,
    WAITING_SERVICE,
    LOOK_AT_TARGET,
    DISABLE_DETECTION,
    SUCCESS, 
    FAILURE
};

class SeatFinderNode : public rclcpp::Node {
public:
    SeatFinderNode() : Node("seat_finder_fsm") {
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = cb_group_;
        
        // --- Publishers ---
        head_gp_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/hardware/head/goal_pose", 1);
        head_look_at_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/hardware/head/look_at", 1);
        
        // --- Subscriptions ---
        head_gr_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/hardware/head/goal_reached", 1,
            std::bind(&SeatFinderNode::head_gr_cb, this, std::placeholders::_1), sub_options);
        
        // --- Service Clients (Actualizado para Jazzy QoS) ---
        vision_client_ = this->create_client<vision_interfaces::srv::FindSeat>("/vision/find_seat", 
            rclcpp::ServicesQoS(), cb_group_);
        enable_client_ = this->create_client<std_srvs::srv::SetBool>("/vision/enable_continuous", 
            rclcpp::ServicesQoS(), cb_group_);

        // --- Server Task ---
        task_service_ = this->create_service<std_srvs::srv::Trigger>("/tasks/find_seat",
            std::bind(&SeatFinderNode::handle_task_request, this, std::placeholders::_1, std::placeholders::_2), 
            rclcpp::ServicesQoS(), cb_group_);

        state_ = State::IDLE;
        timer_ = this->create_wall_timer(300ms, std::bind(&SeatFinderNode::control_loop, this), cb_group_);
        
        RCLCPP_INFO(this->get_logger(), "Seat Finder FSM Node Ready (Jazzy Edition).");
    }

private:
    void handle_task_request(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response) 
    {
        if (state_ != State::IDLE) {
            response->success = false;
            response->message = "Already on a search.";
            return;
        }

        target_pan_ = scan_start_;
        end_of_task_ = false;
        is_final_success_ = false;
        state_ = State::ENABLE_DETECTION;
        
        RCLCPP_INFO(this->get_logger(), "Request received. Starting...");

        rclcpp::Rate loop_rate(10); 
        while (!end_of_task_ && rclcpp::ok()) {
            loop_rate.sleep();
        }

        response->success = is_final_success_;
        response->message = is_final_success_ ? "Free seat found!" : "No free seat found.";
        
        state_ = State::IDLE;
    }

    void control_loop() {
        switch (state_) {
            case State::ENABLE_DETECTION:
                call_set_bool_service(true);
                next_state_after_service_ = State::MOVING_HEAD;
                state_ = State::WAITING_SERVICE;
                break;

            case State::MOVING_HEAD:
                publish_head_goal(target_pan_);
                state_ = State::WAITING_HEAD;
                break;
            
            case State::STABILIZING:
                if (++stabilize_count_ > 2) {
                    stabilize_count_ = 0;
                    state_ = State::CALLING_DETECTION;
                }
                break;

            case State::CALLING_DETECTION:
                call_vision_service();
                state_ = State::WAITING_SERVICE;
                break;

            case State::LOOK_AT_TARGET:
                final_goal.header.stamp = this->now();
                head_look_at_pub_->publish(final_goal);
                is_final_success_ = true;
                state_ = State::DISABLE_DETECTION;
                break;

            case State::DISABLE_DETECTION:
                call_set_bool_service(false);
                next_state_after_service_ = is_final_success_ ? State::SUCCESS : State::FAILURE;
                state_ = State::WAITING_SERVICE;
                break;

            case State::SUCCESS:
            case State::FAILURE:
                end_of_task_ = true;
                break;

            default:
                break;
        }
    }

    void call_set_bool_service(bool value) {
        if (!enable_client_->wait_for_service(1s)) {
            state_ = State::FAILURE;
            return;
        }
        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = value;

        // CORRECCIÓN: Usar el SharedFuture del cliente
        enable_client_->async_send_request(req, [this, value](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
            auto res = future.get();
            if (res->success) {
                this->state_ = this->next_state_after_service_;
            } else {
                this->state_ = State::FAILURE;
            }
        });
    }

    void call_vision_service() {
        if (!vision_client_->wait_for_service(1s)) {
            state_ = State::FAILURE;
            return;
        }
        auto req = std::make_shared<vision_interfaces::srv::FindSeat::Request>();

        vision_client_->async_send_request(req, [this](rclcpp::Client<vision_interfaces::srv::FindSeat>::SharedFuture future) {
            auto res = future.get();
            if (res->success) {
                this->final_goal = res->point;
                this->state_ = State::LOOK_AT_TARGET;
            } else {
                this->next_step();
            }
        });
    }

    void next_step() {
        target_pan_ += scan_step_;
        if (target_pan_ > scan_end_) {
            state_ = State::DISABLE_DETECTION;
        } else {
            state_ = State::MOVING_HEAD;
        }
    }

    void publish_head_goal(double pan_val) {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = {pan_val, tilt_val};
        head_gp_pub_->publish(msg);
    }

    void head_gr_cb(const std_msgs::msg::Bool::SharedPtr msg) {
        if (state_ == State::WAITING_HEAD && msg->data) {
            state_ = State::STABILIZING;
        }
    }

    State state_;
    State next_state_after_service_;
    float target_pan_;
    const float tilt_val = -0.5;
    uint8_t stabilize_count_ = 0;
    geometry_msgs::msg::PointStamped final_goal;
    bool end_of_task_ = false;
    bool is_final_success_ = false;
    float scan_start_ = -1.57;
    float scan_end_ = 1.57;
    float scan_step_ = 0.52;

    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr head_gp_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr head_gr_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr head_look_at_pub_;
    rclcpp::Client<vision_interfaces::srv::FindSeat>::SharedPtr vision_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr task_service_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SeatFinderNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}