#include <chrono>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "vision_interfaces/srv/find_seat.hpp"

using namespace std::chrono_literals;

enum class State {
    IDLE,
    MOVING_HEAD,
    WAITING_HEAD,
    STABILIZING,
    CALLING_DETECTION,
    WAITING_SERVICE,
    LOOK_AT_TARGET,
    SUCCESS, 
    FAILURE
};

class SeatFinderNode : public rclcpp::Node {
public:
    SeatFinderNode() : Node("seat_finder_fsm") {
        // --- Callback group ---
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = cb_group_;
        
  
        // --- Head control topics ---
        head_gp_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/hardware/head/goal_pose", 1);
        head_gr_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/hardware/head/goal_reached", 1,
            std::bind(&SeatFinderNode::head_gr_cb, this, std::placeholders::_1), sub_options);
        head_look_at_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/hardware/head/look_at", 1);
        
        // --- Vision service ---
        vision_client_ = this->create_client<vision_interfaces::srv::FindSeat>("/vision/find_seat", 
            rmw_qos_profile_services_default, cb_group_);

        // --- Task service ---
        task_service_ = this->create_service<std_srvs::srv::Trigger>("/tasks/find_seat",
            std::bind(&SeatFinderNode::handle_task_request, this, std::placeholders::_1, std::placeholders::_2), 
            rmw_qos_profile_services_default, cb_group_);

        // --- Control loop timer ---
        state_ = State::IDLE;
        timer_ = this->create_wall_timer(300ms, std::bind(&SeatFinderNode::control_loop, this), cb_group_);
        RCLCPP_INFO(this->get_logger(), "Find seat action Ready.");
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
        state_ = State::MOVING_HEAD;
        RCLCPP_INFO(this->get_logger(), "Request received. Starting search...");

        // Bucle de espera (bloqueante para el servicio)
        rclcpp::Rate loop_rate(10); 
        end_of_task = false;
        while (!end_of_task && rclcpp::ok()) {
            loop_rate.sleep();
        }

        response->success = (state_ == State::SUCCESS);
        response->message = response->success ? "Free seat found" : "No free seat found";
        
        state_ = State::IDLE;
        RCLCPP_INFO(this->get_logger(), "Task completed, response sent");
    }

    void control_loop() {
        // Importante: No bloqueamos NADA aquí, solo transiciones de estado rápidas
        switch (state_) {
            case State::IDLE:
                failure_count_ = 0;
                break;

            case State::SUCCESS:
            case State::FAILURE:
                end_of_task = true;
                failure_count_ = 0;
                break;
                
            case State::MOVING_HEAD:
                publish_head_goal(target_pan_);
                state_ = State::WAITING_HEAD;
                failure_count_ = 0;
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
                failure_count_ = 0;
                break;

            case State::LOOK_AT_TARGET:
                final_goal.header.stamp.sec = 0;
                final_goal.header.stamp.nanosec = 0;
                head_look_at_pub_->publish(final_goal);
                failure_count_ = 0;
                state_ = State::SUCCESS;
                break;

            default: 
                RCLCPP_DEBUG(this->get_logger(), "Waiting...");
                if (++failure_count_ > 10 ) {
                    failure_count_ = 0;
                    state_ = State::FAILURE;
                }
                break;
        }
    }

    void call_vision_service() {
        if (!vision_client_->wait_for_service(2s)) {
            state_ = State::FAILURE;
            return;
        }
        auto req = std::make_shared<vision_interfaces::srv::FindSeat::Request>();

        // Usamos el callback asíncrono para liberar el hilo del timer de inmediato
        vision_client_->async_send_request(req, [this](rclcpp::Client<vision_interfaces::srv::FindSeat>::SharedFuture future) {
            auto res = future.get();
            if (res->success) {
                RCLCPP_INFO(this->get_logger(), "Seat found!");
                final_goal = res->point;
                state_ = State::LOOK_AT_TARGET;
            } else {
                this->next_step();
            }
        });
    }

    void next_step() {
        target_pan_ += scan_step_;

        bool finished = (target_pan_ > scan_end_);
        if (finished) {
            state_ = State::FAILURE;
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

    // Miembros
    State state_;
    float current_pan_ = 0.0;
    float target_pan_;
    const float tilt_val = -0.5;
    uint8_t failure_count_ = 0;
    uint8_t stabilize_count_ = 0;
    geometry_msgs::msg::PointStamped final_goal;
    bool end_of_task = false;

    float scan_start_ = -1.57;
    float scan_end_ = 1.57;
    float scan_step_ = 0.52;

    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr head_gp_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr head_gr_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr head_look_at_pub_;
    rclcpp::Client<vision_interfaces::srv::FindSeat>::SharedPtr vision_client_;
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