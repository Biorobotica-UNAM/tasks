#include <chrono>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "vision_interfaces/srv/find_seat.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

using namespace std::chrono_literals;

enum class State {
    IDLE,
    MOVING_HEAD,
    WAITING_HEAD,
    REQUESTING_SERVICE,
    WAITING_FOR_RESPONSE,
    LOOK_AT,
    SUCCESS, 
    FAILURE
};

class SeatFinderNode : public rclcpp::Node {
public:
    SeatFinderNode() : Node("seat_finder_fsm") {
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        auto options = rclcpp::SubscriptionOptions();
        options.callback_group = cb_group_;
        
        state_ = State::IDLE;
        
        head_gp_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/hardware/head/goal_pose", 1);
        head_look_at_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/hardware/head/look_at", 1);
        
        head_cp_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/hardware/head/current_pose", 10, 
            std::bind(&SeatFinderNode::current_pose_cb, this, std::placeholders::_1), options);
        
        client_ = this->create_client<vision_interfaces::srv::FindSeat>("/check_seats", 
            rmw_qos_profile_services_default, cb_group_);

        task_service_ = this->create_service<std_srvs::srv::Trigger>("/tasks/find_seat",
            std::bind(&SeatFinderNode::handle_task_request, this, std::placeholders::_1, std::placeholders::_2), 
            rmw_qos_profile_services_default, cb_group_);

        timer_ = this->create_wall_timer(500ms, std::bind(&SeatFinderNode::control_loop, this), cb_group_);
        
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

        target_pan_ = -1.57;
        state_ = State::MOVING_HEAD;
        RCLCPP_INFO(this->get_logger(), "Request received. Starting search...");

        // Este es el único bloqueo necesario: esperar a que la FSM termine
        rclcpp::Rate loop_rate(10); 
        while (state_ != State::SUCCESS && state_ != State::FAILURE && rclcpp::ok()) {
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
            case State::MOVING_HEAD:
                publish_head_goal(target_pan_);
                state_ = State::WAITING_HEAD;
                break;

            case State::WAITING_HEAD:
                if (std::abs(current_pan_ - target_pan_) < 0.05) {
                    state_ = State::REQUESTING_SERVICE;
                }
                break;

            case State::REQUESTING_SERVICE:
                call_seat_service_async(); // Llamada NO bloqueante
                state_ = State::WAITING_FOR_RESPONSE;
                break;

            case State::LOOK_AT:
                final_goal.header.stamp.sec = 0;
                final_goal.header.stamp.nanosec = 0;
                head_look_at_publisher_->publish(final_goal);
                state_ = State::SUCCESS;
                /*static int look_counter = 0;
                if (look_counter == 0){
                }
                if (++look_counter > 4) { // 500ms * 4 = 2 segundos
                    look_counter = 0;
                }*/
                break;

            default: break;
        }
    }

    void call_seat_service_async() {
        if (!client_->wait_for_service(1s)) {
            RCLCPP_ERROR(this->get_logger(), "Service /check_seats not available");
            state_ = State::FAILURE;
            return;
        }

        auto request = std::make_shared<vision_interfaces::srv::FindSeat::Request>();
        RCLCPP_INFO(this->get_logger(), "Calling /check_seats...");

        // Usamos el callback asíncrono para liberar el hilo del timer de inmediato
        client_->async_send_request(request, [this](rclcpp::Client<vision_interfaces::srv::FindSeat>::SharedFuture future) {
            auto res = future.get();
            if (res->success) {
                RCLCPP_INFO(this->get_logger(), "Seat found!");
                final_goal = res->point;
                state_ = State::LOOK_AT;
            } else {
                target_pan_ += 0.52;
                if (target_pan_ > 1.57) {
                    RCLCPP_WARN(this->get_logger(), "Task failed. No free seat found");
                    state_ = State::FAILURE;
                } else {
                    RCLCPP_INFO(this->get_logger(), "No seat, moving to %.2f", target_pan_);
                    state_ = State::MOVING_HEAD;
                }
            }
        });
    }

    void publish_head_goal(double pan_val) {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = {pan_val, tilt_val};
        head_gp_publisher_->publish(msg);
    }

    void current_pose_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 1) current_pan_ = static_cast<float>(msg->data[0]);
    }

    // Miembros
    State state_;
    float current_pan_ = 0.0;
    float target_pan_;
    const float tilt_val = -0.5;
    geometry_msgs::msg::PointStamped final_goal;

    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<vision_interfaces::srv::FindSeat>::SharedPtr client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr task_service_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr head_gp_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr head_look_at_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr head_cp_subscriber_;
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