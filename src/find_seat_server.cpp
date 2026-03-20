#include <chrono>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

enum class State {
    SEARCHING,
    PANNING,
    SUCCESS, 
    FAILURE
};

class SeatFinderNode : public rclcpp::Node {
public:
    SeatFinderNode() : Node("seat_finder_fsm"), state_(State::SEARCHING) {
        // Inicializamos variables de paneo
        target_pan_ = -1.57; // Empezamos en el extremo izquierdo
        pan_increment_ = 0.52; // Incrementos de ~30 grados
        
        head_gp_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/hardware/head/goal_pose", 1);
        
        head_cp_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/hardware/head/current_pose", 10, 
            std::bind(&SeatFinderNode::current_pose_cb, this, std::placeholders::_1));
        
        client_ = this->create_client<std_srvs::srv::Trigger>("/check_seats");

        timer_ = this->create_wall_timer(500ms, std::bind(&SeatFinderNode::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "FSM iniciada. Primer objetivo: %.2f rad", target_pan_);
    }

private:
    void control_loop() {
        switch (state_) {
            case State::SEARCHING:
                // Solo llamamos al servicio si la cabeza ya está cerca del objetivo actual
                if (std::abs(current_pan_ - target_pan_) < 0.1) {
                    RCLCPP_INFO(this->get_logger(), "Posición alcanzada. Buscando asiento...");
                    call_seat_service();
                } else {
                    // Si aún no llega, seguimos publicando la meta
                    publish_head_goal(target_pan_);
                }
                break;

            case State::PANNING:
                move_to_next_position();
                break;

            case State::SUCCESS:
                target_pan_ = 0.0;
                publish_head_goal(target_pan_);
                break;
            
            case State::FAILURE:
                break;
                
                
        }
    }

    void move_to_next_position() {
        target_pan_ += pan_increment_;

        // Si excedemos el límite, reiniciamos o nos detenemos
        if (target_pan_ > 1.57) {
            RCLCPP_WARN(this->get_logger(), "Se recorrió todo el rango y no se encontró nada.");
            state_ = State::FAILURE;
        }

        RCLCPP_INFO(this->get_logger(), "Cambiando objetivo de paneo a: %.2f", target_pan_);
        publish_head_goal(target_pan_);
        
        // Volvemos a SEARCHING para verificar esta nueva posición
        state_ = State::SEARCHING;
    }

    void publish_head_goal(double pan_val) {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = {pan_val, 0.0}; // Pan y Tilt (tilt en 0)
        head_gp_publisher_->publish(msg);
    }

    void call_seat_service() {
        if (!client_->wait_for_service(100ms)) return;

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        // Usamos un sistema simple para no saturar de llamadas mientras esperamos respuesta
        if (!waiting_for_service_) {
            waiting_for_service_ = true;
            client_->async_send_request(request, 
                std::bind(&SeatFinderNode::service_callback, this, std::placeholders::_1));
        }
    }

    void service_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        waiting_for_service_ = false;
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "¡ASIENTO ENCONTRADO!");
            state_ = State::SUCCESS;
        } else {
            state_ = State::PANNING;
        }
    }

    void current_pose_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 2) {
            current_pan_ = static_cast<float>(msg->data[0]);
            current_tilt_ = static_cast<float>(msg->data[1]);
        }
    }

    // Variables de control
    State state_;
    float current_pan_ = 0.0;
    float current_tilt_ = 0.0;
    float target_pan_;
    float pan_increment_;
    bool waiting_for_service_ = false;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr head_gp_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr head_cp_subscriber_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SeatFinderNode>());
    rclcpp::shutdown();
    return 0;
}