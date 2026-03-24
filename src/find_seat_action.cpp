#include <chrono>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

enum class State {
    MOVING_HEAD,
    WAITING_HEAD,
    REQUESTING_SERVICE,
    WAITING_FOR_RESPONSE,
    SUCCESS, 
    FAILURE
};

//TODO: hacer esta wea como action en vez de servicio

class SeatFinderNode : public rclcpp::Node {
public:
    SeatFinderNode() : Node("seat_finder_fsm"), seat_found_(false) {
        // Inicializamos variables de paneo
        target_pan_ = -1.57; // Empezamos en el extremo izquierdo
        pan_increment_ = 0.52; // Incrementos de ~30 grados

        state_ = State::MOVING_HEAD;
        waiting_for_service_ = false;
        
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
            case State::MOVING_HEAD:
                RCLCPP_INFO(this->get_logger(), "Moviendo cabeza a %.2f", target_pan_);
                publish_head_goal(target_pan_);
                state_ = State::WAITING_HEAD;
                break;

            case State::WAITING_HEAD:
                if (std::abs(current_pan_ - target_pan_) < 0.05) {
                    RCLCPP_INFO(this->get_logger(), "Cabeza en posición.");
                    state_ = State::REQUESTING_SERVICE;
                }
                break;

            case State::REQUESTING_SERVICE:
                RCLCPP_INFO(this->get_logger(), ">>> LLAMANDO AL SERVICIO...");
                call_seat_service();
                state_ = State::WAITING_FOR_RESPONSE;
                break;

            case State::WAITING_FOR_RESPONSE:
                // AQUÍ EL NODO NO HACE NADA. 
                // El timer entra y sale sin ejecutar lógica hasta que el callback cambie el estado.
                break;

            case State::SUCCESS:
                RCLCPP_INFO(this->get_logger(), "TERMINADO: Asiento libre encontrado.");
                timer_->cancel();
                break;

            case State::FAILURE:
                RCLCPP_ERROR(this->get_logger(), "TERMINADO: No se encontró nada.");
                timer_->cancel();
                break;
        }
    }


    void publish_head_goal(double pan_val) {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = {pan_val, tilt_val}; // Pan y Tilt (tilt en 0)
        head_gp_publisher_->publish(msg);
    }

    bool call_seat_service() {
        if (!client_->wait_for_service(100ms)) {
            RCLCPP_WARN(this->get_logger(), "Servicio /check_seats no disponible.");
            return false;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        
        // Solo enviamos si no hay una petición pendiente (doble seguro)
        if (!waiting_for_service_) {
            waiting_for_service_ = true;
            client_->async_send_request(request, 
                std::bind(&SeatFinderNode::service_callback, this, std::placeholders::_1));
            return true;
        }
        return false;
    }

    void service_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        auto response = future.get();
        waiting_for_service_ = false; // Liberamos la bandera

        if (response->success) {
            state_ = State::SUCCESS;
        } else {
            RCLCPP_INFO(this->get_logger(), "<<< Servicio respondió: No hay asiento.");
            
            target_pan_ += pan_increment_;
            if (target_pan_ > 1.57) {
                state_ = State::FAILURE;
            } else {
                RCLCPP_INFO(this->get_logger(), "Preparando siguiente movimiento a %.2f", target_pan_);
                state_ = State::MOVING_HEAD; // Volvemos a empezar el ciclo de movimiento
            }
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
    bool seat_found_;
    const float tilt_val = -0.5;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr head_gp_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr head_cp_subscriber_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SeatFinderNode>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}