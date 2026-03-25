#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
// Asumo que estas son tus interfaces personalizadas
#include "faces_interfaces/srv/face_recog.hpp" 

using namespace std::chrono_literals;

enum class State {
    IDLE,
    MOVING_HEAD,
    WAITING_HEAD,
    CALLING_DETECTION,
    CALLING_IDENTIFY,
    WAITING_SERVICE, 
    LOOK_AT_TARGET,
    IDENTIFYING_TARGET,
    SUCCESS,
    FAILURE
};

class PersonFinderNode : public rclcpp::Node {
public:
    PersonFinderNode() : Node("person_finder_fsm") {
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        auto options = rclcpp::SubscriptionOptions();
        options.callback_group = cb_group_;

        // --- Publishers y Subscribers ---
        head_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/hardware/head/goal_pose", 1);
        look_at_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/hardware/head/look_at", 1);
        
        head_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/hardware/head/current_pose", 10,
            std::bind(&PersonFinderNode::head_pose_cb, this, std::placeholders::_1), options);

        // --- Clientes de Visión ---
        // Servicio 1: Solo detecta si hay humanos
        detect_client_ = this->create_client<faces_interfaces::srv::FaceRecog>("/vision/detect_person", 
            rmw_qos_profile_services_default, cb_group_);
        
        // Servicio 2: Identifica por nombre 
        identify_client_ = this->create_client<faces_interfaces::srv::FaceRecog>("/vision/identify_person", 
            rmw_qos_profile_services_default, cb_group_);

        // --- Servicio de Tarea (El que tú llamas) ---
        task_service_ = this->create_service<faces_interfaces::srv::FaceRecog>(
            "/tasks/find_person",
            std::bind(&PersonFinderNode::handle_task, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, cb_group_);

        timer_ = this->create_wall_timer(500ms, std::bind(&PersonFinderNode::control_loop, this), cb_group_);
        
        state_ = State::IDLE;
        RCLCPP_INFO(this->get_logger(), "Find person action ready.");
    }

private:
    // Callback principal que bloquea al cliente hasta terminar la FSM
    void handle_task(const faces_interfaces::srv::FaceRecog::Request::SharedPtr req,
                      faces_interfaces::srv::FaceRecog::Response::SharedPtr res) 
    {
        if (state_ != State::IDLE) return;

        target_name_ = req->name;
        target_is_specified_ = !target_name_.empty();

        target_pan_ = -1.57;
        state_ = State::MOVING_HEAD;

        RCLCPP_INFO(this->get_logger(), target_is_specified_ ? 
                "Buscando a: %s" : "Buscando a cualquier persona", target_name_.c_str());

        rclcpp::Rate loop_rate(10);
        while (state_ != State::SUCCESS && state_ != State::FAILURE && rclcpp::ok()) {
            loop_rate.sleep();
        }

        res->success = (state_ == State::SUCCESS);
        //res->point = last_person_pose_;

        state_ = State::IDLE;
    }

    void control_loop() {
        if (state_ == State::IDLE || state_ == State::WAITING_SERVICE) return;

        switch (state_) {
            case State::MOVING_HEAD:
                publish_head(target_pan_);
                state_ = State::WAITING_HEAD;
                break;

            case State::WAITING_HEAD:
                if (std::abs(current_pan_ - target_pan_) < 0.05) {
                    state_ = State::CALLING_DETECTION; 
                }
                break;

            case State::CALLING_DETECTION:
                call_detect_anyone(); 
                break;

            case State::IDENTIFYING_TARGET: // Este estado ahora se activa desde la lambda de detección
                call_identify_target(); 
                break;

            case State::LOOK_AT_TARGET:
                execute_look_at();
                break;
            
            default: break;
        }
    }
    // Servicio 1: ¿Hay alguien? (Detección rápida)
    void call_detect_anyone() {
        state_ = State::WAITING_SERVICE;
        
        auto req = std::make_shared<faces_interfaces::srv::FaceRecog::Request>();
        detect_client_->async_send_request(req, [this](rclcpp::Client<faces_interfaces::srv::FaceRecog>::SharedFuture future) {
            auto res = future.get();
            if (res->success) {
                if (this->target_is_specified_) {
                    // Si buscamos a alguien concreto, vamos al paso de identidad
                    RCLCPP_INFO(this->get_logger(), "Persona detectada, verificando identidad...");
                    this->state_ = State::IDENTIFYING_TARGET;
                } else {
                    // Si solo buscamos "un humano", ya terminamos con éxito
                    RCLCPP_INFO(this->get_logger(), "¡Encontré a una persona!");
                    this->state_ = State::LOOK_AT_TARGET;
                }
            } else {
                this->next_pan_step();
            }
        });
    }

    // Servicio 2: ¿Es la persona que busco? (Identificación pesada)
    void call_identify_target() {
        // 1. BLOQUEO INMEDIATO
        state_ = State::WAITING_SERVICE; 

        if (!identify_client_->wait_for_service(500ms)) {
            RCLCPP_WARN(this->get_logger(), "Servicio de identificación no disponible");
            next_pan_step();
            return;
        }

        auto req = std::make_shared<faces_interfaces::srv::FaceRecog::Request>();
        req->name = target_name_;

        identify_client_->async_send_request(req, [this](rclcpp::Client<faces_interfaces::srv::FaceRecog>::SharedFuture future) {
            auto res = future.get();
            if (res->success) {
                RCLCPP_INFO(this->get_logger(), "¡%s identificado!", target_name_.c_str());
                this->state_ = State::LOOK_AT_TARGET;
            } else {
                RCLCPP_WARN(this->get_logger(), "No es %s, sigo buscando...", target_name_.c_str());
                this->next_pan_step(); 
            }
        });
    }

    void execute_look_at() {
        //static int count = 0;
        //last_person_pose_.header.stamp.sec = 0; // El truco del tiempo 0 que vimos
        //last_person_pose_.header.stamp.nanosec = 0;
        //look_at_pub_->publish(last_person_pose_);
        state_ = State::SUCCESS;
    }

    void next_pan_step() {
        target_pan_ += 0.6; // Pasos más largos para personas
        if (target_pan_ > 1.57) {
            state_ = State::FAILURE;
        } else {
            state_ = State::MOVING_HEAD;
        }
    }

    void publish_head(double pan) {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = {pan, -0.2}; // Mirando un poco hacia abajo
        head_pub_->publish(msg);
    }

    void head_pose_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() > 0) current_pan_ = msg->data[0];
    }

    // Variables
    State state_;
    std::string target_name_;
    float current_pan_ = 0.0;
    float target_pan_ = 0.0;
    bool target_is_specified_ = false;
    //geometry_msgs::msg::PointStamped last_person_pose_;

    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr head_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr look_at_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr head_sub_;
    rclcpp::Client<faces_interfaces::srv::FaceRecog>::SharedPtr detect_client_;
    rclcpp::Client<faces_interfaces::srv::FaceRecog>::SharedPtr identify_client_;
    rclcpp::Service<faces_interfaces::srv::FaceRecog>::SharedPtr task_service_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PersonFinderNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}