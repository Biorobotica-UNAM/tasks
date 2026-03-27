#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
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
    SUCCESS,
    FAILURE
};

class PersonFinderNode : public rclcpp::Node {
public:
    PersonFinderNode() : Node("person_finder_fsm") {
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        
        head_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/hardware/head/goal_pose", 1);
        
        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = cb_group_;
        head_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/hardware/head/current_pose", 10,
            std::bind(&PersonFinderNode::head_pose_cb, this, std::placeholders::_1), sub_options);

        head_goal_reached_sub = this->create_subscription<std_msgs::msg::Bool>(
            "/hardware/head/goal_reached", 1,
            std::bind(&PersonFinderNode::head_goal_reached_cb, this, std::placeholders::_1), sub_options);

        detect_client_ = this->create_client<faces_interfaces::srv::FaceRecog>("/vision/detect_person", rmw_qos_profile_services_default, cb_group_);
        identify_client_ = this->create_client<faces_interfaces::srv::FaceRecog>("/vision/identify_person", rmw_qos_profile_services_default, cb_group_);

        task_service_ = this->create_service<faces_interfaces::srv::FaceRecog>(
            "/tasks/find_person",
            std::bind(&PersonFinderNode::handle_task, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, cb_group_);

        timer_ = this->create_wall_timer(200ms, std::bind(&PersonFinderNode::control_loop, this), cb_group_);
        
        state_ = State::IDLE;
    }

private:
    void handle_task(const faces_interfaces::srv::FaceRecog::Request::SharedPtr req,
                      faces_interfaces::srv::FaceRecog::Response::SharedPtr res) 
    {
        if (state_ != State::IDLE) return;

        target_name_ = req->name;
        target_is_specified_ = !target_name_.empty();

        if (target_is_specified_) {
            // MODO PANEO: De izquierda (-1.57) a derecha (1.57) -> Incrementamos
            scan_start_ = -1.57;
            scan_end_ = 1.57;
            scan_step_ = 0.4; 
            RCLCPP_INFO(this->get_logger(), "Modo PANEO (Horizonte)");
        } else {
            // MODO TILTEO: De arriba (1.0) a abajo (-0.5) -> Decrementamos
            scan_start_ = 0.47;
            scan_end_ =  0.0;
            scan_step_ = -0.1;
            RCLCPP_INFO(this->get_logger(), "Modo TILTEO (Vertical)");
        }

        current_target_val_ = scan_start_;
        state_ = State::MOVING_HEAD;

        // Bucle de espera (bloqueante para el servicio)
        rclcpp::Rate loop_rate(10);
        while (state_ != State::SUCCESS && state_ != State::FAILURE && rclcpp::ok()) {
            loop_rate.sleep();
        }

        res->success = (state_ == State::SUCCESS);
        state_ = State::IDLE;
    }

    void head_goal_reached_cb(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data){
            state_ = State::CALLING_DETECTION;
        }
    }

    void control_loop() {
        if (state_ == State::IDLE || state_ == State::WAITING_SERVICE) return;

        switch (state_) {
            case State::MOVING_HEAD:
                publish_head_goal();
                state_ = State::WAITING_HEAD;
                failure_count = 0;
                break;

            case State::CALLING_DETECTION:
                RCLCPP_INFO(this->get_logger(), "Detection service called");
                call_vision_service();
                failure_count = 0;
                break;

            case State::LOOK_AT_TARGET:
                state_ = State::SUCCESS;
                failure_count = 0;
                break;
            
            default: 
                RCLCPP_INFO(this->get_logger(), "Waiting for something");
                if (++failure_count > 8 ) {
                    failure_count = 0;
                    state_ = State::FAILURE;
                }
                break;
        }
    }

    void publish_head_goal() {
        auto msg = std_msgs::msg::Float64MultiArray();
        if (target_is_specified_) {
            msg.data = {current_target_val_, 0.0}; // Mover PAN, dejar tilt en 0
        } else {
            msg.data = {0.0, current_target_val_}; // Mover TILT, dejar pan en 0
        }
        head_pub_->publish(msg);
    }


    void call_vision_service() {
        state_ = State::WAITING_SERVICE;
        auto req = std::make_shared<faces_interfaces::srv::FaceRecog::Request>();
        
        // Si hay nombre, usamos identify, si no, detect
        auto client = target_is_specified_ ? identify_client_ : detect_client_;
        if (target_is_specified_) req->name = target_name_;

        client->async_send_request(req, [this](rclcpp::Client<faces_interfaces::srv::FaceRecog>::SharedFuture future) {
            if (future.get()->success) {
                this->state_ = State::LOOK_AT_TARGET;
            } else {
                this->next_step();
            }
        });
    }

    void next_step() {
        current_target_val_ += scan_step_;

        // Lógica flexible para saber si terminamos el recorrido:
        // Si el paso es positivo, terminamos al superar scan_end_
        // Si el paso es negativo, terminamos al ser menores que scan_end_
        bool finished = (scan_step_ > 0) ? (current_target_val_ > scan_end_) 
                                         : (current_target_val_ < scan_end_);

        if (finished) {
            RCLCPP_ERROR(this->get_logger(), "Escaneo completado sin éxito.");
            state_ = State::FAILURE;
        } else {
            state_ = State::MOVING_HEAD;
        }
    }

    void head_pose_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 2) {
            current_head_pos_[0] = msg->data[0]; // pan
            current_head_pos_[1] = msg->data[1]; // tilt
        }
    }

    // Variables de control
    State state_;
    std::string target_name_;
    bool target_is_specified_;
    float current_target_val_ = 0.0;
    std::vector<double> current_head_pos_ = {0.0, 0.0};
    uint failure_count;

    // Scan config
    float scan_start_;
    float scan_end_;
    float scan_step_;

    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr head_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr head_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr head_goal_reached_sub;
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