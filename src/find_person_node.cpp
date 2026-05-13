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
#include "vision_interfaces/srv/bbox_to_pose.hpp"

using namespace std::chrono_literals;

enum class State {
    IDLE,
    MOVING_HEAD,
    WAITING_HEAD,
    STABILIZING,
    CALLING_DETECTION, // busca si hay alguien
    CALLING_IDENTIFY,  // intenta buscar si esta en su base de datos 
    CALLING_REGISTER, // si no esta en la base, guarda el rostro
    CALLING_DESCRIBE, // genera una descripcion de la pesona 
    WAITING_SERVICE,
    CALLING_BBOX_TO_POSE_SERVICE, 
    LOOK_AT_TARGET,
    SUCCESS,
    FAILURE
};

class PersonFinderNode : public rclcpp::Node {
public:
    PersonFinderNode() : Node("person_finder_fsm") {
        // --- Callback group ---
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = cb_group_;
        
        // --- Head control topics ---
        head_gp_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/hardware/head/goal_pose", 1); 
        head_gr_sub = this->create_subscription<std_msgs::msg::Bool>(
            "/hardware/head/goal_reached", 1,
            std::bind(&PersonFinderNode::head_gr_cb, this, std::placeholders::_1), sub_options);
        head_look_at_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/hardware/head/look_at", 1);

        // --- Vision services ---
        detect_client_ = this->create_client<faces_interfaces::srv::FaceRecog>("/vision/detect_person", rmw_qos_profile_services_default, cb_group_);
        identify_client_ = this->create_client<faces_interfaces::srv::FaceRecog>("/vision/identify_person", rmw_qos_profile_services_default, cb_group_);
        register_client_ = this->create_client<faces_interfaces::srv::FaceRecog>("/vision/register_person", rmw_qos_profile_services_default, cb_group_);
        describe_client_ = this->create_client<faces_interfaces::srv::FaceRecog>("/vision/describe_person", rmw_qos_profile_services_default, cb_group_);
        bbox2pose_client_ = this->create_client<vision_interfaces::srv::BboxToPose>("/vision/bbox_to_pose", rmw_qos_profile_services_default, cb_group_);

        // --- Task service ---
        task_service_ = this->create_service<faces_interfaces::srv::FaceRecog>(
            "/tasks/find_person",
            std::bind(&PersonFinderNode::handle_task_request, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, cb_group_);

        // --- Control loop timer ---
        state_ = State::IDLE;
        timer_ = this->create_wall_timer(300ms, std::bind(&PersonFinderNode::control_loop, this), cb_group_);
        
        RCLCPP_INFO(this->get_logger(), "Find person action Ready.");
    }

private:
    void handle_task_request(const faces_interfaces::srv::FaceRecog::Request::SharedPtr req,
                      faces_interfaces::srv::FaceRecog::Response::SharedPtr res) 
    {
        if (state_ != State::IDLE) return;

        target_name_ = req->name;
        target_is_specified_ = !target_name_.empty();

        if (target_is_specified_) {
            // MODO PANEO: De izquierda (-1.57) a derecha (1.57) -> Incrementamos
            scan_start_ = -1.2;
            scan_end_ = 1.2;
            scan_step_ = 0.4; 
            RCLCPP_INFO(this->get_logger(), "Modo PANEO (Horizonte)");
        } else {
            // MODO TILTEO: De arriba (1.0) a abajo (-0.5) -> Decrementamos
            scan_start_ = 0.45;
            scan_end_ =  -0.2;
            scan_step_ = -0.15;
            RCLCPP_INFO(this->get_logger(), "Modo TILTEO (Vertical)");
        }

        current_target_val_ = scan_start_;
        state_ = State::MOVING_HEAD;

        // Bucle de espera (bloqueante para el servicio)
        rclcpp::Rate loop_rate(10);
        end_of_task = false;
        while (!end_of_task && rclcpp::ok()) {
            loop_rate.sleep();
        }

        res->success = (state_ == State::SUCCESS);
        state_ = State::IDLE;
        RCLCPP_INFO(this->get_logger(), "Task completed, response sent");
    }

    void control_loop() {
        if (state_ == State::IDLE || state_ == State::WAITING_SERVICE) return;

        switch (state_) {
            case State::SUCCESS:
            case State::FAILURE:
                end_of_task = true;
                failure_count_ = 0;
                break;

            case State::MOVING_HEAD:
                publish_head_goal();
                state_ = State::WAITING_HEAD;
                failure_count_ = 0;
                break;

            case State::STABILIZING:
                if (++stabilize_count_ > 3) {
                    stabilize_count_ = 0;
                    state_ = State::CALLING_DETECTION;
                }
                break;

            case State::CALLING_DETECTION:
                RCLCPP_INFO(this->get_logger(), "Detection/Identify service called");
                call_vision_service();
                state_ = State::WAITING_SERVICE;
                break;

            case State::CALLING_REGISTER:
                RCLCPP_INFO(this->get_logger(), "Registering new person...");
                call_register_service();
                state_ = State::WAITING_SERVICE;
                break;

            case State::CALLING_BBOX_TO_POSE_SERVICE:
                RCLCPP_INFO(this->get_logger(), "Calling BBox to Pose service...");
                call_bbox_to_pose_service();
                state_ = State::WAITING_SERVICE;
                break;

            case State::LOOK_AT_TARGET:
                final_goal.header.stamp = this->now();
                head_look_at_pub_->publish(final_goal);
                state_ = State::CALLING_DESCRIBE; // Después de mirar, describimos
                break;

            case State::CALLING_DESCRIBE:
                RCLCPP_INFO(this->get_logger(), "Generating Florence-2 report...");
                call_describe_service();
                state_ = State::WAITING_SERVICE;
                break;
            
            default: 
                break;
        }
    }

    void publish_head_goal() {
        auto msg = std_msgs::msg::Float64MultiArray();
        if (target_is_specified_) {
            msg.data = {current_target_val_, 0.0}; 
        } else {
            msg.data = {0.0, current_target_val_}; 
        }
        head_gp_pub_->publish(msg);
    }

    void head_gr_cb(const std_msgs::msg::Bool::SharedPtr msg) {
        if (state_ == State::WAITING_HEAD && msg->data) {
            state_ = State::STABILIZING;
        }
    }

    void call_vision_service() {
        auto client = target_is_specified_ ? identify_client_ : detect_client_;
        auto req = std::make_shared<faces_interfaces::srv::FaceRecog::Request>();
        if (target_is_specified_) req->name = target_name_;

        client->async_send_request(req, [this](rclcpp::Client<faces_interfaces::srv::FaceRecog>::SharedFuture future) {
            auto result = future.get();
            if (result->success) {
                this->last_bbox_ = result->bbox;
                // Si lo identificó y no es Unknown, vamos a pose. Si es Unknown o solo detectó, registramos.
                if (target_is_specified_ || (result->name != "Unknown" && !result->name.empty())) {
                    this->state_ = State::CALLING_BBOX_TO_POSE_SERVICE;
                } else {
                    this->state_ = State::CALLING_REGISTER;
                }
            } else {
                this->next_step();
            }
        });
    }

    void call_register_service() {
        auto req = std::make_shared<faces_interfaces::srv::FaceRecog::Request>();
        req->name = target_is_specified_ ? target_name_ : "Person_" + std::to_string(this->now().seconds());

        register_client_->async_send_request(req, [this](rclcpp::Client<faces_interfaces::srv::FaceRecog>::SharedFuture future) {
            if (future.get()->success) this->state_ = State::CALLING_BBOX_TO_POSE_SERVICE;
            else this->state_ = State::FAILURE;
        });
    }

    void call_bbox_to_pose_service() {
        auto req = std::make_shared<vision_interfaces::srv::BboxToPose::Request>();
        req->x1 = last_bbox_[0]; req->y1 = last_bbox_[1];
        req->x2 = last_bbox_[2]; req->y2 = last_bbox_[3];

        bbox2pose_client_->async_send_request(req, [this](rclcpp::Client<vision_interfaces::srv::BboxToPose>::SharedFuture future) {
            auto res = future.get();
            if (res->success) {
                this->final_goal.header.frame_id = res->pose.header.frame_id;
                this->final_goal.point = res->pose.pose.position; 
                this->state_ = State::LOOK_AT_TARGET;
            } else {
                this->state_ = State::FAILURE;
            }
        });
    }

    void call_describe_service() {
        auto req = std::make_shared<faces_interfaces::srv::FaceRecog::Request>();
        describe_client_->async_send_request(req, [this](rclcpp::Client<faces_interfaces::srv::FaceRecog>::SharedFuture future) {
            if (future.get()->success) this->state_ = State::SUCCESS;
            else this->state_ = State::FAILURE;
        });
    }

    void next_step() {
        current_target_val_ += scan_step_;
        bool finished = (scan_step_ > 0) ? (current_target_val_ > scan_end_) 
                                         : (current_target_val_ < scan_end_);

        if (finished) {
            RCLCPP_ERROR(this->get_logger(), "Escaneo completado sin éxito.");
            state_ = State::FAILURE;
        } else {
            state_ = State::MOVING_HEAD;
        }
    }

    // Variables de control
    State state_;
    std::string target_name_;
    bool target_is_specified_;
    float current_target_val_ = 0.0;
    uint8_t failure_count_ = 0;
    uint8_t stabilize_count_ = 0;
    geometry_msgs::msg::PointStamped final_goal;
    bool end_of_task = false;

    // Scan config
    float scan_start_;
    float scan_end_;
    float scan_step_;
    std::vector<float> last_bbox_;

    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr head_gp_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr head_gr_sub;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr head_look_at_pub_;
    rclcpp::Client<faces_interfaces::srv::FaceRecog>::SharedPtr detect_client_, identify_client_, register_client_, describe_client_;
    rclcpp::Client<vision_interfaces::srv::BboxToPose>::SharedPtr bbox2pose_client_;
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