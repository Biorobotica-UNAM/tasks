#include <chrono>
#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "vision_interfaces/srv/find_object.hpp"
#include "vision_interfaces/srv/get_detections.hpp"

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

class ObjectFinderNode : public rclcpp::Node {
public:
    ObjectFinderNode() : Node("object_finder_fsm") {
        // --- Callback group ---
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
       
        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = cb_group_;

        // --- Head control topics ---
        head_gp_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/hardware/head/goal_pose", 1);
        head_gr_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/hardware/head/goal_reached", 1,
            std::bind(&ObjectFinderNode::head_gr_cb, this, std::placeholders::_1), sub_options);
        head_look_at_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/hardware/head/look_at", 1);
       
        // --- Vision service client ---
        vision_client_ = this->create_client<vision_interfaces::srv::GetDetections>(
            "/vision/owl/owlv2_service", rclcpp::ServicesQoS().get_rmw_qos_profile(), cb_group_);

        // --- Task service server ---
        task_service_ = this->create_service<vision_interfaces::srv::FindObject>(
            "/tasks/find_object",
            std::bind(&ObjectFinderNode::handle_task_request, this, std::placeholders::_1, std::placeholders::_2),
            rclcpp::ServicesQoS().get_rmw_qos_profile(), cb_group_);

        // --- Control loop timer ---
        state_ = State::IDLE;
        timer_ = this->create_wall_timer(300ms, std::bind(&ObjectFinderNode::control_loop, this), cb_group_);
       
        RCLCPP_INFO(this->get_logger(), "Find object FSM Node Ready (Jazzy Optimized).");
    }

private:
    void handle_task_request(const std::shared_ptr<vision_interfaces::srv::FindObject::Request> request,
                             std::shared_ptr<vision_interfaces::srv::FindObject::Response> response)
    {
        if (state_ != State::IDLE) {
            response->success = false;
            response->message = "Already on a search.";
            return;
        }

        if (request->class_name.empty()) {
            response->success = false;
            response->message = "No class name provided.";
            return;
        }

        target_class_ = request->class_name;
        scan_idx_ = 0;
        target_pan_ = scan_poses_[scan_idx_].first;
        target_tilt_ = scan_poses_[scan_idx_].second;
        end_of_task = false;
        state_ = State::MOVING_HEAD;

        RCLCPP_INFO(this->get_logger(), "Starting search for: %s", target_class_.c_str());

        // Bucle de espera bloqueante para el cliente del servicio
        rclcpp::Rate loop_rate(10);
        while (!end_of_task && rclcpp::ok()) {
            loop_rate.sleep();
        }

        response->success = (state_ == State::SUCCESS);
        response->detected_class = this->detected_object_name;
        response->message = response->success ? "Object found" : "No object found after scan";
       
        state_ = State::IDLE;
        RCLCPP_INFO(this->get_logger(), "Task finished. Response sent.");
    }

    void control_loop() {
        switch (state_) {
            case State::MOVING_HEAD:
                publish_head_goal(target_pan_, target_tilt_);
                state_ = State::WAITING_HEAD;
                failure_count_ = 0;
                break;

            case State::STABILIZING:
                if (++stabilize_count_ > 3) { // ~1 segundo de espera
                    stabilize_count_ = 0;
                    state_ = State::CALLING_DETECTION;
                }
                break;

            case State::CALLING_DETECTION:
                call_vision_service();
                state_ = State::WAITING_SERVICE;
                break;

            case State::LOOK_AT_TARGET:
                head_look_at_pub_->publish(final_goal);
                state_ = State::SUCCESS;
                break;

            case State::SUCCESS:
            case State::FAILURE:
                end_of_task = true;
                break;

            default:
                break;
        }
    }

    void call_vision_service() {
        if (!vision_client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Vision service not available");
            state_ = State::FAILURE;
            return;
        }

        auto req = std::make_shared<vision_interfaces::srv::GetDetections::Request>();
        req->queries = {target_class_};

        vision_client_->async_send_request(req, [this](rclcpp::Client<vision_interfaces::srv::GetDetections>::SharedFuture future) {
            auto res = future.get();
            if (res->success && !res->detections.empty()) {
                RCLCPP_INFO(this->get_logger(), "Object detected!");
               
                // Tomamos la primera detección
                this->detected_object_name = res->detections[0].class_name;
               
                // Configuramos para que el robot mire a la TF generada por visión
                final_goal.header.frame_id = "OWL_" + this->detected_object_name;
                final_goal.header.stamp = this->get_clock()->now();
                final_goal.point.x = 0.0;
                final_goal.point.y = 0.0;
                final_goal.point.z = 0.0;

                state_ = State::LOOK_AT_TARGET;
            } else {
                RCLCPP_INFO(this->get_logger(), "Not found in this view. Moving to next pose.");
                this->next_step();
            }
        });
    }

    void next_step() {
        scan_idx_++;
        if (scan_idx_ >= scan_poses_.size()) {
            state_ = State::FAILURE;
            return;
        }
        target_pan_ = scan_poses_[scan_idx_].first;
        target_tilt_ = scan_poses_[scan_idx_].second;
        state_ = State::MOVING_HEAD;
    }

    void publish_head_goal(double pan_val, double tilt_val) {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = {pan_val, tilt_val};
        head_gp_pub_->publish(msg);
    }

    void head_gr_cb(const std_msgs::msg::Bool::SharedPtr msg) {
        if (state_ == State::WAITING_HEAD && msg->data) {
            state_ = State::STABILIZING;
        }
    }

    // Variables de control
    State state_;
    std::string detected_object_name;
    std::string target_class_;
    float target_pan_ = 0.0;
    float target_tilt_ = 0.0;
    uint8_t failure_count_ = 0;
    uint8_t stabilize_count_ = 0;
    bool end_of_task = false;
    size_t scan_idx_ = 0;

    geometry_msgs::msg::PointStamped final_goal;

    // Poses de escaneo (Pan, Tilt)
    std::vector<std::pair<float, float>> scan_poses_ = {
        { 0.0f,  -0.35f},
        {-0.45f, -0.40f},
        { 0.45f, -0.40f},
        { 0.0f,  -0.65f},
        {-0.30f, -0.70f},
        { 0.30f, -0.70f}
    };
   
    // ROS 2 Interfaces
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr head_gp_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr head_gr_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr head_look_at_pub_;
    rclcpp::Client<vision_interfaces::srv::GetDetections>::SharedPtr vision_client_;
    rclcpp::Service<vision_interfaces::srv::FindObject>::SharedPtr task_service_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectFinderNode>();
   
    // Executor multihilo para permitir que el timer y los servicios corran en paralelo
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
   
    rclcpp::shutdown();
    return 0;
}