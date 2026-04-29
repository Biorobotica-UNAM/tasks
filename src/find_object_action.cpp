#include <chrono>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "vision_interfaces/srv/get_bbox.hpp"

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
        
        // --- Vision service ---
        vision_client_ = this->create_client<vision_interfaces::srv::GetBbox>("/vision/owl/Owlv2Service", 
            rmw_qos_profile_services_default, cb_group_);

        // --- Task service ---
        task_service_ = this->create_service<vision_interfaces::srv::GetBbox>("/tasks/find_object",
            std::bind(&ObjectFinderNode::handle_task_request, this, std::placeholders::_1, std::placeholders::_2), 
            rmw_qos_profile_services_default, cb_group_);

        // --- Control loop timer ---
        state_ = State::IDLE;
        timer_ = this->create_wall_timer(300ms, std::bind(&ObjectFinderNode::control_loop, this), cb_group_);
        RCLCPP_INFO(this->get_logger(), "Find object action Ready.");
    }

private:
    void handle_task_request(const std::shared_ptr<vision_interfaces::srv::GetBbox::Request> request,
                             std::shared_ptr<vision_interfaces::srv::GetBbox::Response> response) 
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
        state_ = State::MOVING_HEAD;
        RCLCPP_INFO(this->get_logger(), "Request received. Starting search...");

        // Bucle de espera (bloqueante para el servicio)
        rclcpp::Rate loop_rate(10); 
        end_of_task = false;
        while ( !end_of_task && rclcpp::ok()) {
            loop_rate.sleep();
        }

        response->success = (state_ == State::SUCCESS);
        if (response->success) {
            // Aquí es donde el Find.cpp recibirá el nombre real
            response->detected_class = this->detected_object_name; 
        }  
        response->message = response->success ? "Object found" : "No object found";
        
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
                publish_head_goal(target_pan_, target_tilt_);
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
                    end_of_task = true;
                }
                break;
        }
    }

    void call_vision_service() {
        if (!vision_client_->wait_for_service(2s)) {
            state_ = State::FAILURE;
            return;
        }
        auto req = std::make_shared<vision_interfaces::srv::GetBbox::Request>();
        req->class_name = target_class_;

        // Usamos el callback asíncrono para liberar el hilo del timer de inmediato
        vision_client_->async_send_request(req, [this](rclcpp::Client<vision_interfaces::srv::GetBbox>::SharedFuture future) {
            auto res = future.get();
            if (res->success) {
                RCLCPP_INFO(this->get_logger(), "Object found!");
                final_goal = res->point;
                this->detected_object_name = res->detected_class;
                state_ = State::LOOK_AT_TARGET;
            } else {
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

    std::string detected_object_name;
    State state_;
    float target_pan_ = 0.0;
    float target_tilt_ = -0.5;
    uint8_t failure_count_ = 0;
    uint8_t stabilize_count_ = 0;
    bool end_of_task = false;
    geometry_msgs::msg::PointStamped final_goal;

    std::vector<std::pair<float, float>> scan_poses_ = {
        { 0.0f,  -0.35f},   // centro, un poco arriba
        {-0.35f, -0.45f},   // izquierda media
        { 0.35f, -0.45f},   // derecha media
        { 0.0f,  -0.60f},   // centro más abajo
        {-0.25f, -0.65f},   // izquierda abajo
        { 0.25f, -0.65f}    // derecha abajo
    };

    size_t scan_idx_ = 0;
    std::string target_class_;
    
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr head_gp_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr head_gr_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr head_look_at_pub_;
    rclcpp::Client<vision_interfaces::srv::GetBbox>::SharedPtr vision_client_;
    rclcpp::Service<vision_interfaces::srv::GetBbox>::SharedPtr task_service_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectFinderNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}