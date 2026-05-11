#include <chrono>
#include <memory>
#include <vector>
#include <atomic>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "human_interfaces/srv/detect_pointing.hpp" 
#include "torso_interfaces/srv/move_torso.hpp"

using namespace std::chrono_literals;

enum class State {
    IDLE,
    MOVING_HEAD,
    WAITING_HEAD,
    STABILIZING,
    CALLING_DETECTION,
    WAITING_SERVICE,
    MOVING_TORSO,
    WAITING_TORSO,
    LOOK_AT_TARGET,
    SUCCESS, 
    FAILURE
};

class WavingHumanFinderNode : public rclcpp::Node {
public:
    WavingHumanFinderNode() : Node("waving_finder_fsm") {
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        
        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = cb_group_;
  
        head_gp_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/hardware/head/goal_pose", 1);
        head_gr_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/hardware/head/goal_reached", 1,
            std::bind(&WavingHumanFinderNode::head_gr_cb, this, std::placeholders::_1), sub_options);
        
        head_look_at_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/hardware/head/look_at", 1);
        
        vision_client_ = this->create_client<human_interfaces::srv::DetectPointing>("/human_detection/detect_wrist", rmw_qos_profile_services_default, cb_group_);
        torso_client_ = this->create_client<torso_interfaces::srv::MoveTorso>("/hardware/torso/move_torso", rmw_qos_profile_services_default, cb_group_);

        task_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/tasks/find_waving_human",
            std::bind(&WavingHumanFinderNode::handle_task_request, this, std::placeholders::_1, std::placeholders::_2), 
            rmw_qos_profile_services_default, cb_group_);

        state_ = State::IDLE;
        timer_ = this->create_wall_timer(100ms, std::bind(&WavingHumanFinderNode::control_loop, this), cb_group_);
        
        RCLCPP_INFO(this->get_logger(), "Waving Detector ready.");
    }

    void handle_task_request(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response) 
    {
        if (state_ != State::IDLE) {
            response->success = false;
            response->message = "FSM is busy.";
            return;
        }

        scan_idx_ = 0;
        end_of_task_ = false;
        target_pan_ = scan_poses_[scan_idx_].first;
        target_tilt_ = scan_poses_[scan_idx_].second;
        
        state_ = State::MOVING_HEAD;

        int timeout_count = 0;
        while (!end_of_task_ && rclcpp::ok() && timeout_count < 300) { // 30 segundos de timeout total
            std::this_thread::sleep_for(100ms);
            timeout_count++;
        }

        if (timeout_count >= 300) {
            response->success = false;
            response->message = "Timeout: Search took too long";
        } else if (state_ == State::SUCCESS) {
            response->success = true;
            response->message = "Human found waving!";
        } else {
            response->success = false;
            response->message = "Human not found.";
        }

        state_ = State::IDLE;
        end_of_task_ = true; 
    }

    void control_loop() {
        if (state_ == State::IDLE) return;

        switch (state_) {
            case State::MOVING_HEAD:
                publish_head_goal(target_pan_, target_tilt_);
                state_ = State::WAITING_HEAD;
                break;

            case State::WAITING_HEAD:
                break;

            case State::STABILIZING:
                if (++stabilize_count_ > 5) { 
                    stabilize_count_ = 0;
                    state_ = State::CALLING_DETECTION;
                }
                break;

            case State::CALLING_DETECTION:
                call_vision_service();
                state_ = State::WAITING_SERVICE;
                break;

            case State::MOVING_TORSO:
                call_torso_service(0.40);
                state_ = State::WAITING_TORSO;
                break;

            case State::WAITING_TORSO:
            case State::WAITING_SERVICE:
                break;

            case State::LOOK_AT_TARGET:
                head_look_at_pub_->publish(final_goal);
                state_ = State::SUCCESS;
                break;

            case State::SUCCESS:
            case State::FAILURE:
                end_of_task_ = true;
                break;

            default:
                break;
        }
    }

    void call_torso_service(float height) {
        if (!torso_client_->wait_for_service(500ms)) {
            state_ = State::MOVING_HEAD;
            return;
        }
        auto req = std::make_shared<torso_interfaces::srv::MoveTorso::Request>();
        req->height = height; 
        torso_client_->async_send_request(req, [this](rclcpp::Client<torso_interfaces::srv::MoveTorso>::SharedFuture future) {
            state_ = State::MOVING_HEAD;
        });
    }

    void call_vision_service() {
        if (!vision_client_->wait_for_service(500ms)) {
            this->next_step();
            return;
        }
        auto req = std::make_shared<human_interfaces::srv::DetectPointing::Request>();
        vision_client_->async_send_request(req, [this](rclcpp::Client<human_interfaces::srv::DetectPointing>::SharedFuture future) {
            auto res = future.get();
            if (res->human_detected) {
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

        if (scan_idx_ == 3) {
            state_ = State::MOVING_TORSO;
        } else {
            state_ = State::MOVING_HEAD;
        }
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

    State state_;
    float target_pan_ = 0.0;
    float target_tilt_ = 0.0;
    uint8_t stabilize_count_ = 0;
    std::atomic<bool> end_of_task_{false};
    geometry_msgs::msg::PointStamped final_goal;

    std::vector<std::pair<float, float>> scan_poses_ = {
        { 0.0f, -0.20f}, {-0.4f, -0.20f}, { 0.4f, -0.20f},
        { 0.0f, -0.05f}, {-0.4f, -0.05f}, { 0.4f, -0.05f}
    };

    size_t scan_idx_ = 0;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr head_gp_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr head_gr_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr head_look_at_pub_;
    rclcpp::Client<human_interfaces::srv::DetectPointing>::SharedPtr vision_client_;
    rclcpp::Client<torso_interfaces::srv::MoveTorso>::SharedPtr torso_client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr task_service_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WavingHumanFinderNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}