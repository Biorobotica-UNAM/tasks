#include <chrono>
#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include <algorithm>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "vision_interfaces/srv/find_object.hpp"
#include "vision_interfaces/srv/get_detections.hpp"

#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" 

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
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = cb_group_;
  
        // Inicialización de TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Publishers y Subscribers
        head_gp_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/hardware/head/goal_pose", 1);
        head_gr_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/hardware/head/goal_reached", 1,
            std::bind(&ObjectFinderNode::head_gr_cb, this, std::placeholders::_1), sub_options);
        head_look_at_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/hardware/head/look_at", 1);
        
        vision_client_ = this->create_client<vision_interfaces::srv::GetDetections>("/vision/owl/owlv2_service", 
            rmw_qos_profile_services_default, cb_group_);

        task_service_ = this->create_service<vision_interfaces::srv::FindObject>("/tasks/find_object",
            std::bind(&ObjectFinderNode::handle_task_request, this, std::placeholders::_1, std::placeholders::_2), 
            rmw_qos_profile_services_default, cb_group_);

        state_ = State::IDLE;
        timer_ = this->create_wall_timer(300ms, std::bind(&ObjectFinderNode::control_loop, this), cb_group_);
        RCLCPP_INFO(this->get_logger(), "Find object action Ready with TF2 Support.");
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

        target_class_ = request->class_name;
        scan_idx_ = 0;
        target_pan_ = scan_poses_[scan_idx_].first;
        target_tilt_ = scan_poses_[scan_idx_].second;
        state_ = State::MOVING_HEAD;
        
        end_of_task = false;
        rclcpp::Rate loop_rate(10); 
        while (!end_of_task && rclcpp::ok()) {
            loop_rate.sleep();
        }

        response->success = (state_ == State::SUCCESS);
        response->detected_class = this->detected_object_name; 
        response->message = response->success ? "Object found and referenced to base_link" : "Object not found";
        
        state_ = State::IDLE;
    }

    void control_loop() {
        switch (state_) {
            case State::MOVING_HEAD:
                publish_head_goal(target_pan_, target_tilt_);
                state_ = State::WAITING_HEAD;
                break;

            case State::STABILIZING:
                if (++stabilize_count_ > 3) { // Un poco más de tiempo para que la imagen no salga movida
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

            default: break;
        }
    }

    void call_vision_service() {
        if (!vision_client_->wait_for_service(1s)) {
            RCLCPP_ERROR(this->get_logger(), "Vision service not available");
            next_step();
            return;
        }

        auto req = std::make_shared<vision_interfaces::srv::GetDetections::Request>();
        req->queries = {target_class_};

        vision_client_->async_send_request(req, [this](rclcpp::Client<vision_interfaces::srv::GetDetections>::SharedFuture future) {
            auto res = future.get();
            if (res->success && !res->detections.empty()) {
                
                std::string clean_name = target_class_;
                std::replace(clean_name.begin(), clean_name.end(), ' ', '_');
                std::string target_frame = "OWL_" + clean_name;
                try {

                    auto tf_stamped = tf_buffer_->lookupTransform(
                        "base_link", 
                        target_frame, 
                        tf2::TimePointZero, 
                        500ms);
                    double x = tf_stamped.transform.translation.x;
                    double y = tf_stamped.transform.translation.y;
                    double z = tf_stamped.transform.translation.z;

                    tf2::Quaternion q(
                        tf_stamped.transform.rotation.x,
                        tf_stamped.transform.rotation.y,
                        tf_stamped.transform.rotation.z,
                        tf_stamped.transform.rotation.w
                    );
                    double roll, pitch, yaw;
                    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

                    double r_deg = roll * 57.2958;
                    double p_deg = pitch * 57.2958;
                    double y_deg = yaw * 57.2958;

                    final_goal.header.frame_id = "base_link";
                    final_goal.header.stamp = this->now();
                    final_goal.point.x = x;
                    final_goal.point.y = y;
                    final_goal.point.z = z;
                    

                    // ajustado para que el look_at sea natural
                    if (final_goal.point.z < 0.1) final_goal.point.z = 0.5; 

                    this->detected_object_name = res->detections[0].class_name;
                    state_ = State::LOOK_AT_TARGET;
                    RCLCPP_INFO(this->get_logger(), "Success! Target in base_link: [%.2f, %.2f, %.2f]", 
                                final_goal.point.x, final_goal.point.y, final_goal.point.z);

                } catch (tf2::TransformException &ex) {
                    RCLCPP_WARN(this->get_logger(), "TF2 Error: %s. Retrying next pose...", ex.what());
                    this->next_step();
                }
            } else {
                this->next_step();
            }
        });
    }

    void next_step() {
        scan_idx_++;
        if (scan_idx_ >= scan_poses_.size()) {
            state_ = State::FAILURE;
        } else {
            target_pan_ = scan_poses_[scan_idx_].first;
            target_tilt_ = scan_poses_[scan_idx_].second;
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

    // Miembros de la clase
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    std::string detected_object_name;
    State state_;
    float target_pan_, target_tilt_;
    bool end_of_task = false;
    uint8_t stabilize_count_ = 0;
    geometry_msgs::msg::PointStamped final_goal;
    std::string target_class_;

    std::vector<std::pair<float, float>> scan_poses_ = {
        { 0.0f,  -0.35f}, {-0.35f, -0.45f}, { 0.35f, -0.45f},
        { 0.0f,  -0.60f}, {-0.25f, -0.65f}, { 0.25f, -0.65f}
    };
    size_t scan_idx_ = 0;
    
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
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}