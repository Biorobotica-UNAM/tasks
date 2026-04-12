#include <chrono>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
// 1. Cambiamos la interfaz al servicio de humanos
#include "human_interfaces/srv/detect_pointing.hpp" 

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
        
        // 2. Cliente para el detector de waving
        vision_client_ = this->create_client<human_interfaces::srv::DetectPointing>(
            "/human_detection/detect_wrist", rmw_qos_profile_services_default, cb_group_);

        // 3. El servicio que ofrece este nodo (ahora Trigger o personalizado)
        // Lo cambié a Trigger para que sea más fácil de llamar sin argumentos
        task_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/tasks/find_waving_human",
            std::bind(&WavingHumanFinderNode::handle_task_request, this, std::placeholders::_1, std::placeholders::_2), 
            rmw_qos_profile_services_default, cb_group_);

        state_ = State::IDLE;
        timer_ = this->create_wall_timer(300ms, std::bind(&WavingHumanFinderNode::control_loop, this), cb_group_);
        RCLCPP_INFO(this->get_logger(), "Waving Detector FSM Ready.");
    }

private:
    void handle_task_request(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response) 
    {
        if (state_ != State::IDLE) {
            response->success = false;
            response->message = "Already searching for a human.";
            return;
        }

        scan_idx_ = 0;
        target_pan_ = scan_poses_[scan_idx_].first;
        target_tilt_ = scan_poses_[scan_idx_].second;
        state_ = State::MOVING_HEAD;
        RCLCPP_INFO(this->get_logger(), "Starting scan for waving humans...");

        rclcpp::Rate loop_rate(10); 
        end_of_task = false;
        while (!end_of_task && rclcpp::ok()) {
            loop_rate.sleep();
        }

        response->success = (state_ == State::SUCCESS);
        response->message = response->success ? "Human found waving" : "Nobody waving found";
        
        state_ = State::IDLE;
    }

    void control_loop() {
        switch (state_) {
            case State::IDLE:
                failure_count_ = 0;
                break;

            case State::SUCCESS:
            case State::FAILURE:
                end_of_task = true;
                break;

            case State::MOVING_HEAD:
                publish_head_goal(target_pan_, target_tilt_);
                state_ = State::WAITING_HEAD;
                break;

            case State::STABILIZING:
                if (++stabilize_count_ > 3) { // Un poquito más de tiempo para que la cámara enfoque
                    stabilize_count_ = 0;
                    state_ = State::CALLING_DETECTION;
                }
                break;

            case State::CALLING_DETECTION:
                call_vision_service();
                state_ = State::WAITING_SERVICE;
                break;

            case State::LOOK_AT_TARGET:
                // Aquí podrías usar la TF que publica el nodo de Python
                // Por ahora enviamos un punto vacío o el último conocido para confirmar éxito
                head_look_at_pub_->publish(final_goal);
                state_ = State::SUCCESS;
                break;

            default: 
                break;
        }
    }

    void call_vision_service() {
        if (!vision_client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Vision service not available");
            this->next_step();
            return;
        }

        auto req = std::make_shared<human_interfaces::srv::DetectPointing::Request>();
        
        vision_client_->async_send_request(req, [this](rclcpp::Client<human_interfaces::srv::DetectPointing>::SharedFuture future) {
            auto res = future.get();
            if (res->human_detected) {
                RCLCPP_INFO(this->get_logger(), "Waving human detected!");
                // Nota: Tu nodo de Python no devuelve el punto 3D en el servicio, lo publica en TF.
                // Aquí podrías llenar final_goal con datos de TF si fuera necesario.
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

    State state_;
    float target_pan_, target_tilt_;
    uint8_t stabilize_count_ = 0;
    bool end_of_task = false;
    geometry_msgs::msg::PointStamped final_goal;

    // Ajusté un poco los ángulos para buscar personas (más horizontales)
    std::vector<std::pair<float, float>> scan_poses_ = {
        { 0.0f,  -0.20f}, { -0.4f, -0.20f}, { 0.4f, -0.20f},
        { 0.0f,  -0.40f}, { -0.4f, -0.40f}, { 0.4f, -0.40f}
    };

    size_t scan_idx_ = 0;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr head_gp_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr head_gr_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr head_look_at_pub_;
    rclcpp::Client<human_interfaces::srv::DetectPointing>::SharedPtr vision_client_;
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