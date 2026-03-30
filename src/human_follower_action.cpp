#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp" // Supongamos que esto da la distancia o confianza
#include "std_srvs/srv/set_bool.hpp"
#include <chrono>

using namespace std::chrono_literals;

enum class State { 
    IDLE, 
    LOCKING, 
    FOLLOWING, 
    LOST };

class HumanTrackerFSM : public rclcpp::Node {
public:
    HumanTrackerFSM() : Node("human_tracker_fsm"), current_state_(State::IDLE) {
        // --- Publishers ---
        pub_hf_enable_ = this->create_publisher<std_msgs::msg::Bool>("/hri/human_following/enable", 1);
        pub_lf_enable_ = this->create_publisher<std_msgs::msg::Bool>("/hri/leg_finder/enable", 1);
        pub_pt_enable_ = this->create_publisher<std_msgs::msg::Bool>("/person_tracker/enable", 1);

        // --- Subscribers (Para retroalimentación de los otros nodos) ---
        // Aquí deberías suscribirte a los tópicos que te dicen si ven a alguien
        sub_leg_found_ = this->create_subscription<std_msgs::msg::Bool>(
            "/hri/leg_finder/leg_detected", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) { leg_visible_ = msg->data; });
        
        // --- Service ---
        srv_enable_ = this->create_service<std_srvs::srv::SetBool>(
            "/hri/human_tracking/enable",
            std::bind(&HumanTrackerFSM::callback_enable, this, std::placeholders::_1, std::placeholders::_2));

        // --- Main Loop (FSM) ---
        timer_ = this->create_wall_timer(100ms, std::bind(&HumanTrackerFSM::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "FSM Human Tracker Initialized in IDLE state.");
    }

private:
    void control_loop() {
        if (!master_enable_) {
            change_state(State::IDLE);
            return;
        }

        switch (current_state_) {
            case State::IDLE:
                set_modules(false, true, true); // Motores OFF, Sensores ON
                if (leg_visible_) change_state(State::LOCKING);
                break;

            case State::LOCKING:
                // Tiempo de confirmación para evitar falsos positivos
                if (leg_visible_) {
                    lock_counter_++;
                    if (lock_counter_ > 10) change_state(State::FOLLOWING);
                } else {
                    change_state(State::IDLE);
                }
                break;

            case State::FOLLOWING:
                set_modules(true, true, true); // Todo prendido
                if (!leg_visible_) {
                    lost_time_ = this->now();
                    change_state(State::LOST);
                }
                break;

            case State::LOST:
                set_modules(false, true, true); // Detener motores pero seguir buscando
                if (leg_visible_) {
                    change_state(State::FOLLOWING);
                } else if ((this->now() - lost_time_).seconds() > 5.0) {
                    RCLCPP_WARN(this->get_logger(), "Target lost for too long. Resetting...");
                    change_state(State::IDLE);
                }
                break;
        }
    }

    void set_modules(bool hf, bool lf, bool pt) {
        auto msg = std_msgs::msg::Bool();
        msg.data = hf; pub_hf_enable_->publish(msg);
        msg.data = lf; pub_lf_enable_->publish(msg);
        msg.data = pt; pub_pt_enable_->publish(msg);
    }

    void change_state(State new_state) {
        if (current_state_ == new_state) return;
        current_state_ = new_state;
        lock_counter_ = 0;
        RCLCPP_INFO(this->get_logger(), "State changed to: %d", static_cast<int>(new_state));
    }

    // Callbacks y Variables
    void callback_enable(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                         std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        master_enable_ = req->data;
        res->success = true;
        res->message = master_enable_ ? "FSM Enabled" : "FSM Disabled";
    }

    State current_state_;
    bool master_enable_ = false;
    bool leg_visible_ = false;
    int lock_counter_ = 0;
    rclcpp::Time lost_time_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_hf_enable_, pub_lf_enable_, pub_pt_enable_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_leg_found_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_enable_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HumanTrackerFSM>());
    rclcpp::shutdown();
    return 0;
}