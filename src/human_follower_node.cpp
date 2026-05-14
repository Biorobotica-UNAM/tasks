#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <chrono>

using namespace std::chrono_literals;

enum class State { IDLE, LOCKING, FOLLOWING, LOST };

class HumanTrackerAction : public rclcpp::Node {
public:
    HumanTrackerAction() : Node("human_tracker_action"), current_state_(State::IDLE) {
        // --- Publishers ---
        pub_target_filtered_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/hri/target_filtered", 10);
        pub_hf_enable_ = this->create_publisher<std_msgs::msg::Bool>("/hri/human_following/enable", 1);

        // --- Subscribers ---
        sub_legs_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/hri/leg_finder/leg_pose", 10, [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
                last_legs_ = *msg;
                have_legs_ = true;
            });
        
        sub_body_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/human_detection/target_point", 10, [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
                last_body_ = *msg;
                have_body_ = true;
            });

        // --- Service ---
        srv_enable_ = this->create_service<std_srvs::srv::SetBool>(
            "/hri/human_tracking/enable",
            std::bind(&HumanTrackerAction::callback_master_enable, this, std::placeholders::_1, std::placeholders::_2));

        timer_ = this->create_wall_timer(50ms, std::bind(&HumanTrackerAction::control_loop, this));
    }

private:
    // Estructura Kalman (La movemos aquí para que la fusión sea suave)
    struct KalmanFilter {
        float x = 0.0f, p = 1.0f, q = 0.05f, r = 0.15f;
        float filter(float m) {
            p += q;
            float k = p / (p + r);
            x += k * (m - x);
            p *= (1.0f - k);
            return x;
        }
    };

    KalmanFilter k_x, k_y;
    State current_state_;
    bool master_enable_ = false;
    bool have_legs_ = false, have_body_ = false;
    geometry_msgs::msg::PointStamped last_legs_, last_body_;
    int lock_counter_ = 0;
    rclcpp::Time last_seen_time_;

    // Constantes de Fusión
    const float LEG_WEIGHT = 0.7; // Confiamos más en el LiDAR para distancia
    const float BODY_WEIGHT = 0.3;
    const float MAX_FUSION_DIST = 0.6; // Distancia máxima entre cámara y láser para fusionar

    void control_loop() {
        if (!master_enable_) {
            update_state(State::IDLE);
            return;
        }

        bool any_sensor = check_sensors();
        
        switch (current_state_) {
            case State::IDLE:
                if (any_sensor) update_state(State::LOCKING);
                break;

            case State::LOCKING:
                if (any_sensor) {
                    if (++lock_counter_ > 10) update_state(State::FOLLOWING);
                } else {
                    update_state(State::IDLE);
                }
                break;

            case State::FOLLOWING:
                if (any_sensor) {
                    process_and_publish_fusion();
                    last_seen_time_ = this->now();
                } else {
                    update_state(State::LOST);
                }
                break;

            case State::LOST:
                if (any_sensor) {
                    update_state(State::FOLLOWING);
                } else if ((this->now() - last_seen_time_).seconds() > 3.0) {
                    update_state(State::IDLE);
                }
                break;
        }
    }

    bool check_sensors() {
        auto now = this->now();
        bool legs_fresh = have_legs_ && (now - last_legs_.header.stamp).seconds() < 0.5;
        bool body_fresh = have_body_ && (now - last_body_.header.stamp).seconds() < 0.5;
        return (legs_fresh || body_fresh);
    }

    void process_and_publish_fusion() {
        auto now = this->now();
        
        // Calculamos la edad de los datos en segundos
        double legs_age = (now - last_legs_.header.stamp).seconds();
        double body_age = (now - last_body_.header.stamp).seconds();

        // Umbrales de frescura (Láser es más rápido, Cámara más lenta)
        bool legs_fresh = (legs_age < 0.2); // 200ms
        bool body_fresh = (body_age < 0.5); // 500ms (YOLO es más lento)

        float raw_x, raw_y;

        if (legs_fresh && body_fresh) {
            // FUSIÓN DINÁMICA: Si ambos están, promediamos con pesos
            raw_x = (last_legs_.point.x * 0.7) + (last_body_.point.x * 0.3);
            raw_y = (last_legs_.point.y * 0.7) + (last_body_.point.y * 0.3);
        } 
        else if (legs_fresh) {
            // Solo tenemos piernas (LiDAR)
            raw_x = last_legs_.point.x;
            raw_y = last_legs_.point.y;
        } 
        else if (body_fresh) {
            // Solo tenemos cuerpo (Cámara)
            raw_x = last_body_.point.x;
            raw_y = last_body_.point.y;
        } 
        else {
            // Si nada es fresco, no publicamos (el Follower frenará por timeout)
            return; 
        }

        // El Kalman aquí suaviza la transición entre cuando hay piernas y cuando no
        geometry_msgs::msg::PointStamped fused_msg;
        fused_msg.header.stamp = now;
        fused_msg.header.frame_id = "base_link";
        fused_msg.point.x = k_x.filter(raw_x);
        fused_msg.point.y = k_y.filter(raw_y);
        
        pub_target_filtered_->publish(fused_msg);
    }

    void update_state(State next) {
        if (current_state_ == next) return;
        current_state_ = next;
        lock_counter_ = 0;
        
        // Solo habilitar el movimiento si estamos en FOLLOWING
        std_msgs::msg::Bool en;
        en.data = (next == State::FOLLOWING);
        pub_hf_enable_->publish(en);
        
        RCLCPP_INFO(this->get_logger(), "FSM: State -> %d", static_cast<int>(next));
    }

    void callback_master_enable(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        master_enable_ = req->data;
        res->success = true;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_target_filtered_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_hf_enable_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_legs_, sub_body_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_enable_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HumanTrackerAction>());
    rclcpp::shutdown();
    return 0;
}