#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <string>
#include <vector>

#include "take_interfaces/srv/take.hpp"
#include "manipulation_tools/Manipulation.hpp"

using Take = take_interfaces::srv::Take;

enum class State
{
    PLAN_REACH,
    EXECUTE_REACH,
    CLOSE_GRIPPER,
    PLAN_RETRACT,
    EXECUTE_RETRACT,
    DONE
};

static constexpr int MAX_ATTEMPTS = 2;

static std::string stateName(State s)
{
    switch (s)
    {
        case State::PLAN_REACH:      return "PLAN_REACH";
        case State::EXECUTE_REACH:   return "EXECUTE_REACH";
        case State::CLOSE_GRIPPER:   return "CLOSE_GRIPPER";
        case State::PLAN_RETRACT:    return "PLAN_RETRACT";
        case State::EXECUTE_RETRACT: return "EXECUTE_RETRACT";
        case State::DONE:            return "DONE";
        default:                     return "UNKNOWN";
    }
}

class TakeNode : public rclcpp::Node
{
public:
    TakeNode() : Node("take_node")
    {
        srv_cb_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        service_ = this->create_service<Take>(
            "take",
            std::bind(&TakeNode::takeCallback, this,
                      std::placeholders::_1, std::placeholders::_2),
             rmw_qos_profile_services_default,
            srv_cb_group_);

        RCLCPP_INFO(this->get_logger(), "Take node ready");
    }

    void init()
    {
        manipulation_ = std::make_shared<Manipulation>(this->shared_from_this());
    }

private:
    rclcpp::CallbackGroup::SharedPtr srv_cb_group_;
    rclcpp::Service<Take>::SharedPtr service_;
    std::shared_ptr<Manipulation>    manipulation_;

    State       state_        = State::PLAN_REACH;
    bool        has_previous_ = false;
    std::string last_arm_;
    geometry_msgs::msg::PoseStamped last_pose_;

    std::vector<std::string> joint_names_;
    std::vector<double>      waypoints_;

    bool isSameRequest(
        const geometry_msgs::msg::PoseStamped& pose,
        const std::string& arm) const
    {
        if (!has_previous_) return false;
        if (arm != last_arm_) return false;

        const auto& a = last_pose_.pose;
        const auto& b = pose.pose;

        return last_pose_.header.frame_id == pose.header.frame_id &&
               a.position.x    == b.position.x    &&
               a.position.y    == b.position.y    &&
               a.position.z    == b.position.z    &&
               a.orientation.x == b.orientation.x &&
               a.orientation.y == b.orientation.y &&
               a.orientation.z == b.orientation.z &&
               a.orientation.w == b.orientation.w;
    }

    geometry_msgs::msg::PoseStamped retractPose(const std::string& arm) const
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id    = "base_link";
        pose.pose.position.x    =  0.4;
		pose.pose.position.y = (arm == "right_arm_group") ? -0.35 : 0.35;
        pose.pose.position.z    =  0.9;
        pose.pose.orientation.x =  0.0;
        pose.pose.orientation.y =  -0.707;
        pose.pose.orientation.z =  0.0;
        pose.pose.orientation.w =  0.707;

        return pose;
    }

    template<typename Fn>
    bool attempt(const std::string& step, Fn&& fn)
    {
        for (int i = 1; i <= MAX_ATTEMPTS; ++i)
        {
            RCLCPP_INFO(this->get_logger(),
                "STATE: %s — intento %d/%d", step.c_str(), i, MAX_ATTEMPTS);

            std::string msg;
            if (fn(msg))
            {
                RCLCPP_INFO(this->get_logger(),
                    "%s completado en intento %d/%d", step.c_str(), i, MAX_ATTEMPTS);
                return true;
            }

            RCLCPP_WARN(this->get_logger(),
                "%s falló (intento %d/%d): %s",
                step.c_str(), i, MAX_ATTEMPTS, msg.c_str());
        }

        RCLCPP_ERROR(this->get_logger(),
            "%s agotó %d intentos — abortando", step.c_str(), MAX_ATTEMPTS);
        return false;
    }

    void takeCallback(
        const std::shared_ptr<Take::Request>  request,
        std::shared_ptr<Take::Response>       response)
    {
        std::string arm;
        if      (request->arm == "right") arm = "right_arm_group";
        else if (request->arm == "left")  arm = "left_arm_group";
        else
        {
            response->success = false;
            response->message = "Brazo inválido: usa 'right' o 'left'";
            return;
        }

        if (isSameRequest(request->target_pose, arm) && state_ != State::DONE)
        {
            RCLCPP_INFO(this->get_logger(),
                "Retry detectado — reanudando desde %s",
                stateName(state_).c_str());
        }
        else
        {
            RCLCPP_INFO(this->get_logger(),
                "Nueva tarea — reiniciando SM desde PLAN_REACH");
            state_ = State::PLAN_REACH;
            joint_names_.clear();
            waypoints_.clear();
        }

        last_pose_    = request->target_pose;
        last_arm_     = arm;
        has_previous_ = true;

        while (rclcpp::ok())
        {
            switch (state_)
            {
                // ── PLAN_REACH ────────────────────────────────
                case State::PLAN_REACH:
                {
                    bool ok = attempt("PLAN_REACH", [&](std::string& msg) {
                        return manipulation_->PlanTrajectory(
                            request->target_pose, arm,
                            joint_names_, waypoints_, msg);
                    });

                    if (!ok)
                    {
                        response->success = false;
                        response->message = "PLAN_REACH: agotados " +
                            std::to_string(MAX_ATTEMPTS) + " intentos";
                        return;
                    }
                    state_ = State::EXECUTE_REACH;
                    break;
                }

                // ── EXECUTE_REACH ─────────────────────────────
                case State::EXECUTE_REACH:
                {
                    bool ok = attempt("EXECUTE_REACH", [&](std::string& msg) {
                        return manipulation_->FollowTrajectory(
                            arm, joint_names_, waypoints_, msg);
                    });

                    if (!ok)
                    {
                        response->success = false;
                        response->message = "EXECUTE_REACH: agotados " +
                            std::to_string(MAX_ATTEMPTS) + " intentos";
                        return;
                    }
                    state_ = State::CLOSE_GRIPPER;
                    break;
                }

                // ── CLOSE_GRIPPER ─────────────────────────────
                case State::CLOSE_GRIPPER:
                {
                    bool ok = attempt("CLOSE_GRIPPER", [&](std::string& msg) {
                        // TODO: reemplazar con llamada real al gripper
                        RCLCPP_INFO(this->get_logger(), "Cerrando gripper (fake)");
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                        msg = "";
                        return true;
                    });

                    if (!ok)
                    {
                        response->success = false;
                        response->message = "CLOSE_GRIPPER: agotados " +
                            std::to_string(MAX_ATTEMPTS) + " intentos";
                        return;
                    }
                    state_ = State::PLAN_RETRACT;
                    break;
                }

                // ── PLAN_RETRACT ──────────────────────────────
                case State::PLAN_RETRACT:
                {
                    const auto home = retractPose(arm);

                    bool ok = attempt("PLAN_RETRACT", [&](std::string& msg) {
                        return manipulation_->PlanTrajectory(
                            home, arm, joint_names_, waypoints_, msg);
                    });

                    if (!ok)
                    {
                        response->success = false;
                        response->message = "PLAN_RETRACT: agotados " +
                            std::to_string(MAX_ATTEMPTS) + " intentos";
                        return;
                    }
                    state_ = State::EXECUTE_RETRACT;
                    break;
                }

                // ── EXECUTE_RETRACT ───────────────────────────
                case State::EXECUTE_RETRACT:
                {
                    bool ok = attempt("EXECUTE_RETRACT", [&](std::string& msg) {
                        return manipulation_->FollowTrajectory(
                            arm, joint_names_, waypoints_, msg);
                    });

                    if (!ok)
                    {
                        response->success = false;
                        response->message = "EXECUTE_RETRACT: agotados " +
                            std::to_string(MAX_ATTEMPTS) + " intentos";
                        return;
                    }
                    state_ = State::DONE;
                    break;
                }

                // ── DONE ──────────────────────────────────────
                case State::DONE:
                {
                    RCLCPP_INFO(this->get_logger(), "STATE: DONE — tarea completada");
                    response->success = true;
                    response->message = "Take completado exitosamente";

                    state_        = State::PLAN_REACH;
                    has_previous_ = false;
                    return;
                }
            }
        }

        response->success = false;
        response->message = "Ejecución interrumpida (rclcpp shutdown)";
    }
};

// ---------------------------------------------------------
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<TakeNode>();
    node->init();
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
