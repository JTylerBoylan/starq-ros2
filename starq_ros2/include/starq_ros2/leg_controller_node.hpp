#ifndef STARQ_ROS__LEG_CONTROLLER_NODE_HPP_
#define STARQ_ROS__LEG_CONTROLLER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "starq/leg_controller.hpp"

#include "starq_ros2/msg/leg_command.hpp"
#include "starq_ros2/msg/leg_state.hpp"

#define STATE_PUBLISH_RATE_MS 20

namespace starq::ros2
{

    class LegControllerNode : public rclcpp::Node
    {
    public:
        using Ptr = std::shared_ptr<LegControllerNode>;

        LegControllerNode(LegController::Ptr leg_controller)
            : Node("leg_controller_node"),
              leg_controller_(leg_controller)
        {
            // QoS settings for fast communication
            auto qos_fast = rclcpp::QoS(rclcpp::KeepLast(10));
            qos_fast.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
            qos_fast.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
            qos_fast.deadline(std::chrono::milliseconds(100));

            // Leg command subscriber
            leg_command_sub_ = this->create_subscription<starq_ros2::msg::LegCommand>(
                "leg/cmd", qos_fast,
                std::bind(&LegControllerNode::leg_command_callback, this, std::placeholders::_1));

            // Leg state publisher
            leg_state_pub_ = this->create_publisher<starq_ros2::msg::LegState>("leg/state", 10);

            // Leg state timer
            leg_state_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(STATE_PUBLISH_RATE_MS),
                std::bind(&LegControllerNode::publish_leg_state, this));

            RCLCPP_INFO(this->get_logger(), "Created leg controller node.");
        }

        void setLegIds(const std::vector<uint8_t> &leg_ids)
        {
            leg_ids_ = leg_ids;
        }

    private:
        void leg_command_callback(const starq_ros2::msg::LegCommand::SharedPtr msg)
        {
            Vector3f foot_position, foot_velocity, foot_force;
            ros2eigen(msg->input_position, foot_position);
            ros2eigen(msg->input_velocity, foot_velocity);
            ros2eigen(msg->input_force, foot_force);

            leg_controller_->setControlMode(msg->control_mode, msg->input_mode);
            switch (msg->control_mode)
            {
            case ControlMode::POSITION:
                leg_controller_->setFootPosition(msg->leg_id, foot_position, foot_velocity, foot_force);
                break;
            case ControlMode::VELOCITY:
                leg_controller_->setFootVelocity(msg->leg_id, foot_velocity, foot_force);
                break;
            case ControlMode::TORQUE:
                leg_controller_->setFootForce(msg->leg_id, foot_force);
                break;
            }
        }

        void publish_leg_state()
        {
            for (uint8_t leg_id : leg_ids_)
            {
                VectorXf foot_position, foot_velocity, foot_force;
                leg_controller_->getFootPositionEstimate(leg_id, foot_position);
                leg_controller_->getFootVelocityEstimate(leg_id, foot_velocity);
                leg_controller_->getFootForceEstimate(leg_id, foot_force);

                geometry_msgs::msg::Vector3 foot_position_ros, foot_velocity_ros, foot_force_ros;
                eigen2ros(foot_position, foot_position_ros);
                eigen2ros(foot_velocity, foot_velocity_ros);
                eigen2ros(foot_force, foot_force_ros);

                starq_ros2::msg::LegState msg;
                msg.leg_id = leg_id;
                msg.position_estimate = foot_position_ros;
                msg.velocity_estimate = foot_velocity_ros;
                msg.force_estimate = foot_force_ros;
                leg_state_pub_->publish(msg);
            }
        }

        void ros2eigen(const geometry_msgs::msg::Vector3 &ros, Eigen::Vector3f &eigen)
        {
            eigen.x() = ros.x;
            eigen.y() = ros.y;
            eigen.z() = ros.z;
        }

        void eigen2ros(const Eigen::VectorXf &eigen, geometry_msgs::msg::Vector3 &ros)
        {
            ros.x = eigen.x();
            ros.y = eigen.y();
            ros.z = eigen.z();
        }

        LegController::Ptr leg_controller_;
        rclcpp::Subscription<starq_ros2::msg::LegCommand>::SharedPtr leg_command_sub_;
        rclcpp::Publisher<starq_ros2::msg::LegState>::SharedPtr leg_state_pub_;
        rclcpp::TimerBase::SharedPtr leg_state_timer_;

    protected:
        std::vector<uint8_t> leg_ids_;
    };

}

#endif