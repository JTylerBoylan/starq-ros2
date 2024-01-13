#ifndef STARQ_ROS__MOTOR_CONTROLLER_NODE_HPP_
#define STARQ_ROS__MOTOR_CONTROLLER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "starq/motor_controller.hpp"
#include "starq_ros2/msg/motor_command.hpp"
#include "starq_ros2/msg/motor_state.hpp"

#define STATE_PUBLISH_RATE_MS 20

namespace starq::ros2
{

    class MotorControllerNode : public rclcpp::Node
    {
    public:
        using Ptr = std::shared_ptr<MotorControllerNode>;

        MotorControllerNode(MotorController::Ptr motor_controller)
            : Node("motor_controller_node"),
              motor_controller_(motor_controller)
        {
            // QoS settings for fast communication
            auto qos_fast = rclcpp::QoS(rclcpp::KeepLast(10));
            qos_fast.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
            qos_fast.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
            qos_fast.deadline(std::chrono::milliseconds(100));

            // Motor command subscriber
            motor_command_sub_ = this->create_subscription<starq_ros2::msg::MotorCommand>(
                "motor/cmd", qos_fast,
                std::bind(&MotorControllerNode::motor_command_callback, this, std::placeholders::_1));

            // Motor state publisher
            motor_state_pub_ = this->create_publisher<starq_ros2::msg::MotorState>("motor/state", 10);

            // Motor state timer
            motor_state_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(STATE_PUBLISH_RATE_MS),
                std::bind(&MotorControllerNode::publish_motor_state, this));

            RCLCPP_INFO(this->get_logger(), "Created motor controller node.");
        }

        void setMotorIDs(std::vector<uint8_t> motor_ids)
        {
            motor_ids_ = motor_ids;
        }

    private:
        void motor_command_callback(const starq_ros2::msg::MotorCommand::SharedPtr msg)
        {
            motor_controller_->setState(msg->motor_id, msg->axis_state);
            motor_controller_->setControlMode(msg->motor_id, msg->control_mode, msg->input_mode);

            switch (msg->control_mode)
            {
                case ControlMode::POSITION:
                    motor_controller_->setPosition(msg->motor_id, msg->input_position, msg->input_velocity, msg->input_torque);
                    break;
                case ControlMode::VELOCITY:
                    motor_controller_->setVelocity(msg->motor_id, msg->input_velocity, msg->input_torque);
                    break;
                case ControlMode::TORQUE:
                    motor_controller_->setTorque(msg->motor_id, msg->input_torque);
                    break;
            };
        }

        void publish_motor_state()
        {
            for (uint8_t motor_id : motor_ids_)
            {
                starq_ros2::msg::MotorState msg;
                msg.motor_id = motor_id;
                msg.position_estimate = motor_controller_->getPositionEstimate(motor_id);
                msg.velocity_estimate = motor_controller_->getVelocityEstimate(motor_id);
                msg.torque_estimate = motor_controller_->getTorqueEstimate(motor_id);

                motor_state_pub_->publish(msg);
            }
        }

        MotorController::Ptr motor_controller_;
        rclcpp::Subscription<starq_ros2::msg::MotorCommand>::SharedPtr motor_command_sub_;
        rclcpp::Publisher<starq_ros2::msg::MotorState>::SharedPtr motor_state_pub_;
        rclcpp::TimerBase::SharedPtr motor_state_timer_;

        protected:
        std::vector<uint8_t> motor_ids_;
    };

}

#endif