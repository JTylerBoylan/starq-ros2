#ifndef STARQ_ROS__ODRIVE_CONTROLLER_NODE_HPP_
#define STARQ_ROS__ODRIVE_CONTROLLER_NODE_HPP_

#include "starq_ros2/motor_controller_node.hpp"
#include "starq/odrive/odrive_controller.hpp"
#include "starq_ros2/msg/o_drive_info.hpp"

#define INFO_PUBLISH_RATE_MS 50

namespace starq::ros2
{

    using namespace starq::odrive;

    class ODriveControllerNode : public MotorControllerNode
    {
        public:
            using Ptr = std::shared_ptr<ODriveControllerNode>;

            ODriveControllerNode(ODriveController::Ptr odrive_controller)
                : MotorControllerNode(odrive_controller),
                  odrive_controller_(odrive_controller)
            {
                odrive_info_pub_ = this->create_publisher<starq_ros2::msg::ODriveInfo>("odrive/info", 10);

                odrive_info_timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(INFO_PUBLISH_RATE_MS),
                    std::bind(&ODriveControllerNode::publish_odrive_info, this)
                );

                RCLCPP_INFO(this->get_logger(), "Created ODrive controller node.");
            }

            void publish_odrive_info()
            {
                for (uint8_t motor_id : this->motor_ids_)
                {
                    starq_ros2::msg::ODriveInfo msg;
                    msg.motor_id = motor_id;
                    msg.axis_state = odrive_controller_->getAxisState(motor_id);
                    msg.axis_error = odrive_controller_->getAxisError(motor_id);
                    msg.iq_setpoint = odrive_controller_->getIqSetpoint(motor_id);
                    msg.iq_measured = odrive_controller_->getIqMeasured(motor_id);
                    msg.fet_temperature = odrive_controller_->getFETTemperature(motor_id);
                    msg.motor_temperature = odrive_controller_->getMotorTemperature(motor_id);
                    msg.dc_bus_voltage = odrive_controller_->getBusVoltage(motor_id);
                    msg.dc_bus_current = odrive_controller_->getBusCurrent(motor_id);

                    odrive_info_pub_->publish(msg);
                }
            }
        
        private:
            ODriveController::Ptr odrive_controller_;
            rclcpp::Publisher<starq_ros2::msg::ODriveInfo>::SharedPtr odrive_info_pub_;
            rclcpp::TimerBase::SharedPtr odrive_info_timer_;
    };

}

#endif