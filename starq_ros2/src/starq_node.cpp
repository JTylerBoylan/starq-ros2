
#include "starq/odrive/odrive_controller.hpp"
#include "starq/dynamics/starq_fivebar2d.hpp"
#include "starq/leg_controller.hpp"

#include "rclcpp/rclcpp.hpp"
#include "starq_ros2/leg_controller_node.hpp"
#include "starq_ros2/motor_controller_node.hpp"

#define LEG_ID 0x0

#define MOTOR_ID_0 0x0
#define MOTOR_ID_1 0x1

#define LEG_LINK_1_LENGTH_MM 50.0f
#define LEG_LINK_2_LENGTH_MM 150.0f

#define GEAR_RATIO_1 6.0f
#define GEAR_RATIO_2 6.0f

using namespace starq;
using namespace starq::can;
using namespace starq::odrive;
using namespace starq::dynamics;

int main(int argc, char **argv)
{

    printf("Hello world!\n");

    CANSocket::Ptr socket = std::make_shared<CANSocket>("can0");

    if (socket->connect())
    {
        printf("Connected to CAN interface.\n");
    }
    else
    {
        printf("Failed to connect to CAN interface.\n");
        return 1;
    }

    ODriveController::Ptr odrive = std::make_shared<ODriveController>(socket);
    odrive->setGearRatio(MOTOR_ID_0, GEAR_RATIO_1);
    odrive->setGearRatio(MOTOR_ID_1, GEAR_RATIO_2);

    printf("Created ODrive controller.\n");

    LegController::Ptr leg = std::make_shared<LegController>(odrive);

    printf("Created leg controller.\n");

    leg->setMotorIDs(LEG_ID, {MOTOR_ID_0, MOTOR_ID_1});

    printf("Set motor IDs.\n");

    STARQ_FiveBar2D::Ptr fivebar_dynamics = std::make_shared<STARQ_FiveBar2D>(
        LEG_LINK_1_LENGTH_MM,
        LEG_LINK_2_LENGTH_MM);
    leg->setLegDynamics(LEG_ID, fivebar_dynamics);

    printf("Set leg dynamics.\n");

    rclcpp::init(argc, argv);

    auto motor_controller_node = std::make_shared<starq::ros2::MotorControllerNode>(odrive);
    motor_controller_node->setMotorIDs({MOTOR_ID_0, MOTOR_ID_1});

    auto leg_controller_node = std::make_shared<starq::ros2::LegControllerNode>(leg);
    leg_controller_node->setLegIds({LEG_ID});

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(motor_controller_node);
    executor.add_node(leg_controller_node);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}