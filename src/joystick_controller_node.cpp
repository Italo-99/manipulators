#include "manipulators/JoystickController.h"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto controller = std::make_shared<JoystickController>("manipulator_joystick_controller");
    controller->spinner();

    return 0;
}