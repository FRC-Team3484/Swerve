#include "OI.h"

using namespace ControllerConstants;

double OI::GetLeftY() {return frc::ApplyDeadband(_driver_controller.GetLeftY(), JOYSTICK_DEADBAND);}
double OI::GetLeftX() {return frc::ApplyDeadband(_driver_controller.GetLeftX(), JOYSTICK_DEADBAND);}
double OI::GetRightX() {return frc::ApplyDeadband(_driver_controller.GetRightX(), JOYSTICK_DEADBAND);}

double OI::GetA() {return _driver_controller.GetAButton();}