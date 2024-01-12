#include "OI.h"

using namespace ControllerConstants;

double OI::GetLeftY() {return frc::ApplyDeadband(_driver_controller.GetLeftY(), JOYSTICK_DEADBAND);}
double OI::GetLeftX() {return frc::ApplyDeadband(_driver_controller.GetLeftX(), JOYSTICK_DEADBAND);}
double OI::GetRightX() {return frc::ApplyDeadband(_driver_controller.GetRightX(), JOYSTICK_DEADBAND);}




bool OI::GetYPressed() {return _driver_controller.GetYButtonPressed();}
bool OI::GetBPressed() {return _driver_controller.GetXButtonPressed();}
bool OI::GetXPressed() {return _driver_controller.GetBButtonPressed();}

double OI::GetA() {return _driver_controller.GetAButton();}