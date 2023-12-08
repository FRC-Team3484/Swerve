#include "commands/TeleopDriveCommand.h"

using namespace DrivetrainConstants;

TeleopDriveCommand::TeleopDriveCommand(DrivetrainSubsystem* drivetrain, OI* oi)
    : _drivetrian_subsystem{drivetrain}, _oi{oi}{
    AddRequirements(_drivetrian_subsystem);    
}

void TeleopDriveCommand::Initialize() {}

void TeleopDriveCommand::Execute() {
    if (_oi->GetA()) {
        _drivetrian_subsystem->ZeroHeading();
    }

    const units::meters_per_second_t x_speed = -_oi->GetLeftY() * MAX_LINEAR_SPEED;
    const units::meters_per_second_t y_speed = -_oi->GetLeftX() * MAX_LINEAR_SPEED;
    const units::meters_per_second_t rotation = -_oi->GetRightX() * MAX_ROTATION_SPEED;

    _drivetrian_subsystem->Drive(x_speed, y_speed, rotation, true);
}

void TeleopDriveCommand::End(bool interrupted) {
    _drivetrian_subsystem->Drive(0_mps, 0_mps, 0_rad_per_s, true);
} 

bool TeleopDriveCommand::IsFinished() {return false;}