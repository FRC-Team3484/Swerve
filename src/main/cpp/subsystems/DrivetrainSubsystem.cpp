#include "subsystems/DrivetrainSubsystem.h"

using namespace DrivetrainConstants;

DrivetrainSubsystem::DrivetrainSubsystem()
    :_odometry{kinematics, GetHeading(), {_modules[FL].GetPosition(), _modules[FR].GetPosition(), _modules[BL].GetPosition(), _modules[BR].GetPosition()}}
    {}
void DrivetrainSubsystem::Periodic() {
    _odometry.Update(GetHeading(), {_modules[FL].GetPosition(), _modules[FR].GetPosition(), _modules[BL].GetPosition(), _modules[BR].GetPosition()});
}
void DrivetrainSubsystem::Drive(units::meters_per_second_t x_speed, units::meters_per_second_t y_speed, units::meters_per_second_t rotation, bool open_loop = false) {
    auto states = kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::FromFieldRelativeSpeeds(x_speed, y_speed, rotation, GetHeading()));

    SetModuleStates(states, open_loop);
}

void DrivetrainSubsystem::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desired_states, bool open_loop) {
    kinematics.DesaturateWheelSpeeds(&desired_states, MAX_LINEAR_SPEED);
    _modules[FL].SetDesiredState(desired_states[FL], open_loop);
    _modules[FR].SetDesiredState(desired_states[FR], open_loop);
    _modules[BL].SetDesiredState(desired_states[BL], open_loop);
    _modules[BR].SetDesiredState(desired_states[BR], open_loop);
}

void DrivetrainSubsystem::ResetEncoders() {
    _modules[FL].ResetEncoder();
    _modules[FR].ResetEncoder();
    _modules[BL].ResetEncoder();
    _modules[BR].ResetEncoder();
}

units::degree_t DrivetrainSubsystem:: GetHeading(){
    return units::degree_t{_gyro.GetYaw()};
}

void DrivetrainSubsystem::ZeroHeading() {
    _gyro.SetYaw(0);
}

units::degrees_per_second_t DrivetrainSubsystem::GetTurnRate() {
    double xyz_dps[3];
    _gyro.GetRawGyro(xyz_dps);
    return units::degrees_per_second_t{xyz_dps[2]};
}

frc::Pose2d DrivetrainSubsystem::GetPose() {
    return _odometry.GetPose();
}

void DrivetrainSubsystem::ResetOdometry(frc::Pose2d pose) {
    _odometry.ResetPosition(GetHeading(),
    {_modules[FL].GetPosition(), _modules[FR].GetPosition(), _modules[BL].GetPosition(), _modules[BR].GetPosition()}, pose);
}



