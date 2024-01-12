#include "subsystems/DrivetrainSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace DrivetrainConstants;

DrivetrainSubsystem::DrivetrainSubsystem()
    :_odometry{kinematics, 0_deg, GetModulePositions()}
    {
        _gyro = new AHRS{frc::SPI::Port::kMXP};
        ZeroHeading();
        ResetOdometry({0_m, 0_m, 0_deg});
    }
void DrivetrainSubsystem::Periodic() {
    _odometry.Update(GetHeading(), GetModulePositions());
    frc::SmartDashboard::PutNumber("gyro", GetHeading().value());
    frc::SmartDashboard::PutNumber("Pose X", GetPose().X().value());
    frc::SmartDashboard::PutNumber("Pose Y", GetPose().Y().value());
    frc::SmartDashboard::PutNumber("FL Angle PV", GetModulePositions()[FL].angle.Degrees().value());
    frc::SmartDashboard::PutNumber("FR Angle PV", GetModulePositions()[FR].angle.Degrees().value());
    frc::SmartDashboard::PutNumber("BL Angle PV", GetModulePositions()[BL].angle.Degrees().value());
    frc::SmartDashboard::PutNumber("BR Angle PV", GetModulePositions()[BR].angle.Degrees().value());
}
void DrivetrainSubsystem::Drive(units::meters_per_second_t x_speed, units::meters_per_second_t y_speed, units::radians_per_second_t rotation, bool open_loop) {
    auto states = kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::FromFieldRelativeSpeeds(x_speed, y_speed, rotation, GetHeading()));

    SetModuleStates(states, open_loop);
}

void DrivetrainSubsystem::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desired_states, bool open_loop) {
    kinematics.DesaturateWheelSpeeds(&desired_states, MAX_LINEAR_SPEED);
    frc::SmartDashboard::PutNumber("FL Speed SP", desired_states[FL].speed.value());
    frc::SmartDashboard::PutNumber("FL Angle SP", desired_states[FL].angle.Degrees().value());
    frc::SmartDashboard::PutNumber("FR Speed SP", desired_states[FR].speed.value());
    frc::SmartDashboard::PutNumber("FR Angle SP", desired_states[FR].angle.Degrees().value());
    frc::SmartDashboard::PutNumber("BL Speed SP", desired_states[BL].speed.value());
    frc::SmartDashboard::PutNumber("BL Angle SP", desired_states[BL].angle.Degrees().value());
    frc::SmartDashboard::PutNumber("BR Speed SP", desired_states[BR].speed.value());
    frc::SmartDashboard::PutNumber("BR Angle SP", desired_states[BR].angle.Degrees().value());
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
    ResetOdometry(GetPose());
}

units::degree_t DrivetrainSubsystem::GetHeading(){
    if (_gyro != NULL) {
        return units::degree_t{-_gyro->GetAngle()};
    }

    return 0_deg;
    
}

void DrivetrainSubsystem::ZeroHeading() {
    if (_gyro != NULL) {
        _gyro->ZeroYaw();
    }
    ResetOdometry(GetPose());
}

units::degrees_per_second_t DrivetrainSubsystem::GetTurnRate() {
    if (_gyro != NULL) {
        return units::degrees_per_second_t{-_gyro->GetRate()};
    }
    return 0_deg_per_s;
}

frc::Pose2d DrivetrainSubsystem::GetPose() {
    return _odometry.GetPose();
}

void DrivetrainSubsystem::ResetOdometry(frc::Pose2d pose) {
    _odometry.ResetPosition(GetHeading(), GetModulePositions(), pose);
}

wpi::array<frc::SwerveModulePosition, 4> DrivetrainSubsystem::GetModulePositions() {
    return {_modules[FL].GetPosition(), _modules[FR].GetPosition(), _modules[BL].GetPosition(), _modules[BR].GetPosition()} ;
}

frc::ChassisSpeeds DrivetrainSubsystem::GetChassisSpeeds() {
    return kinematics.ToChassisSpeeds({_modules[FL].GetState(), _modules[FR].GetState(), _modules[BL].GetState(), _modules[BR].GetState()});
}

