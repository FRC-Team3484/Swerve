#include "commands/GoToPoseCommands.h"

using namespace frc;
using namespace units;
using namespace DrivetrainConstants;

GoToPoseCommand::GoToPoseCommand(DrivetrainSubsystem* drivetrain, frc::Pose2d target)
	: _drivetrain{drivetrain}, _target_pose{target}{
	AddRequirements(_drivetrain);
	}
void GoToPoseCommand::Initialize() {};

void GoToPoseCommand::Execute() {
	_pose_delta = _target_pose.RelativeTo(_drivetrain->GetPose());
	
	const frc::ChassisSpeeds chassis_speeds = _drivetrain->GetChassisSpeeds();
	const units:: meters_per_second_t linear_speed = units::math::sqrt(chassis_speeds.vx* chassis_speeds.vx + chassis_speeds.vy * chassis_speeds.vy);

	const Translation2d linear_delta = _pose_delta.Translation();

	const TrapezoidProfile<meters>::State current_linear_state{0_m, linear_speed};

	const TrapezoidProfile<meters>::State target_linear_state{linear_delta.Norm(), 0_mps};

	frc::TrapezoidProfile<units::meters> linear_profile = TrapezoidProfile<meters>{_linear_constraints, target_linear_state, current_linear_state};

	const meters_per_second_t linear_velocity = linear_profile.Calculate(20_ms).velocity;

	const meters_per_second_t x_velocity = linear_velocity* linear_delta.Angle().Cos();
	const meters_per_second_t y_velocity = linear_velocity* linear_delta.Angle().Sin();

	const Rotation2d rotation_delta = _pose_delta.Rotation();

	const TrapezoidProfile<radians>::State current_rotation_state{0_rad, chassis_speeds.omega};

	const TrapezoidProfile<radians>::State target_rotation_state{rotation_delta.Radians(), 0_rad_per_s};

	frc::TrapezoidProfile<units::radians> rotation_profile = TrapezoidProfile<radians>{_rotation_constraints, target_rotation_state, current_rotation_state};

	const radians_per_second_t rotation_velocity = rotation_profile.Calculate(20_ms).velocity;

	_drivetrain->Drive(x_velocity, y_velocity,rotation_velocity, false);
}

void GoToPoseCommand::End(bool Iterrupted) {
	_drivetrain->Drive(0_mps, 0_mps, 0_rad_per_s, true);
}

bool GoToPoseCommand::IsFinished(){
	return _pose_delta.Translation().Norm() <= POSITION_TOLERANCE && units::math::abs(_pose_delta.Rotation().Radians())<= ANGLE_TOLERANCE;
}


