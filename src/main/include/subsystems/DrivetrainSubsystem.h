#ifndef DRIVETRAINSUBSYSTEM_H
#define DRIVETRAINSUBSYSTEM_H

#include "Constants.h"
#include "SwerveModule.h"

#include <units/angle.h>
#include <units/angular_velocity.h>

#include <AHRS.h>
#include <ctre/Phoenix.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

class DrivetrainSubsystem : public frc2::SubsystemBase {
    public:
        DrivetrainSubsystem();
        void Periodic() override;
        void Drive(units::meters_per_second_t x_speed, units::meters_per_second_t y_speed, units::radians_per_second_t rotation, bool open_loop = false);
        void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desired_states, bool open_loop =false);
        void ResetEncoders();
        units::degree_t GetHeading();
        void ZeroHeading();
        units::degrees_per_second_t GetTurnRate();
        frc::Pose2d GetPose();
        void ResetOdometry(frc::Pose2d pose);
        wpi::array<frc::SwerveModulePosition, 4> GetModulePositions();
        frc::ChassisSpeeds GetChassisSpeeds();

        frc::SwerveDriveKinematics<4> kinematics{
            frc::Translation2d{DrivetrainConstants::DRIVETRAIN_LENGTH/2, DrivetrainConstants::DRIVETRAIN_WIDTH/2},
            frc::Translation2d{DrivetrainConstants::DRIVETRAIN_LENGTH/2, -DrivetrainConstants::DRIVETRAIN_WIDTH/2},
            frc::Translation2d{-DrivetrainConstants::DRIVETRAIN_LENGTH/2, DrivetrainConstants::DRIVETRAIN_WIDTH/2},
            frc::Translation2d{-DrivetrainConstants::DRIVETRAIN_LENGTH/2, -DrivetrainConstants::DRIVETRAIN_WIDTH/2},
        };

    private:
        SwerveModule _modules[4]={
            SwerveModule{FL},
            SwerveModule{FR},
            SwerveModule{BL},
            SwerveModule{BR}
        };
        //PigeonIMU _gyro{DrivetrainConstants::GYRO_PORT};
        AHRS* _gyro;

        frc::SwerveDriveOdometry<4> _odometry;
};


#endif