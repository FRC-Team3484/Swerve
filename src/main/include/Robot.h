// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef ROBOT_H
#define ROBOT_H

#include "OI.h"
#include "Constants.h"
#include "subsystems/DrivetrainSubsystem.h"
#include "commands/TeleopDriveCommand.h"
#include "commands/GoToPoseCommands.h"

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/geometry/Pose2d.h>


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;


 private:
 OI _oi{};

 DrivetrainSubsystem _drivetrain_subsystem{};

 TeleopDriveCommand _drivetrain_command{&_drivetrain_subsystem, &_oi};

 GoToPoseCommand _auton_command{&_drivetrain_subsystem, frc::Pose2d {1_ft, 0_ft, 0_deg }};
};

#endif