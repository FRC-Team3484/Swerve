// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "OI.h"
#include <frc/XboxController.h>


#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  v = 0;
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() 
{
  frc2::CommandScheduler::GetInstance().Run();
  frc::SmartDashboard::PutString("Auton Cmd State", _auton_command.IsScheduled() ? "Scheduled" : "Not Scheduled");

  if (frc::RobotBase::IsAutonomous() && _auton_command.IsFinished())
  {
    _drivetrain_subsystem.Drive(0_mps, 0_mps, 0_rad_per_s, true);
  }
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {
  if(_auton_command.IsScheduled())
    _auton_command.Cancel();
}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  _auton_command.Schedule(); 

}

void Robot::AutonomousPeriodic() 
{
}

void Robot::TeleopInit() {
  if( _auton_command.IsScheduled())
    _auton_command.Cancel();

  _drivetrain_command.Schedule();



}
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  
  

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
  if (_oi.GetYPressed()){
    v += 0.1;
      if (v>1) {
      v = 1;
    }
  }

  else if (_oi.GetBPressed()){
    v -= 0.1;
    if (v<0) {
      v = 0;
    }     
  }
  else if (_oi.GetXPressed()){
   v = 0;
 
  }

   Motor1.Set(v);
   Motor2.Set(v);
   Motor1.Feed();
   Motor2.Feed();

}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
