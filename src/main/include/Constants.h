// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef CONSTANTS_H
#define CONSTANTS_H 

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */
 
#include <units/voltage.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h> 

namespace ControllerConstants {
    constexpr int DRIVER_CONTROLLER_PORT = 0;
    constexpr double JOYSTICK_DEADBAND = 0.02;
}

namespace DrivetrainConstants {
    #define FL 0
    #define FR 1
    #define BL 2
    #define BR 3

    constexpr int DRIVE_MOTOR_PORTS[] = {0, 1, 2, 3};
    constexpr int STEER_MOTOR_PORTS[] = {4, 5, 6, 7};
    constexpr int ENCODER_PORTS[] = {8, 9, 10, 11};

    constexpr bool STEER_MOTOR_REVERSED[] = {false, false, false, false};
    constexpr bool ENCODER_REVERSED[] = {false, false, false, false};

    constexpr double ENCODER_OFFSET[] = {0.0, 0.0, 0.0, 0.0};

    constexpr units::inch_t DRIVETRAIN_WIDTH = 25_in;
    constexpr units::inch_t DRIVETRAIN_LENGTH = 25_in;
    constexpr double DRIVE_GEAR_RATIO = 36000.0/5880.0;
    constexpr double STEER_GEAR_RATIO = 12.8;
    constexpr int DRIVE_TICKS_PER_REVOLUTION =2048;
    constexpr int STEER_TICKS_PER_REVOLUTION = 2048;
    constexpr units::inch_t WHEEL_RADIUS = 2_in;

    constexpr units::feet_per_second_t MAX_LINEAR_SPEED = 8_fps;
    constexpr units::radians_per_second_t MAX_ROTATION_SPEED = 5.431_rad_per_s;
}

namespace DrivePIDConstants {
    constexpr double P = 1.0;
    constexpr double I = 0.0;
    constexpr double D = 0.0;
}
namespace SteerPIDConstants {
    constexpr double P = 1.0;
    constexpr double I = 0.0;
    constexpr double D = 0.0;
    constexpr units::radians_per_second_t MAX_SPEED = 3_rad_per_s;
    constexpr units::radians_per_second_squared_t MAX_ACCELRATION  = 6_rad_per_s_sq;
}
namespace DriveFeedForwardConstants {
    constexpr units::volt_t S = 1.0_V;
    constexpr auto V = 0.8_V / 1.0_mps;
    constexpr auto A = 0.15_V / 1.0_mps_sq;
}

#endif