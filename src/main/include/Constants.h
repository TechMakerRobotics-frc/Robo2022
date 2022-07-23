// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/numbers>
#include <frc/XboxController.h>


#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace DriveConstants {
constexpr int kLeftMotor1Port = 4;
constexpr int kLeftMotor2Port = 6;
constexpr int kRightMotor1Port = 5;
constexpr int kRightMotor2Port = 7;

constexpr int kLeftEncoderPorts[]{8, 7};
constexpr int kRightEncoderPorts[]{4, 3};
constexpr bool kLeftEncoderReversed = false;
constexpr bool kRightEncoderReversed = false;

constexpr auto kTrackwidth = 4.45_m;
extern const frc::DifferentialDriveKinematics kDriveKinematics;

constexpr int kEncoderCPR = 8192;
constexpr double kWheelDiameterInches = 6;
constexpr double kEncoderDistancePerPulse = 0.48/2048.;
    // Assumes the encoders are directly mounted on the wheel shafts
    //(kWheelDiameterInches * wpi::numbers::pi) /
    //static_cast<double>(kEncoderCPR);

// These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
// These characterization values MUST be determined either experimentally or
// theoretically for *your* robot's drive. The Robot Characterization
// Toolsuite provides a convenient tool for obtaining these values for your
// robot.
constexpr auto ks = 1.06_V;
constexpr auto kv = 2.49 * 1_V * 1_s / 1_m;
constexpr auto ka = 0.0375 * 1_V * 1_s * 1_s / 1_m;

// Example value only - as above, this must be tuned for your drive!
//constexpr double kPDriveVel = 0.848;
/*constexpr double kPDriveVel = 0.848;
constexpr double kIDriveVel = 0.4;
constexpr double kDDriveVel = 0.1;*/
constexpr double kPDriveVel = 0.1;
constexpr double kIDriveVel = 0.0;
constexpr double kDDriveVel = 0.0;
}  // namespace DriveConstants
namespace AutoConstants {
constexpr auto kMaxSpeed = 0.5_mps;
constexpr auto kMaxAcceleration = 1_mps_sq;

// Reasonable baseline values for a RAMSETE follower in units of meters and
// seconds
constexpr double kRamseteB = 2;
constexpr double kRamseteZeta = 0.7;
}  // namespace AutoConstants


namespace ShooterConstants {
constexpr int kLeftMotorPort = 1;
constexpr int kRightMotorPort = 2;
constexpr int kMiddleMotorPort = 3;
constexpr int kConveyorMotorPort = 0;
constexpr int kIntakeMotorPort = 8;
constexpr int kIntakeLeftMotorPort = 9;
constexpr int kIntakeRightMotorPort = 10;


constexpr int kTargetUp = 2;
constexpr int kTargetDown = 3;

constexpr int kIntakeUp = 0;
constexpr int kIntakeDown = 1;

constexpr int kMaxSpeed = 1;
constexpr double kMaxSpeedIntake = 0.2;
}  // namespace DriveConstants
constexpr frc::GenericHID::JoystickHand lHand = frc::GenericHID::kLeftHand;
constexpr frc::GenericHID::JoystickHand rHand = frc::GenericHID::kRightHand;
namespace OIConstants {
constexpr int kDriverControllerPort = 0;
constexpr int kDriverControllerPort2 = 1;

 
}  // namespace OIConstants
