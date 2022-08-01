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
constexpr int kLeftMotor1Port = 6;
constexpr int kLeftMotor2Port = 8;
constexpr int kRightMotor1Port = 7;
constexpr int kRightMotor2Port = 9;

constexpr int kLeftEncoderPorts[]{0, 1};
constexpr int kRightEncoderPorts[]{2, 3};
constexpr bool kLeftEncoderReversed = false;
constexpr bool kRightEncoderReversed = false;

constexpr auto kTrackwidth = 4.45_m;
extern const frc::DifferentialDriveKinematics kDriveKinematics;

constexpr int kEncoderCPR = 8192;
constexpr double kWheelDiameterInches = 6;
constexpr double kEncoderDistancePerPulse = 0.48/2048.;

// These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
// These characterization values MUST be determined either experimentally or
// theoretically for *your* robot's drive. The Robot Characterization
// Toolsuite provides a convenient tool for obtaining these values for your
// robot.
constexpr auto ks = 1.06_V;
constexpr auto kv = 2.49 * 1_V * 1_s / 1_m;
constexpr auto ka = 0.0375 * 1_V * 1_s * 1_s / 1_m;

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
constexpr int kLeftMotorPort = 4;
constexpr int kRightMotorPort = 5;
constexpr int kConveyorMotorPort = 3;
constexpr int kIntakeMotorPort = 0;
constexpr int kAimMotorPort = 1;
constexpr int kTriggerMotorPort = 1;

constexpr int kShooterEncoderPorts[]{4,5};
constexpr int kAimEncoderPorts[]{6,7};


constexpr int kIntakeUp = 1;
constexpr int kIntakeDown = 0;

constexpr int kMaxSpeed = 1;
constexpr double kMaxSpeedIntake = 0.2;

constexpr int kEncoderCPR = 8192;
constexpr double kWheelDiameterInches = 6;
constexpr double kEncoderDistancePerPulse = 0.48/2048.;
}  // namespace ShooterConstants

namespace ClimberConstants{
constexpr int kClimberMotorPort = 2;
}   // namespace ClimberConstants
namespace OIConstants {
constexpr int kDriverControllerPort = 0;
constexpr int kOperatorControllerPort = 1;

constexpr double limelightMountAngleDegrees = 30.0;
constexpr double limelightLensHeight = 0.50;
constexpr double goalHeight = 2.64;

constexpr int kShooterAimButton = 1;
constexpr int kTriggerButton = 2;
constexpr int kConveyorButton = 5;
constexpr int kConveyorRevetButton = 6;
constexpr int kClimberRevertButton = 8;
constexpr int kIntakeButton = 9;
constexpr int kClimberButton = 12;

}  // namespace OIConstants
