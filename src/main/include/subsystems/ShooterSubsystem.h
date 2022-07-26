// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Encoder.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>
#include <units/voltage.h>
//#include "WPILib.h"
#include "AHRS.h"
#include <frc/motorcontrol/MotorControllerGroup.h>
#include "ctre/Phoenix.h"
#include <frc/DoubleSolenoid.h>
#include <frc/Compressor.h>
#include "rev/CANSparkMax.h"
#include "rev/CANSparkMaxLowLevel.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/PowerDistribution.h>

#include "Constants.h"

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Subsystem methods go here.

  void Set(double speed);
  void Shoot(bool act);
  void SetConveyor(double speed);
  double Get();
  void ToggleTarget();
  //void SetAutonomous(bool state);
  void ToggleIntake();
  void SetIntake(bool state, bool motorOnly);
  void RevertIntake(bool state);
  void ToggleConveyor();
  void RevertShoot(bool act);
  void SetCompressor(bool state);


 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  bool shooting = false;
  bool autonomous = false;
  // The motor controllers
  WPI_VictorSPX m_left;
  WPI_VictorSPX m_right;
  WPI_VictorSPX m_middle;
  WPI_VictorSPX m_conveyor;
  WPI_VictorSPX m_leftIntake;
  WPI_VictorSPX m_rightIntake;
  
  rev::CANSparkMax m_intake;
  frc::DoubleSolenoid target;
  frc::DoubleSolenoid intake;
  double ActualSpeed;
  frc::PowerDistribution m_pdp;

  frc::Compressor compressor;
  // The motors on the right side of the drive
  frc::MotorControllerGroup m_motors{m_right, m_left};
  frc::MotorControllerGroup m_shooter{m_middle, m_conveyor};

};
