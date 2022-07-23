// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>

using namespace ShooterConstants;
//#define testeSemIntake
ShooterSubsystem::ShooterSubsystem()
    : m_left{kLeftMotorPort},
      m_right{kRightMotorPort},
      m_middle{kMiddleMotorPort},
      m_conveyor{kConveyorMotorPort},
      m_leftIntake{kIntakeLeftMotorPort},
      m_rightIntake{kIntakeRightMotorPort},
      target{kTargetUp, kTargetDown},
      intake{kIntakeUp, kIntakeDown},
      m_intake{kIntakeMotorPort, rev::CANSparkMaxLowLevel::MotorType::kBrushless}
{
  m_shooter.SetInverted(true);
  m_conveyor.SetInverted(true);
  target.Set(frc::DoubleSolenoid::kForward);
#ifndef testeSemIntake
  intake.Set(frc::DoubleSolenoid::kReverse);
#endif
}

void ShooterSubsystem::Periodic()
{
}

void ShooterSubsystem::Set(double speed)
{
  // Implementation of subsystem periodic method goes here.
  ActualSpeed = speed;
  m_motors.Set(speed);
  if (speed > 0)
    shooting = true;
  else
  {
    shooting = false;
  }
}
void ShooterSubsystem::Shoot(bool act)
{
  if (act)
  {
    m_intake.Set(0.5);
    m_middle.Set(-1);
     m_conveyor.Set(-1);

  }
  else
  {
    m_middle.Set(0);
    m_intake.Set(0);
     m_conveyor.Set(0);
  }
}
void ShooterSubsystem::RevertShoot(bool act)
{
  if (act)
  {
    m_middle.Set(1);
    m_shooter.Set(-1);
    m_conveyor.Set(1);
  }
  else
  {
    m_middle.Set(0);
    m_shooter.Set(0);
    m_conveyor.Set(0);
  }
}
void ShooterSubsystem::SetConveyor(double speed)
{
  m_conveyor.Set(speed);
}

void ShooterSubsystem::ToggleTarget()
{
  target.Toggle();
}

void ShooterSubsystem::SetIntake(bool state, bool motorOnly)
{
  if (state == 1)
  {
    #ifndef testeSemIntake
    if(!motorOnly) intake.Set(frc::DoubleSolenoid::kForward);
    #endif
    m_intake.Set(kMaxSpeedIntake);
    m_rightIntake.Set(1);
    m_leftIntake.Set(-1);
  }
  else
  {
    #ifndef testeSemIntake
    if(!motorOnly)intake.Set(frc::DoubleSolenoid::kReverse);
    #endif
    m_intake.Set(0);
    m_rightIntake.Set(0);
    m_leftIntake.Set(0);
  }
}
void ShooterSubsystem::RevertIntake(bool state)
{
  #ifndef testeSemIntake
  intake.Set(frc::DoubleSolenoid::kReverse);
  #endif
  if (state == 1)
  {
    m_intake.Set(-0.5);
    m_rightIntake.Set(-1);
    m_leftIntake.Set(1);
  }
  else
  {
    m_intake.Set(0);
    m_rightIntake.Set(0);
    m_leftIntake.Set(0);
  }
}
void ShooterSubsystem::ToggleIntake()
{
  intake.Toggle();
  if (intake.Get() == frc::DoubleSolenoid::kForward)
  {
    Set(0);
    m_intake.Set(0);
    m_rightIntake.Set(0);
    m_leftIntake.Set(0);
  }
  else
  {
    m_intake.Set(kMaxSpeedIntake);
    m_rightIntake.Set(1);
    m_leftIntake.Set(-1);
  }
}
void ShooterSubsystem::ToggleConveyor()
{
  if (m_conveyor.Get() == 0)
  {
    m_conveyor.Set(-1);
  }
  else
  {
    m_conveyor.Set(0);
  }
}
/*void ShooterSubsystem::SetAutonomous(bool state)
{
  autonomous = state;
  if (autonomous)
    compressor.Stop();
  else
    compressor.Start();
}*/
void ShooterSubsystem::SetCompressor(bool state)
{
  if (state)
    compressor.Start();
  else
    compressor.Stop();
}
double ShooterSubsystem::Get()
{
  // Implementation of subsystem periodic method goes here.
  return ActualSpeed;
}
