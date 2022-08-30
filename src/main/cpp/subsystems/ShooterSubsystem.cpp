// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>

using namespace ShooterConstants;
ShooterSubsystem::ShooterSubsystem()
    : m_left{kLeftMotorPort},
      m_right{kRightMotorPort},
      m_conveyor{kConveyorMotorPort},
      m_intake{kIntakeMotorPort},
      intake{frc::PneumaticsModuleType::CTREPCM, kIntakeUp, kIntakeDown},
      m_trigger{kTriggerMotorPort, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
      m_aim{kAimMotorPort},
      compressor{frc::PneumaticsModuleType::CTREPCM},
      m_AimEncoder{kAimEncoderPorts[0], kAimEncoderPorts[1], false, frc::Encoder::k1X}
{
  m_conveyor.SetInverted(false);
  m_right.SetInverted(true);
  // Set the distance per pulse for the encoders
  m_AimEncoder.SetDistancePerPulse(1);

  m_AimEncoder.SetSamplesToAverage(10);
  m_trigger.SetInverted(true);

  ResetEncoders();
}

void ShooterSubsystem::ResetEncoders()
{
  m_AimEncoder.Reset();
}
frc2::Command *ShooterSubsystem::SetIntake()
{
  return new frc2::SequentialCommandGroup(
      frc2::InstantCommand([this]
                           { m_intake.Set(0.5); },
                           {}),
      frc2::InstantCommand([this]
                           { m_conveyor.Set(0.5); },
                           {}),
      frc2::InstantCommand([this]
                           { intake.Set(intake.kForward); },
                           {}),
      frc2::WaitCommand(500_ms),
      frc2::InstantCommand([this]
                           { intake.Set(intake.kOff); },
                           {}));
}

frc2::Command *ShooterSubsystem::ResetIntake()
{
  return new frc2::SequentialCommandGroup(
      frc2::InstantCommand([this]
                           { m_intake.Set(0); },
                           {}),
      frc2::InstantCommand([this]
                           { m_conveyor.Set(0); },
                           {}),
      frc2::InstantCommand([this]
                           { intake.Set(intake.kReverse); },
                           {}),
      frc2::WaitCommand(500_ms),
      frc2::InstantCommand([this]
                           { intake.Set(intake.kOff); },
                           {}));
}
void ShooterSubsystem::SetConveyor(double speed)
{
  m_conveyor.Set(speed);
}

void ShooterSubsystem::SetTrigger(double speed)
{
  if(speed>0.5)
  m_conveyor.Set(0.5);
  else
  m_conveyor.Set(speed);
  m_trigger.Set(speed);
}

void ShooterSubsystem::SetAim(double position)
{
  double speed = 0;
  if (m_AimEncoder.GetDistance() > position)
  {
    speed = 0.5;
    while (m_AimEncoder.GetDistance()> position)
    {
      m_aim.Set(speed);
    }
  }
  else if (m_AimEncoder.GetDistance() < position){
    speed = -0.5;
    while (m_AimEncoder.GetDistance()< position)
    {
      m_aim.Set(speed);
    }
  }

  m_aim.Set(0);
}
void ShooterSubsystem::ActiveAim(double speed)
{
  if((speed>0 && m_AimEncoder.GetDistance()<400) ||(speed<0 && m_AimEncoder.GetDistance()<20))
   m_aim.Set(speed);
}

void ShooterSubsystem::SetShooter(double speed)
{
  m_motors.Set(speed);
}

void ShooterSubsystem::SetCompressor(bool state)
{
  if (state)
    compressor.Enabled();
  else
    compressor.Disable();
}
void ShooterSubsystem::Periodic()
{
  frc::SmartDashboard::PutNumber("Encoder Mira", m_AimEncoder.GetDistance());
}
