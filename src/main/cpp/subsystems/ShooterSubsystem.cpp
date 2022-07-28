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
      compressor{frc::PneumaticsModuleType::CTREPCM},
      m_ShooterEncoder{kShooterEncoderPorts[0], kShooterEncoderPorts[1], false, frc::Encoder::k1X},
      m_AimEncoder{kAimEncoderPorts[0], kAimEncoderPorts[1], false, frc::Encoder::k1X}
{
  m_conveyor.SetInverted(true);
  intake.Set(frc::DoubleSolenoid::kReverse);
  // Set the distance per pulse for the encoders
  m_ShooterEncoder.SetDistancePerPulse(kEncoderDistancePerPulse);
  m_AimEncoder.SetDistancePerPulse(kEncoderDistancePerPulse);
  m_ShooterEncoder.SetSamplesToAverage(10);
  m_AimEncoder.SetSamplesToAverage(10);

  ResetEncoders();
}

void ShooterSubsystem::ResetEncoders()
{
  m_ShooterEncoder.Reset();
  m_AimEncoder.Reset();
}
frc2::Command *ShooterSubsystem::SetIntake()
{
  return new frc2::SequentialCommandGroup(
      frc2::InstantCommand([this]
                           { m_intake.Set(1); },
                           {}),
      frc2::InstantCommand([this]
                           { m_conveyor.Set(1); },
                           {}),
      frc2::InstantCommand([this]
                           { intake.Set(intake.kForward); },
                           {}),
      frc2::WaitCommand(200_ms),
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
      frc2::WaitCommand(200_ms),
      frc2::InstantCommand([this]
                           { intake.Set(intake.kOff); },
                           {}));
}
void ShooterSubsystem::SetConveyor(double speed)
{
  m_conveyor.Set(speed);
  m_intake.Set(speed);
}

void ShooterSubsystem::SetTrigger(double speed)
{
  m_conveyor.Set(speed);
  m_trigger.Set(speed);
}

void ShooterSubsystem::SetAim(double position)
{
  
  
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
}
