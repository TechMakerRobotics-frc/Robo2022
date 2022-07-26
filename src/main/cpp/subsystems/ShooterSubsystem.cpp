// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>

using namespace ShooterConstants;
ShooterSubsystem::ShooterSubsystem()
    : m_left{kLeftMotorPort},
      m_right{kRightMotorPort},
      m_conveyor{kConveyorMotorPort},
      m_intake{kIntakeMotorPort},
      intake{frc::PneumaticsModuleType::CTREPCM, kIntakeUp, kIntakeDown},
      m_trigger{kTriggerMotorPort, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
      compressor{frc::PneumaticsModuleType::CTREPCM},
      m_ShooterEncoder{kShooterEncoderPorts[0], kShooterEncoderPorts[1],false,frc::Encoder::k1X},
      m_AimEncoder{kAimEncoderPorts[0], kAimEncoderPorts[1],false,frc::Encoder::k1X}
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
void ShooterSubsystem::Periodic()
{
}

void ShooterSubsystem::Shoot(bool act)
{
  
}

void ShooterSubsystem::SetConveyor(double speed)
{
  
}

void ShooterSubsystem::ToggleTarget()
{
  
}

void ShooterSubsystem::SetIntake(bool state, bool motorOnly)
{
  
}

void ShooterSubsystem::ToggleIntake()
{
 
}

void ShooterSubsystem::SetCompressor(bool state)
{
  if (state)
    compressor.Enabled();
  else
    compressor.Disable();
}

