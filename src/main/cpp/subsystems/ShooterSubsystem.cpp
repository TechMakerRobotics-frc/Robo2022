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
      m_AimEncoder{kAimEncoderPorts[0], kAimEncoderPorts[1], true, frc::Encoder::k1X}
{
  m_conveyor.SetInverted(false);
  m_right.SetInverted(true);
  // Set the distance per pulse for the encoders
  m_AimEncoder.SetDistancePerPulse(1);

  m_AimEncoder.SetSamplesToAverage(10);
  m_trigger.SetInverted(false);

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
bool ShooterSubsystem::getAimGreatThen(double value){

  return  m_AimEncoder.GetDistance()>value;
}
bool ShooterSubsystem::getAimLowerThen(double value){
  return  m_AimEncoder.GetDistance()<value;

  }
void ShooterSubsystem::SetAim(units::voltage::volt_t position)
{
  m_aim.SetVoltage(position);
}
void ShooterSubsystem::ActiveAim(double speed)
{
  
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
double ShooterSubsystem::getAimEncoder(){
  return m_AimEncoder.GetDistance();
}
void ShooterSubsystem::Periodic()
{
  frc::SmartDashboard::PutNumber("Encoder Mira", m_AimEncoder.GetDistance());
}
