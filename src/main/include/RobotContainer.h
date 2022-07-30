// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc/Timer.h>
#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/ClimberSubsystem.h"
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc/SerialPort.h>
/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer
{
public:
  RobotContainer();

  frc2::Command *FinalAutonomousCommand();
  frc2::Command *GetAutonomousCommand();
  // The chooser for the autonomous routines
  DriveSubsystem m_drive;
  ShooterSubsystem m_shooter;
  ClimberSubsystem m_climber;
  void Periodic();
private:
  // The driver's controller
  frc::XboxController m_driverController{OIConstants::kDriverControllerPort};
  frc::Joystick m_operatorController{OIConstants::kOperatorControllerPort};
  frc::Timer timer;
  frc2::Command *m_IntakeCommand = nullptr;
  void AimTarget();
  frc::SerialPort serial;
  // The robot's subsystems

  // Comandos para o climber
  frc2::InstantCommand m_ClimberSet{[this]
                                    { m_climber.SetClimber(1); },
                                    {}};
  frc2::InstantCommand m_ClimberRevert{[this]
                                       { m_climber.SetClimber(-1); },
                                       {}};
  frc2::InstantCommand m_ClimberReset{[this]
                                      { m_climber.SetClimber(0); },
                                      {}};
  // Final do Climber

  // Comandos para o Intake
  frc2::InstantCommand m_IntakeSet{[this]
                                   {
                                     m_IntakeCommand = m_shooter.SetIntake();
                                     if (m_IntakeCommand != nullptr)
                                       m_IntakeCommand->Schedule();
                                   },
                                   {}};
  frc2::InstantCommand m_IntakeReset{[this]
                                     {
                                       m_IntakeCommand = m_shooter.ResetIntake();
                                       if (m_IntakeCommand != nullptr)
                                         m_IntakeCommand->Schedule();
                                     },
                                     {}};
  // Final do Intake

  // Comandos para o Conveyor
  frc2::InstantCommand m_ConveyorSet{[this]
                                     { m_shooter.SetConveyor(1); },
                                     {}};
  frc2::InstantCommand m_ConveyorReset{[this]
                                       { m_shooter.SetConveyor(0); },
                                       {}};
  frc2::InstantCommand m_ConveyorRevert{[this]
                                        { m_shooter.SetConveyor(-1); },
                                        {}};
  // Final do Conveyor

  // Comandos para o shooter
  frc2::InstantCommand m_TriggerSet{[this]
                                    { m_shooter.SetTrigger(1); },
                                    {}};
  frc2::InstantCommand m_TriggerReset{[this]
                                      { m_shooter.SetTrigger(0); },
                                      {}};
  frc2::InstantCommand m_AimTarget{[this]
                                   {
                                     AimTarget();
                                   },
                                   {}};
  frc2::InstantCommand m_ShooterOn{[this]
                                   {
                                     m_shooter.SetShooter(1);
                                   },
                                   {}};
  frc2::InstantCommand m_ShooterOff{[this]
                                   {
                                     m_shooter.SetShooter(0);
                                   },
                                   {}};
  // Final do shooter

  // The chooser for the autonomous routines
  // The autonomous routines
  frc2::Command *m_simpleAuto = nullptr;
  frc2::Command *m_safetyAuto = nullptr;
  frc2::Command *m_seekAndShootAuto = nullptr;  

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  void ConfigureButtonBindings();


};
