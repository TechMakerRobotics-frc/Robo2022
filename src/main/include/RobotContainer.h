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

  // The chooser for the autonomous routines
  DriveSubsystem m_drive;
  ShooterSubsystem m_shooter;
  ClimberSubsystem m_climber;

private:
  // The driver's controller
  frc::XboxController m_driverController{OIConstants::kDriverControllerPort};
  frc::Joystick m_operatorController{OIConstants::kOperatorControllerPort};
  frc::Timer timer;
  frc2::Command *m_IntakeCommand = nullptr;
  
  // The robot's subsystems

  // Comandos para o climber
  frc2::InstantCommand m_ClimberSet{[this]
                                    { m_climber.setCllimber(1); },
                                    {}};
  frc2::InstantCommand m_ClimberRevert{[this]
                                       { m_climber.setCllimber(-1); },
                                       {}};
  frc2::InstantCommand m_ClimberReset{[this]
                                      { m_climber.setCllimber(0); },
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

  // The chooser for the autonomous routines

  void ConfigureButtonBindings();
};
