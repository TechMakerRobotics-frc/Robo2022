// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <utility>

#include <frc/controller/PIDController.h>
#include <frc/controller/RamseteController.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RamseteCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include "cameraserver/CameraServer.h"
#include "Constants.h"

#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/span.h"
#include <frc/SerialPort.h>
#include <frc/smartdashboard/SmartDashboard.h>

double targetOffsetAngle_Horizontal;
double targetOffsetAngle_Vertical;
double targetArea;
double targetSkew;

double angleToGoalDegrees;
double angleToGoalRadians;
double distanceFromLimelightToGoal;
cs::UsbCamera camera;
cs::VideoSink server;

using namespace OIConstants;
RobotContainer::RobotContainer() : serial{115200, frc::SerialPort::Port::kOnboard, 8, frc::SerialPort::Parity::kParity_None, frc::SerialPort::StopBits::kStopBits_One}
{
    // Initialize all of your commands and subsystems here

    // Configure the button bindings
    ConfigureButtonBindings();
    // The chooser for the autonomous routines
    // Set up default drive command
    m_drive.SetDefaultCommand(frc2::RunCommand(
        [this]
        {
            m_drive.ArcadeDrive(m_driverController.GetLeftY(),
                                -m_driverController.GetRightX());
        },
        {&m_drive}));

    // Add commands to the autonomous command chooser
    m_chooser.SetDefaultOption("Autonomo Simples - TARMAK 2", 1);
    m_chooser.AddOption("Autonomo Seguro - TARMAK 4", 2);
    m_chooser.AddOption("Autonomo Seek'n'Shoot - TARMAK 1", 3);

    // Put the chooser on the dashboard
    frc::SmartDashboard::PutData(&m_chooser);
    camera = frc::CameraServer::StartAutomaticCapture();
    camera.SetResolution(640, 480);
    server = frc::CameraServer::GetServer();
    server.SetSource(camera);
}
frc2::Command *RobotContainer::GetAutonomousCommand()
{
    if (m_chooser.GetSelected() == 1)
    {
        // Create a voltage constraint to ensure we don't accelerate too fast
        frc::DifferentialDriveVoltageConstraint autoVoltageConstraint(
            frc::SimpleMotorFeedforward<units::meters>(
                DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
            DriveConstants::kDriveKinematics, 10_V);

        // Set up config for trajectory
        frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                                     AutoConstants::kMaxAcceleration);
        // Add kinematics to ensure max speed is actually obeyed
        config.SetKinematics(DriveConstants::kDriveKinematics);
        // Apply the voltage constraint
        config.AddConstraint(autoVoltageConstraint);

        auto Trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
            frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
            {},
            frc::Pose2d(-1_m, 0_m, frc::Rotation2d(90_deg)),
            config);

        auto Trajectory2 = frc::TrajectoryGenerator::GenerateTrajectory(
            frc::Pose2d(-1_m, 0_m, frc::Rotation2d(90_deg)),
            {},
            frc::Pose2d(-1_m, 0_m, frc::Rotation2d(0_deg)),
            config);

        frc2::RamseteCommand ramseteCommand(
            Trajectory, [this]()
            { return m_drive.GetPose(); },
            frc::RamseteController(AutoConstants::kRamseteB,
                                   AutoConstants::kRamseteZeta),
            frc::SimpleMotorFeedforward<units::meters>(
                DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
            DriveConstants::kDriveKinematics,
            [this]
            { return m_drive.GetWheelSpeeds(); },
            frc2::PIDController(DriveConstants::kPDriveVel, DriveConstants::kIDriveVel, DriveConstants::kDDriveVel),
            frc2::PIDController(DriveConstants::kPDriveVel, DriveConstants::kIDriveVel, DriveConstants::kDDriveVel),
            [this](auto left, auto right)
            { m_drive.TankDriveVolts(left, right); },
            {&m_drive});
        frc2::RamseteCommand ramseteCommand2(
            Trajectory2, [this]()
            { return m_drive.GetPose(); },
            frc::RamseteController(AutoConstants::kRamseteB,
                                   AutoConstants::kRamseteZeta),
            frc::SimpleMotorFeedforward<units::meters>(
                DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
            DriveConstants::kDriveKinematics,
            [this]
            { return m_drive.GetWheelSpeeds(); },
            frc2::PIDController(DriveConstants::kPDriveVel, DriveConstants::kIDriveVel, DriveConstants::kDDriveVel),
            frc2::PIDController(DriveConstants::kPDriveVel, DriveConstants::kIDriveVel, DriveConstants::kDDriveVel),
            [this](auto left, auto right)
            { m_drive.TankDriveVolts(left, right); },
            {&m_drive});

        return new frc2::SequentialCommandGroup(
            // ligo o compressor
            frc2::InstantCommand([this]
                                 { m_shooter.SetCompressor(1); },
                                 {}),
            m_ShooterOn,
            frc2::WaitCommand(3_s),
            m_TriggerSet,
            frc2::WaitCommand(2_s),
            m_TriggerReset,
            m_ShooterOff,
            m_IntakeSet,

            std::move(ramseteCommand),
            frc2::InstantCommand([this]
                                 { m_drive.TankDriveVolts(0_V, 0_V); },
                                 {}),
            frc2::WaitCommand(5_s),
            std::move(ramseteCommand2),
            frc2::InstantCommand([this]
                                 { m_drive.TankDriveVolts(0_V, 0_V); },
                                 {}),
            m_ShooterOn,
            frc2::WaitCommand(2_s),
            m_TriggerSet,
            frc2::WaitCommand(2_s),
            m_TriggerReset,
            m_ShooterOff);
    }
    if (m_chooser.GetSelected() == 2)
    {
        frc::DifferentialDriveVoltageConstraint autoVoltageConstraint(
            frc::SimpleMotorFeedforward<units::meters>(
                DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
            DriveConstants::kDriveKinematics, 10_V);

        // Set up config for trajectory
        frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                                     AutoConstants::kMaxAcceleration);
        // Add kinematics to ensure max speed is actually obeyed
        config.SetKinematics(DriveConstants::kDriveKinematics);
        // Apply the voltage constraint
        config.AddConstraint(autoVoltageConstraint);

        auto Trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
            frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
            {frc::Translation2d(-1.75_m, 0_m)},
            frc::Pose2d(-1.75_m, -2_m, frc::Rotation2d(90_deg)),
            config);

        frc2::RamseteCommand ramseteCommand(
            Trajectory, [this]()
            { return m_drive.GetPose(); },
            frc::RamseteController(AutoConstants::kRamseteB,
                                   AutoConstants::kRamseteZeta),
            frc::SimpleMotorFeedforward<units::meters>(
                DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
            DriveConstants::kDriveKinematics,
            [this]
            { return m_drive.GetWheelSpeeds(); },
            frc2::PIDController(DriveConstants::kPDriveVel, DriveConstants::kIDriveVel, DriveConstants::kDDriveVel),
            frc2::PIDController(DriveConstants::kPDriveVel, DriveConstants::kIDriveVel, DriveConstants::kDDriveVel),
            [this](auto left, auto right)
            { m_drive.TankDriveVolts(left, right); },
            {&m_drive});

        return new frc2::SequentialCommandGroup(
            // ligo o compressor
            frc2::InstantCommand([this]
                                 { m_shooter.SetCompressor(1); },
                                 {}),
            m_ShooterOn,
            frc2::WaitCommand(2_s),
            m_TriggerSet,
            frc2::WaitCommand(2_s),
            m_TriggerReset,
            m_ShooterOff,
            m_IntakeSet,

            std::move(ramseteCommand),
            frc2::InstantCommand([this]
                                 { m_drive.TankDriveVolts(0_V, 0_V); },
                                 {}),

            m_ShooterOff);
    }
    if (m_chooser.GetSelected() == 3)
    {
        frc::DifferentialDriveVoltageConstraint autoVoltageConstraint(
            frc::SimpleMotorFeedforward<units::meters>(
                DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
            DriveConstants::kDriveKinematics, 10_V);

        // Set up config for trajectory
        frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                                     AutoConstants::kMaxAcceleration);
        // Add kinematics to ensure max speed is actually obeyed
        config.SetKinematics(DriveConstants::kDriveKinematics);
        // Apply the voltage constraint
        config.AddConstraint(autoVoltageConstraint);

        auto Trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
            frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
            {frc::Translation2d(-1_m, 0_m),
             frc::Translation2d(-1_m, 1_m),
             frc::Translation2d(-1_m, -1_m)},
            frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
            config);

        frc2::RamseteCommand ramseteCommand(
            Trajectory, [this]()
            { return m_drive.GetPose(); },
            frc::RamseteController(AutoConstants::kRamseteB,
                                   AutoConstants::kRamseteZeta),
            frc::SimpleMotorFeedforward<units::meters>(
                DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
            DriveConstants::kDriveKinematics,
            [this]
            { return m_drive.GetWheelSpeeds(); },
            frc2::PIDController(DriveConstants::kPDriveVel, DriveConstants::kIDriveVel, DriveConstants::kDDriveVel),
            frc2::PIDController(DriveConstants::kPDriveVel, DriveConstants::kIDriveVel, DriveConstants::kDDriveVel),
            [this](auto left, auto right)
            { m_drive.TankDriveVolts(left, right); },
            {&m_drive});

        return new frc2::SequentialCommandGroup(
            // ligo o compressor
            frc2::InstantCommand([this]
                                 { m_shooter.SetCompressor(1); },
                                 {}),
            m_ShooterOn,
            frc2::WaitCommand(2_s),
            m_TriggerSet,
            frc2::WaitCommand(2_s),
            m_TriggerReset,
            m_ShooterOff,
            m_IntakeSet,

            std::move(ramseteCommand),
            frc2::InstantCommand([this]
                                 { m_drive.TankDriveVolts(0_V, 0_V); },
                                 {}),
            m_ShooterOn,
            frc2::WaitCommand(2_s),
            m_TriggerSet,
            frc2::WaitCommand(10_s),
            m_TriggerReset,
            m_ShooterOff,
            m_IntakeSet,
            m_ShooterOff);
    }
}
void RobotContainer::Periodic()
{

    std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    targetOffsetAngle_Horizontal = table->GetNumber("tx", 0.0);
    targetOffsetAngle_Vertical = table->GetNumber("ty", 0.0);
    targetArea = table->GetNumber("ta", 0.0);
    targetSkew = table->GetNumber("ts", 0.0);
    angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    // calculate distance
    distanceFromLimelightToGoal = (goalHeight - limelightLensHeight) / tan(angleToGoalRadians);
    frc::SmartDashboard::PutNumber("Alvo - Angulo Horizontal", targetOffsetAngle_Horizontal);
    frc::SmartDashboard::PutNumber("Alvo - Angulo Vertical", targetOffsetAngle_Vertical);
    frc::SmartDashboard::PutNumber("Area do Alvo", targetArea);
    frc::SmartDashboard::PutNumber("Angulo para o Alvo", angleToGoalDegrees);
    frc::SmartDashboard::PutNumber("Distancia para o Alvo", distanceFromLimelightToGoal);
    frc::SmartDashboard::PutNumber("LeftY", m_driverController.GetLeftY());
    frc::SmartDashboard::PutNumber("RightX", m_driverController.GetRightX());
}

void RobotContainer::ConfigureButtonBindings()
{

    // Os comandos do Joystick foram desabilitados para evitar acionamentos por acidentes
    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kA)
        .WhenPressed(&m_ClimberSet)
        .WhenReleased(&m_ClimberReset);
    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kB)
        .WhenPressed(&m_ClimberRevert)
        .WhenReleased(&m_ClimberReset);
    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kRightBumper)
        .WhenPressed(&m_AimForward)
        .WhenReleased(&m_AimReset);
    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kLeftBumper)
        .WhenPressed(&m_AimRewind)
        .WhenReleased(&m_AimReset);

    /*
        Controle personalizado

        9  10  11  12
        5   6   7   8
        1   2   3   4

        apenas enquanto pressionadas as teclas

        1-Shooter aim
        2-Trigger
        5-Conveyor
        6-Conveyor Out
        8-Climber Revert
        9-Intake
        12-Climber

    */
    /*Atribuição de botoes para climber
      um botão para o acionamento normal
      um para reverter o climber para a posição inicial
    */

    frc2::JoystickButton(&m_operatorController, kClimberButton)
        .WhenPressed(&m_ClimberSet)
        .WhenReleased(&m_ClimberReset);
    frc2::JoystickButton(&m_operatorController, kClimberRevertButton)
        .WhenPressed(&m_ClimberRevert)
        .WhenReleased(&m_ClimberReset);
    // Final do Climber

    /*Atribuição de botoes para Intake
      um botão para o acionamento normal
    */
    frc2::JoystickButton(&m_operatorController, kIntakeButton)
        .WhenPressed(&m_IntakeSet)
        .WhenReleased(&m_IntakeReset);
    // final do Intake

    /*Atribuição de botoes para Conveyor
      um botão para o acionamento normal
      Um botão para reverter
    */
    frc2::JoystickButton(&m_operatorController, kConveyorButton)
        .WhenPressed(&m_ConveyorSet)
        .WhenReleased(&m_ConveyorReset);
    frc2::JoystickButton(&m_operatorController, kConveyorRevetButton)
        .WhenPressed(&m_ConveyorRevert)
        .WhenReleased(&m_ConveyorReset);
    // Final do Conveyor

    /*Atribuição de botoes para Shooter
      um botão para o acionar sistema de mira e disparo
      Um botão para atirar
    */
    frc2::JoystickButton(&m_operatorController, kTriggerButton)
        .WhenPressed(&m_TriggerSet)
        .WhenReleased(&m_TriggerReset);
    frc2::JoystickButton(&m_operatorController, kShooterAimButton)
        .WhenActive(&m_AimTarget);
}
frc2::Command *RobotContainer::AimTarget()
{
    float Kp = -0.1f;
    float min_command = 0.05f;

    m_shooter.SetShooter(1);
    float tx = targetOffsetAngle_Horizontal;
    float heading_error = -tx;
    float steering_adjust = 0.0f;
    if (tx > 1.0)
    {
        steering_adjust = Kp * heading_error - min_command;
    }
    else if (tx < 1.0)
    {
        steering_adjust = Kp * heading_error + min_command;
    }

    return nullptr;
}
