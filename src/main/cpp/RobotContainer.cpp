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
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include "cameraserver/CameraServer.h"
#include "Constants.h"

RobotContainer::RobotContainer()
{
    // Initialize all of your commands and subsystems here

    // Configure the button bindings
    ConfigureButtonBindings();
    // The chooser for the autonomous routines
    // Set up default drive command
    m_drive.SetDefaultCommand(frc2::RunCommand(
        [this] {
            m_drive.ArcadeDrive(-m_driverController.GetLeftY(),
                                m_driverController.GetRightX() * 0.8);
        },
        {&m_drive}));
    //frc::CameraServer::StartAutomaticCapture();
    // Add commands to the autonomous command chooser
}

void RobotContainer::ConfigureButtonBindings()
{
    //Comandos para o primeiro controle
    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kX)
        .WhenPressed(&m_shooterToggle);

    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kRightBumper)
        .WhenReleased(&m_driveFullSpeed)
        .WhenPressed(&m_driveHalfSpeed);
    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kA)
        .WhenPressed(&m_ShooterOn)
        .WhenReleased(&m_ShooterOff);

    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kY)
        .WhenPressed(&m_ConveyorSet)
        .WhenReleased(&m_ConveyorReset);
    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kB)
        .WhenPressed(&m_IntakeToggle);
    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kStart)
        .WhenPressed(&m_ResetOdometry);
    frc2::POVButton(&m_driverController, 90, 0)
        .WhenPressed(&m_ConveyorRewind)
        .WhenReleased(&m_StopRewind);
    //Comandos do segundo controle. Somente dispositivos, não pode controlar a navegaçao
    /*frc2::JoystickButton(&m_driverController2, (int)frc::XboxController::Button::kX)
        .WhenPressed(&m_shooterToggle);
    frc2::JoystickButton(&m_driverController2, (int)frc::XboxController::Button::kBumperRight)
        .WhenReleased(&m_driveFullSpeed)
        .WhenPressed(&m_driveHalfSpeed);
    frc2::JoystickButton(&m_driverController2, (int)frc::XboxController::Button::kA)
        .WhenPressed(&m_ShooterOn)
        .WhenReleased(&m_ShooterOff);

    frc2::JoystickButton(&m_driverController2, (int)frc::XboxController::Button::kY)
        .WhenPressed(&m_ConveyorSet)
        .WhenReleased(&m_ConveyorReset);
    frc2::JoystickButton(&m_driverController2, (int)frc::XboxController::Button::kB)
        .WhenPressed(&m_IntakeToggle);
    frc2::JoystickButton(&m_driverController2, (int)frc::XboxController::Button::kStart)
        .WhenPressed(&m_ResetOdometry);
    frc2::POVButton(&m_driverController2,90, 0)
        .WhenPressed(&m_ConveyorRewind)
        .WhenReleased(&m_StopRewind);*/
    /*
        Controle personalizado

        9  10  11  12
        5   6   7   8
        1   2   3   4
        
        apenas enquanto pressionadas as teclas

        9 - Desce intake e ativa
        11 - ativa motor intake
        5 - liga conveyor normal
        6 - reverte conveyour e admissão
        1 - liga rotor do shooter e admissão
        2 - liga admissão
        3 - atira
        4 - muda mira
    */
    frc2::JoystickButton(&m_driverController3, 3)
        .WhenPressed(&m_ShooterOn)
        .WhenReleased(&m_ShooterOff);
    frc2::JoystickButton(&m_driverController3, 2)
        .WhenPressed(&m_ConveyorSet)
        .WhenReleased(&m_ConveyorReset);
    frc2::JoystickButton(&m_driverController3, 1)
        .WhenPressed(&m_shooterRotorOn)
        .WhenReleased(&m_shooterRotorOff);
    //frc2::JoystickButton(&m_driverController3, 4)
    //    .WhenPressed(&m_TargetToggle);
    frc2::JoystickButton(&m_driverController3, 5)
        .WhenPressed(&m_ConveyorSet)
        .WhenReleased(&m_ConveyorReset);
    frc2::JoystickButton(&m_driverController3, 6)
        .WhenPressed(&m_ConveyorRewind)
        .WhenReleased(&m_StopRewind);
    frc2::JoystickButton(&m_driverController3, 9)
        .WhenPressed(&m_IntakeSet)
        .WhenReleased(&m_IntakeReset);
    frc2::JoystickButton(&m_driverController3, 10)
        .WhenPressed(&m_IntakeRewind)
        .WhenReleased(&m_IntakeStopRewind);
    frc2::JoystickButton(&m_driverController3, 11)
        .WhenPressed(&m_IntakeSetMotor)
        .WhenReleased(&m_IntakeResetMotor);
}

frc2::Command *RobotContainer::GetCaminhoA()
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

    // An example trajectory to follow.  All units in meters.
    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        // Start at the origin facing the +X direction
        frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
        // Pass through these two interior waypoints, making an 's' curve path
        {
            frc::Translation2d(7.25_m, 0_m),
            frc::Translation2d(7.25_m, 3.25_m)},
        // End 3 meters straight ahead of where we started, facing forward
        frc::Pose2d(0.5_m, 3_m, frc::Rotation2d(180_deg)),
        // Pass the config
        config);

    frc2::RamseteCommand ramseteCommand(
        exampleTrajectory, [this]() { return m_drive.GetPose(); },
        frc::RamseteController(AutoConstants::kRamseteB,
                               AutoConstants::kRamseteZeta),
        frc::SimpleMotorFeedforward<units::meters>(
            DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
        DriveConstants::kDriveKinematics,
        [this] { return m_drive.GetWheelSpeeds(); },
        frc2::PIDController(DriveConstants::kPDriveVel, DriveConstants::kIDriveVel, DriveConstants::kDDriveVel),
        frc2::PIDController(DriveConstants::kPDriveVel, DriveConstants::kIDriveVel, DriveConstants::kDDriveVel),
        [this](auto left, auto right) { m_drive.TankDriveVolts(left, right); },
        {&m_drive});

    // Reset odometry to the starting pose of the trajectory.
    m_drive.ResetOdometry(exampleTrajectory.InitialPose());
    // no auto
    return new frc2::SequentialCommandGroup(
        std::move(ramseteCommand),
        frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}));
}
frc2::Command *RobotContainer::GetCaminhoB()
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

    // An example trajectory to follow.  All units in meters.
    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        // Start at the origin facing the +X direction
        frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
        // Pass through these two interior waypoints, making an 's' curve path
        {
            frc::Translation2d(5.7_m, 0_m),
            frc::Translation2d(5.7_m, 3_m),
            frc::Translation2d(1_m, 3_m),
            frc::Translation2d(1_m, 1.5_m)},
        // End 3 meters straight ahead of where we started, facing forward
        frc::Pose2d(7.5_m, 1.5_m, frc::Rotation2d(0_deg)),
        // Pass the config
        config);
    frc2::RamseteCommand ramseteCommand(
        exampleTrajectory, [this]() { return m_drive.GetPose(); },
        frc::RamseteController(AutoConstants::kRamseteB,
                               AutoConstants::kRamseteZeta),
        frc::SimpleMotorFeedforward<units::meters>(
            DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
        DriveConstants::kDriveKinematics,
        [this] { return m_drive.GetWheelSpeeds(); },
        frc2::PIDController(DriveConstants::kPDriveVel, DriveConstants::kIDriveVel, DriveConstants::kDDriveVel),
        frc2::PIDController(DriveConstants::kPDriveVel, DriveConstants::kIDriveVel, DriveConstants::kDDriveVel),
        [this](auto left, auto right) { m_drive.TankDriveVolts(left, right); },
        {&m_drive});

    // Reset odometry to the starting pose of the trajectory.
    m_drive.ResetOdometry(exampleTrajectory.InitialPose());
    // no auto
    return new frc2::SequentialCommandGroup(
        std::move(ramseteCommand),
        frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}));
}
frc2::Command *RobotContainer::GetCaminhoC()
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

    // An example trajectory to follow.  All units in meters.
    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        // Start at the origin facing the +X direction
        frc::Pose2d(0_m, 0_m, frc::Rotation2d(90_deg)),
        // Pass through these two interior waypoints, making an 's' curve path
        {
            frc::Translation2d(0.5_m, 1.5_m),
            frc::Translation2d(2_m, 3.5_m),
            frc::Translation2d(3.5_m, 3.5_m),
            frc::Translation2d(3.5_m, 1.5_m),
            frc::Translation2d(6_m, 0_m),
            frc::Translation2d(7.5_m, 0_m)},
        // End 3 meters straight ahead of where we started, facing forward
        frc::Pose2d(8_m, 3_m, frc::Rotation2d(90_deg)),
        // Pass the config
        config);

    frc2::RamseteCommand ramseteCommand(
        exampleTrajectory, [this]() { return m_drive.GetPose(); },
        frc::RamseteController(AutoConstants::kRamseteB,
                               AutoConstants::kRamseteZeta),
        frc::SimpleMotorFeedforward<units::meters>(
            DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
        DriveConstants::kDriveKinematics,
        [this] { return m_drive.GetWheelSpeeds(); },
        frc2::PIDController(DriveConstants::kPDriveVel, DriveConstants::kIDriveVel, DriveConstants::kDDriveVel),
        frc2::PIDController(DriveConstants::kPDriveVel, DriveConstants::kIDriveVel, DriveConstants::kDDriveVel),
        [this](auto left, auto right) { m_drive.TankDriveVolts(left, right); },
        {&m_drive});

    // Reset odometry to the starting pose of the trajectory.
    m_drive.ResetOdometry(exampleTrajectory.InitialPose());
    // no auto
    return new frc2::SequentialCommandGroup(
        std::move(ramseteCommand),
        frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}));
}
frc2::Command *RobotContainer::GetCaminhoD()
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
    //config.AddConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        // Start at the origin facing the +X direction
        frc::Pose2d(0_m, 2.28_m, frc::Rotation2d(0_deg)),
        // Pass through these two interior waypoints, making an 's' curve path
        {
            frc::Translation2d(3_m, 2.28_m),
            frc::Translation2d(4.4_m, 0.762_m),
            frc::Translation2d(4.4_m, 4_m)},
        // End 3 meters straight ahead of where we started, facing forward
        frc::Pose2d(8_m, 3_m, frc::Rotation2d(0_deg)),
        // Pass the config
        config);
    frc2::RamseteCommand ramseteCommand(
        exampleTrajectory, [this]() { return m_drive.GetPose(); },
        frc::RamseteController(AutoConstants::kRamseteB,
                               AutoConstants::kRamseteZeta),
        frc::SimpleMotorFeedforward<units::meters>(
            DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
        DriveConstants::kDriveKinematics,
        [this] { return m_drive.GetWheelSpeeds(); },
        frc2::PIDController(DriveConstants::kPDriveVel, DriveConstants::kIDriveVel, DriveConstants::kDDriveVel),
        frc2::PIDController(DriveConstants::kPDriveVel, DriveConstants::kIDriveVel, DriveConstants::kDDriveVel),
        [this](auto left, auto right) { m_drive.TankDriveVolts(left, right); },
        {&m_drive});

    // Reset odometry to the starting pose of the trajectory.
    m_drive.ResetOdometry(exampleTrajectory.InitialPose());
    m_shooter.ToggleIntake();
    m_shooter.SetConveyor(-1);
    // no auto
    return new frc2::SequentialCommandGroup(
        std::move(ramseteCommand),
        frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}));
}
frc2::Command *RobotContainer::GetCaminhoE()
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

    // An example trajectory to follow.  All units in meters.
    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        // Start at the origin facing the +X direction
        frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
        // Pass through these two interior waypoints, making an 's' curve path
        {
            frc::Translation2d(1.75_m, 0_m),
            frc::Translation2d(3.10_m, -1.54_m),
            frc::Translation2d(4.75_m, 0_m)},
        // End 3 meters straight ahead of where we started, facing forward
        frc::Pose2d(8_m, 0_m, frc::Rotation2d(0_deg)),
        // Pass the config
        config);
    frc2::RamseteCommand ramseteCommand(
        exampleTrajectory, [this]() { return m_drive.GetPose(); },
        frc::RamseteController(AutoConstants::kRamseteB,
                               AutoConstants::kRamseteZeta),
        frc::SimpleMotorFeedforward<units::meters>(
            DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
        DriveConstants::kDriveKinematics,
        [this] { return m_drive.GetWheelSpeeds(); },
        frc2::PIDController(DriveConstants::kPDriveVel, DriveConstants::kIDriveVel, DriveConstants::kDDriveVel),
        frc2::PIDController(DriveConstants::kPDriveVel, DriveConstants::kIDriveVel, DriveConstants::kDDriveVel),
        [this](auto left, auto right) { m_drive.TankDriveVolts(left, right); },
        {&m_drive});

    // Reset odometry to the starting pose of the trajectory.
    m_drive.ResetOdometry(exampleTrajectory.InitialPose());
    m_shooter.ToggleIntake();
    m_shooter.SetConveyor(-1);
    // no auto
    return new frc2::SequentialCommandGroup(
        std::move(ramseteCommand),
        frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}));
}
frc2::Command *RobotContainer::Home()
{
    // Create a voltage constraint to ensure we don't accelerate too fast
    frc::DifferentialDriveVoltageConstraint autoVoltageConstraint(
        frc::SimpleMotorFeedforward<units::meters>(
            DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
        DriveConstants::kDriveKinematics, 10_V);

    // Set up config for trajectory
    frc::TrajectoryConfig config(2_mps, 1_mps_sq);
    // Add kinematics to ensure max speed is actually obeyed
    config.SetKinematics(DriveConstants::kDriveKinematics);
    // Apply the voltage constraint
    config.AddConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        // Start at the origin facing the +X direction
        m_drive.GetPose(),
        // Pass through these two interior waypoints, making an 's' curve path
        {
            frc::Translation2d(1_m, 1_m)},
        // End 3 meters straight ahead of where we started, facing forward
        frc::Pose2d(0.5_m, 0_m, frc::Rotation2d(0_deg)),
        // Pass the config
        config);

    frc2::RamseteCommand ramseteCommand(
        exampleTrajectory, [this]() { return m_drive.GetPose(); },
        frc::RamseteController(AutoConstants::kRamseteB,
                               AutoConstants::kRamseteZeta),
        frc::SimpleMotorFeedforward<units::meters>(
            DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
        DriveConstants::kDriveKinematics,
        [this] { return m_drive.GetWheelSpeeds(); },
        frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
        frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
        [this](auto left, auto right) { m_drive.TankDriveVolts(left, right); },
        {&m_drive});

    // no auto
    return new frc2::SequentialCommandGroup(
        std::move(ramseteCommand),
        frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}));
}
frc2::Command *RobotContainer::FinalAutonomousCommand()
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


    auto Trajectory2 = frc::TrajectoryGenerator::GenerateTrajectory(
        frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
        {},
        frc::Pose2d(0.5_m, 0_m, frc::Rotation2d(0_deg)),
        config);
   
    frc2::RamseteCommand ramseteCommand2(
        Trajectory2, [this]() { return m_drive.GetPose(); },
        frc::RamseteController(AutoConstants::kRamseteB,
                               AutoConstants::kRamseteZeta),
        frc::SimpleMotorFeedforward<units::meters>(
            DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
        DriveConstants::kDriveKinematics,
        [this] { return m_drive.GetWheelSpeeds(); },
        frc2::PIDController(DriveConstants::kPDriveVel, DriveConstants::kIDriveVel, DriveConstants::kDDriveVel),
        frc2::PIDController(DriveConstants::kPDriveVel, DriveConstants::kIDriveVel, DriveConstants::kDDriveVel),
        [this](auto left, auto right) { m_drive.TankDriveVolts(left, right); },
        {&m_drive});
 
    return new frc2::SequentialCommandGroup(
        //ligo o compressor
        frc2::InstantCommand([this] { m_shooter.SetCompressor(1); }, {}),
        //aguardo 2 segundos para pressurizar e baixar o intake
        frc2::InstantCommand([this] {
            static units::second_t timeout = timer.GetFPGATimestamp() + 1_s;
            while (timer.GetFPGATimestamp() < timeout)
                m_drive.TankDriveVolts(0_V, 0_V);
        }, {}),
        //baixo o intake e ligo o conveyor
        m_IntakeSet, 
        m_ConveyorSet,
        //aguardo 300 ms para os comandos mecanicos funcionarem 
         frc2::InstantCommand([this] {
            static units::second_t timeout = timer.GetFPGATimestamp() + 250_ms;
            while (timer.GetFPGATimestamp() < timeout)
                m_drive.TankDriveVolts(0_V, 0_V);
        }, {}),
        //faço a jogada de balanco para ajustar a primeira bola no conveyor
         frc2::InstantCommand([this] {
            uint8_t throwRepeater = 0;
            m_drive.ResetEncoders();
            for (throwRepeater = 0; throwRepeater < 3; throwRepeater++)
            {
                while (m_drive.GetAverageEncoderDistance()>-0.1)
                    m_drive.TankDriveVolts(-3_V, -3_V);
                while (m_drive.GetAverageEncoderDistance()<0)
                    m_drive.TankDriveVolts(3_V, 3_V);
                
            }
        },{}),
        //paro de atuar o conveyor para não afogar com as bolas
        m_ConveyorReset, 
        //me desloco 1m para coletar a segunda bola e ficar em posição de tiro
        frc2::InstantCommand([this] {
            m_drive.ResetEncoders();
                while (m_drive.GetAverageEncoderDistance()<1)
                {
                    m_drive.TankDriveVolts(4_V, 4_V);
                }
           
             },{}),
        //aciono o rotor do shooter e aguardo 250ms para que esteja na aceleração maxima
        m_shooterRotorOn, 
        frc2::InstantCommand([this] {
            static units::second_t timeout = timer.GetFPGATimestamp() + 250_ms;
            
            while (timer.GetFPGATimestamp() < timeout)
                m_drive.TankDriveVolts(0_V, 0_V);
            
            
        },{}),
        //atiro as bolas e faco o balanco do robo para que a segunda bola seja atirada tambem
        m_ShooterOn, 
        frc2::InstantCommand([this] {
           uint8_t throwRepeater = 0;
            m_drive.ResetEncoders();
            for (throwRepeater = 0; throwRepeater < 3; throwRepeater++)
            {
                while (m_drive.GetAverageEncoderDistance()<0.1)
                    m_drive.TankDriveVolts(3_V, 3_V);
                while (m_drive.GetAverageEncoderDistance()>0)
                    m_drive.TankDriveVolts(-3_V, -3_V);
                
                
            }
        },{}),
        //desligo o shooter, vou para a ultima
        m_ShooterOff,
        //vou coletar a ultima bola, entaõ reseto a odometria, ligo o intake e o conveyor
        frc2::InstantCommand([this] { m_drive.ResetOdometry(frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg))); }, {}), 
        m_IntakeSet, 
        m_ConveyorSet, 
        std::move(ramseteCommand2),
        frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}), 
        //depois de coletar recolho  o intake e retorno para a posição de tiro
        frc2::InstantCommand([this] {
            m_drive.ResetEncoders();
            while (m_drive.GetAverageEncoderDistance()>-0.3)
                    m_drive.TankDriveVolts(-4_V, -4_V);
                
         
             },{}),
        //atiro a ultima bola, balancando o robo para que a mesma entre no conveyor
        m_ShooterOn,
        frc2::InstantCommand([this] {
            uint8_t throwRepeater = 0;
            m_drive.ResetEncoders();
            for (throwRepeater = 0; throwRepeater < 3; throwRepeater++)
            {
                while (m_drive.GetAverageEncoderDistance()>-0.1)
                    m_drive.TankDriveVolts(-3_V, -3_V);
                while (m_drive.GetAverageEncoderDistance()<0)
                    m_drive.TankDriveVolts(3_V, 3_V);
                
            }  },{}),
        //desativo os sistemas para economia de bateria
        m_ShooterOff, 
        m_shooterRotorOff);
}