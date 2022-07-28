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

#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/span.h"
#include <frc/SerialPort.h>

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
            m_drive.ArcadeDrive(-m_driverController.GetLeftY(),
                                m_driverController.GetRightX() * 0.8);
        },
        {&m_drive}));
    // frc::CameraServer::StartAutomaticCapture();
    //  Add commands to the autonomous command chooser
}

void RobotContainer::ConfigureButtonBindings()
{

    /*
    //Os comandos do Joystick foram desabilitados para evitar acionamentos por acidentes
    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kRightBumper)
        .WhenReleased(&m_driveFullSpeed)
        .WhenPressed(&m_driveHalfSpeed);
    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kA)
        .WhenPressed(&m_ShooterOn)
        .WhenReleased(&m_ShooterOff);
        */

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
void RobotContainer::AimTarget()
{
    float Kp = -0.1f;
    float min_command = 0.05f;

    std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    double targetOffsetAngle_Horizontal = table->GetNumber("tx", 0.0);
    double targetOffsetAngle_Vertical = table->GetNumber("ty", 0.0);
    double targetArea = table->GetNumber("ta", 0.0);
    double targetSkew = table->GetNumber("ts", 0.0);

    float tx = table->GetNumber("tx", 0.0);
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
    // left_command += steering_adjust;
    // right_command -= steering_adjust;
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
        // aguardo 2 segundos para pressurizar e baixar o intake
        frc2::InstantCommand([this]
                             {
            static units::second_t timeout = timer.GetFPGATimestamp() + 1_s;
            while (timer.GetFPGATimestamp() < timeout)
                m_drive.TankDriveVolts(0_V, 0_V); },
                             {}),
        // baixo o intake e ligo o conveyor
        m_IntakeSet,
        // aguardo 300 ms para os comandos mecanicos funcionarem
        frc2::InstantCommand([this]
                             {
            static units::second_t timeout = timer.GetFPGATimestamp() + 250_ms;
            while (timer.GetFPGATimestamp() < timeout)
                m_drive.TankDriveVolts(0_V, 0_V); },
                             {}),
        // faço a jogada de balanco para ajustar a primeira bola no conveyor
        frc2::InstantCommand([this]
                             {
            uint8_t throwRepeater = 0;
            m_drive.ResetEncoders();
            for (throwRepeater = 0; throwRepeater < 3; throwRepeater++)
            {
                while (m_drive.GetAverageEncoderDistance()>-0.1)
                    m_drive.TankDriveVolts(-3_V, -3_V);
                while (m_drive.GetAverageEncoderDistance()<0)
                    m_drive.TankDriveVolts(3_V, 3_V);
                
            } },
                             {}),
        // paro de atuar o conveyor para não afogar com as bolas
        // me desloco 1m para coletar a segunda bola e ficar em posição de tiro
        frc2::InstantCommand([this]
                             {
                                 m_drive.ResetEncoders();
                                 while (m_drive.GetAverageEncoderDistance() < 1)
                                 {
                                     m_drive.TankDriveVolts(4_V, 4_V);
                                 } },
                             {}),
        // aciono o rotor do shooter e aguardo 250ms para que esteja na aceleração maxima
        frc2::InstantCommand([this]
                             {
                                 static units::second_t timeout = timer.GetFPGATimestamp() + 250_ms;

                                 while (timer.GetFPGATimestamp() < timeout)
                                     m_drive.TankDriveVolts(0_V, 0_V); },
                             {}),
        // atiro as bolas e faco o balanco do robo para que a segunda bola seja atirada tambem
        frc2::InstantCommand([this]
                             {
           uint8_t throwRepeater = 0;
            m_drive.ResetEncoders();
            for (throwRepeater = 0; throwRepeater < 3; throwRepeater++)
            {
                while (m_drive.GetAverageEncoderDistance()<0.1)
                    m_drive.TankDriveVolts(3_V, 3_V);
                while (m_drive.GetAverageEncoderDistance()>0)
                    m_drive.TankDriveVolts(-3_V, -3_V);
                
                
            } },
                             {}),
        frc2::InstantCommand([this]
                             {
                                 m_drive.ResetEncoders();
                                 while (m_drive.GetAverageEncoderDistance() > -0.3)
                                     m_drive.TankDriveVolts(-4_V, -4_V); },
                             {}),
        frc2::InstantCommand([this]
                             {
            uint8_t throwRepeater = 0;
            m_drive.ResetEncoders();
            for (throwRepeater = 0; throwRepeater < 3; throwRepeater++)
            {
                while (m_drive.GetAverageEncoderDistance()>-0.1)
                    m_drive.TankDriveVolts(-3_V, -3_V);
                while (m_drive.GetAverageEncoderDistance()<0)
                    m_drive.TankDriveVolts(3_V, 3_V);
                
            } },
                             {}));
}