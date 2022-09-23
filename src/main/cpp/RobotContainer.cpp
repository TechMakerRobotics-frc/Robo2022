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
#include <frc2/command/WaitUntilCommand.h>
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
                                m_driverController.GetRightX() * 0.75);
        },
        {&m_drive}));

    camera = frc::CameraServer::StartAutomaticCapture();
    camera.SetResolution(640, 480);
    server = frc::CameraServer::GetServer();
    server.SetSource(camera);
}
frc2::Command *RobotContainer::GetAutonomousCommand()
{

    return new frc2::SequentialCommandGroup(
        frc2::InstantCommand([this]
        {
            m_shooter.ResetEncoders();
            m_shooter.SetAim(-3_V);

        },{}),
        frc2::WaitUntilCommand(std::function<bool()> { [this](){ return m_shooter.getAimEncoder()>=400; } }),
        frc2::WaitCommand(500_ms),
        frc2::InstantCommand([this]
        {
            m_shooter.SetAim(3_V);

        },{}),
        frc2::WaitUntilCommand(std::function<bool()> { [this](){ return m_shooter.getAimEncoder()<=0; } }),
        frc2::WaitCommand(500_ms)

        );
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
}

void RobotContainer::ConfigureButtonBindings()
{

    // Os comandos do Joystick foram desabilitados para evitar acionamentos por acidentes
    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kA)
        .WhenPressed(&m_ConveyorSet)
        .WhenReleased(&m_ConveyorReset);
    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kB)
        .WhenPressed(&m_ConveyorRevert)
        .WhenReleased(&m_ConveyorReset);

    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kY)
        .WhenPressed(&m_TriggerSet)
        .WhenReleased(&m_TriggerReset);
    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kLeftBumper)
        .WhenPressed(&m_ShooterOn)
        .WhenReleased(&m_ShooterOff);
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
        .WhenPressed(&m_ShooterOn)
        .WhenReleased(&m_ShooterOff);
    frc2::JoystickButton(&m_operatorController, kAimForwardButton)
        .WhenPressed(&m_AimForward)
        .WhenReleased(&m_AimReset);
    frc2::JoystickButton(&m_operatorController, kAimRevertButton)
        .WhenPressed(&m_AimRewind)
        .WhenReleased(&m_AimReset);
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
