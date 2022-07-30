// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClimberSubsystem.h"


using namespace ClimberConstants;
ClimberSubsystem::ClimberSubsystem()
    : m_climber{kClimberMotorPort}
{
  
}
void ClimberSubsystem::Periodic()
{
}
void ClimberSubsystem::SetClimber(double speed){
  m_climber.Set(speed);
}
