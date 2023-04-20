// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <subsystems/Arm.h>

RobotContainer::RobotContainer() {
  // Init subsystems
  Arm::GetInstance();

  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  _controller.B().WhileTrue(Arm::GetInstance().DriveToCoords(Arm::SUBSTATION));
  _controller.Y().OnTrue(Arm::GetInstance().DriveToCoords(Arm::HI));



}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
