// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include "subsystems/Arm.h"
#include "subsystems/DriveBase.h"
#include "utilities/POVHelper.h"

RobotContainer::RobotContainer() {
  // Init subsystems
  Arm::GetInstance();
  DriveBase::GetInstance();

  ConfigureBindings();
  DriveBase::GetInstance().SetDefaultCommand(DriveBase::GetInstance().XboxDrive(_driverController));
}

void RobotContainer::ConfigureBindings() {
   //Main Controller
  _driverController.Start().OnTrue(DriveBase::GetInstance().ResetGyroHeading());
  // _driverController.RightBumper().WhileTrue(cmd::RollerOuttake());
  // _driverController.RightTrigger().WhileTrue(cmd::RollerIntake());



}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
