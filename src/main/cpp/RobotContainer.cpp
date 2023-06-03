// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include "subsystems/Arm.h"
#include "subsystems/DriveBase.h"
#include "subsystems/Intake.h"
#include "utilities/POVHelper.h"

RobotContainer::RobotContainer() {
  // Init subsystems
  Arm::GetInstance();
  DriveBase::GetInstance();
  Intake::GetInstance();

  ConfigureBindings();
  DriveBase::GetInstance().SetDefaultCommand(DriveBase::GetInstance().XboxDrive(_driverController));
}

void RobotContainer::ConfigureBindings() {
   //Main Controller
  _driverController.Start().OnTrue(DriveBase::GetInstance().ResetGyroHeading());
  _driverController.RightBumper().WhileTrue(Intake::GetInstance().Spit());
  _driverController.RightTrigger().WhileTrue(Intake::GetInstance().Suck());

  // //Second controller
  // POVHelper::Up(&_secondController).WhileTrue(Arm::GetInstance().ManualArmMove(0, 20));
  // POVHelper::Down(&_secondController).WhileTrue(Arm::GetInstance().ManualArmMove(0, -20));
  // POVHelper::Right(&_secondController).WhileTrue(Arm::GetInstance().ManualArmMove(20, 0)); //forward
  // POVHelper::Left(&_secondController).WhileTrue(cArm::GetInstance().ManualArmMove(-20, 0)); //backward
  _secondController.Y().OnTrue(Arm::GetInstance().ToHighCone());
  _secondController.B().OnTrue(Arm::GetInstance().ToMidCone());
  _secondController.A().OnTrue(Arm::GetInstance().ToLowHybrid());
  _secondController.X().OnTrue(Arm::GetInstance().ToGroundPickup());
  _secondController.LeftTrigger().OnTrue(Arm::GetInstance().ToSubstation());
  _secondController.RightTrigger().OnTrue(Arm::GetInstance().ToDefault());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
