#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include "subsystems/Arm.h"
#include "subsystems/DriveBase.h"
#include "subsystems/Intake.h"
#include "utilities/POVHelper.h"
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>

RobotContainer::RobotContainer() {
  // Init subsystems
  Arm::GetInstance();
  DriveBase::GetInstance();
  Intake::GetInstance();

  // Setup named commands for pathplanner autos
  pathplanner::NamedCommands::registerCommand("armToHigh", Arm::GetInstance().ToHigh());
  pathplanner::NamedCommands::registerCommand("armToDefault", Arm::GetInstance().ToDefault());
  pathplanner::NamedCommands::registerCommand("armToScore",
                                              Arm::GetInstance().ScoreAtCurrentHeight());
  pathplanner::NamedCommands::registerCommand("spit", Intake::GetInstance().Spit());

  // Config command bindings
  ConfigureBindings();
  DriveBase::GetInstance().SetDefaultCommand(DriveBase::GetInstance().XboxDrive(_driverController));
}

void RobotContainer::ConfigureBindings() {
  // Main Controller
  _driverController.Start().OnTrue(DriveBase::GetInstance().ResetGyroHeading());
  _driverController.RightBumper().WhileTrue(Intake::GetInstance().Spit());
  _driverController.RightTrigger().WhileTrue(Intake::GetInstance().Suck());
  _driverController.LeftBumper().WhileTrue(
      Arm::GetInstance()
          .ScoreAtCurrentHeight()
          .AndThen(Intake::GetInstance().Spit())
          .AlongWith(DriveBase::GetInstance()
                         .DriveCmd(
                             [] {
                               return frc::ChassisSpeeds{0_mps, -1_mps, 0_deg_per_s};
                             },
                             false)
                         .WithTimeout(0.7_s)));

  // Second controller
  // POVHelper::Up(&_secondController).WhileTrue(Arm::GetInstance().ManualArmMove(0, 20));
  // POVHelper::Down(&_secondController).WhileTrue(Arm::GetInstance().ManualArmMove(0, -20));
  // POVHelper::Right(&_secondController).WhileTrue(Arm::GetInstance().ManualArmMove(20, 0));
  // POVHelper::Left(&_secondController).WhileTrue(cArm::GetInstance().ManualArmMove(-20, 0));
  _secondController.Y().OnTrue(Arm::GetInstance().ToHigh());
  _secondController.B().OnTrue(Arm::GetInstance().ToMid());
  _secondController.A().OnTrue(Arm::GetInstance().ToLowHybrid());
  _secondController.X().OnTrue(Arm::GetInstance().ToGroundPickup());
  _secondController.LeftTrigger().OnTrue(Arm::GetInstance().ToSubstation());
  _secondController.RightTrigger().OnTrue(Arm::GetInstance().ToDefault());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return pathplanner::AutoBuilder::buildAuto("myAuto");
}
