// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

class RobotContainer {
 public:
  RobotContainer();

  enum GamePieceMode {CONE, CUBE};
  inline static GamePieceMode gamePieceMode = CONE;

  frc2::CommandPtr GetAutonomousCommand();

 private:
  void ConfigureBindings();
  frc2::CommandXboxController _driverController{0};
  frc2::CommandXboxController _secondController{1};

};
