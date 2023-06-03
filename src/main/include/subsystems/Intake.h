#pragma once

#include <frc2/command/SubsystemBase.h>
#include "utilities/ICSparkMax.h"
#include <frc2/command/CommandPtr.h>
#include "IDs.h"
#include <frc/simulation/DCMotorSim.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>

class Intake : public frc2::SubsystemBase {
 public:
  static Intake &GetInstance() {static Intake inst; return inst;}
  void Periodic() override;
  void SimulationPeriodic() override;
  bool GamePieceDetected();
  frc2::CommandPtr DriveAt(double dutyCycle);
  frc2::CommandPtr Suck();
  frc2::CommandPtr Spit();

 private:
  Intake();
  ICSparkMax<> _motor{canid::intake};
  frc::sim::DCMotorSim _sim{frc::DCMotor::NEO(), 10, 0.0001_kg_sq_m};
  frc::Mechanism2d _intakeMech{4, 4};  // canvas width and height
  frc::MechanismRoot2d* _root = _intakeMech.GetRoot("intakeRoot", 2, 2);  // root x and y
  frc::MechanismLigament2d* _intakeLigament = _root->Append<frc::MechanismLigament2d>(
      "intakeLigament", 0.5, 0_deg, 10);
};
