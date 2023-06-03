#include "subsystems/Intake.h"
#include <frc/smartdashboard/SmartDashboard.h>

Intake::Intake() {
  frc::SmartDashboard::PutData("intake/motor", (wpi::Sendable*)&_motor);
  frc::SmartDashboard::PutData("intake/mech", &_intakeMech);
};

void Intake::Periodic() {
  _intakeLigament->SetAngle(_motor.GetPosition());
}

void Intake::SimulationPeriodic() {
  _sim.SetInputVoltage(_motor.GetSimVoltage());
  _sim.Update(20_ms);
  _motor.UpdateSimEncoder(_sim.GetAngularPosition(), _sim.GetAngularVelocity());
}

frc2::CommandPtr Intake::DriveAt(double dutyCycle) {
  return StartEnd([this, dutyCycle] { _motor.Set(dutyCycle); },
                  [this, dutyCycle] { _motor.Set(0); });
}

frc2::CommandPtr Intake::Suck() {
  return StartEnd([this] { _motor.Set(1); }, [this] { _motor.Set(0.1); }).Until([this] {
    return GamePieceDetected();
  });
}

frc2::CommandPtr Intake::Spit() {
  return DriveAt(-1);
}

bool Intake::GamePieceDetected() {
  return _motor.GetVelocity() == 0_tps && _motor.Get() != 0;
}