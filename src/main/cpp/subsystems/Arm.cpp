#include "subsystems/Arm.h"
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>

Arm::Arm() {
  _bottomMotor.SetConversionFactor(1 / BOTTOM_GEAR_RATIO);
  _bottomMotor.SetPIDFF(BOTTOM_P, BOTTOM_I, BOTTOM_D, BOTTOM_F);
  _bottomMotor.ConfigSmartMotion(BOTTOM_MAX_VEL, BOTTOM_MAX_ACCEL, BOTTOM_TOLERANCE);
  _bottomFollower.Follow(_bottomMotor);
  frc::SmartDashboard::PutData("Arm/bottom motor", (wpi::Sendable*)&_bottomMotor);

  _topMotor.SetConversionFactor(1 / TOP_GEAR_RATIO);
  _topMotor.SetPIDFF(TOP_P, TOP_I, TOP_D, TOP_F);
  _topMotor.ConfigSmartMotion(TOP_MAX_VEL, TOP_MAX_ACCEL, TOP_TOLERANCE);
  _topMotor.SetInverted(true);
  _topFollower.Follow(_topMotor);
  frc::SmartDashboard::PutData("Arm/top motor", (wpi::Sendable*)&_topMotor);

  frc::SmartDashboard::PutData("Arm/Mechanism Display", &_doubleJointedArmMech);
};

void Arm::Periodic() {
  frc::SmartDashboard::PutNumber("Arm/ground to top arm turns", GetGroundToTopArmAngle().value());

  auto EEPos = GetEndEffectorPosition();
  frc::SmartDashboard::PutNumber("Arm/Current X", EEPos.X().value());
  frc::SmartDashboard::PutNumber("Arm/Current Y", EEPos.Y().value());

  // Update mech2d display
  _bottomArmLigament->SetAngle(_bottomMotor.GetPosition());
  _topArmLigament->SetAngle(_topMotor.GetPosition());
}

void Arm::SimulationPeriodic() {
  _bottomArmSim.SetInputVoltage(_bottomMotor.GetSimVoltage());
  _bottomArmSim.Update(20_ms);

  _topArmSim.SetInputVoltage(_topMotor.GetSimVoltage());
  _topArmSim.Update(20_ms);

  auto bottomAngle = _bottomArmSim.GetAngle();
  auto bottomVel = _bottomArmSim.GetVelocity();
  _bottomMotor.UpdateSimEncoder(bottomAngle, bottomVel);

  auto topAngle = _topArmSim.GetAngle();
  auto topVel = _topArmSim.GetVelocity();
  _topMotor.UpdateSimEncoder(topAngle, topVel);
}

frc2::CommandPtr Arm::DriveToAngles(units::radian_t bottomAngle, units::radian_t topAngle) {
  return RunOnce([this, bottomAngle, topAngle] {
           _topMotor.SetSmartMotionTarget(topAngle);
           _bottomMotor.SetSmartMotionTarget(bottomAngle);
           _topArmTargetLigament->SetAngle(topAngle);
           _bottomArmTargetLigament->SetAngle(bottomAngle);
         })
      .AndThen(frc2::cmd::WaitUntil([this] { return OnTarget(); }));
}

frc2::CommandPtr Arm::DriveToCoords(units::meter_t x, units::meter_t y) {
  auto angles = CoordsToAngles(x, y);
  if (angles.has_value()) {
    return DriveToAngles(angles.value().bottomAngle, angles.value().topAngle)
        .AlongWith(frc2::cmd::RunOnce([this, x, y] {
          _endEffectorTarget = {x, y};
        }));
  }
  return frc2::cmd::None();
}

frc2::CommandPtr Arm::DriveToCoords(frc::Translation2d coord) {
  return DriveToCoords(coord.X(), coord.Y());
}

std::optional<Arm::Angles> Arm::CoordsToAngles(units::meter_t x, units::meter_t y) {
  auto armTopAngleFracbottom = units::math::pow<2>(x) + units::math::pow<2>(y) -
                               units::math::pow<2>(BOTTOM_ARM_LENGTH) -
                               units::math::pow<2>(TOP_ARM_LENGTH);
  auto armTopAngleFractop = 2 * BOTTOM_ARM_LENGTH * TOP_ARM_LENGTH;
  auto armTopAngle = -(units::math::acos(armTopAngleFracbottom / armTopAngleFractop));

  auto armBottomAngleFracbottom = TOP_ARM_LENGTH * units::math::sin(armTopAngle);
  auto armBottomAngleFractop = BOTTOM_ARM_LENGTH + TOP_ARM_LENGTH * units::math::cos(armTopAngle);
  auto statement1 = units::math::atan(y / x);
  auto statement2 = units::math::atan(armBottomAngleFracbottom / armBottomAngleFractop);
  auto armBottomAngle = statement1 - statement2;

  // Some X Y targets cause a bad IK output since the arm can't reach there, catch them here
  if (std::isnan(armBottomAngle.value()) || std::isnan(armTopAngle.value())) {
    return {};
  }
  return Angles{armBottomAngle, armTopAngle};
}

units::radian_t Arm::GetGroundToTopArmAngle() {
  return _bottomMotor.GetPosition() + _topMotor.GetPosition();
}

frc::Translation2d Arm::GetEndEffectorPosition() {
  frc::Rotation2d groundToTop{units::radian_t{GetGroundToTopArmAngle()}};
  frc::Rotation2d groundToBottom{units::radian_t{_bottomMotor.GetPosition()}};
  frc::Translation2d topPos{TOP_ARM_LENGTH, groundToTop};
  frc::Translation2d bottomPos{BOTTOM_ARM_LENGTH, groundToBottom};
  return topPos + bottomPos;
}

bool Arm::OnTarget() {
  return _topMotor.OnPosTarget(1_deg) && _bottomMotor.OnPosTarget(1_deg);
}

frc2::CommandPtr Arm::ToPreScore() {
  return DriveToCoords(50_cm, 110_cm).Until([&] { return GetEndEffectorPosition().Y() > 70_cm; });
}

frc2::CommandPtr Arm::ScoreAtCurrentHeight() {
  return frc2::cmd::Either(ToScoredHigh(), ToScoredMid(), [this] {
    return GetEndEffectorPosition().Y() > 100_cm;
  });
}