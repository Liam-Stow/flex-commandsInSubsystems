// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveBase.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include <frc/RobotBase.h>
#include <units/time.h>
#include <frc/DriverStation.h>
#include <units/dimensionless.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/MathUtil.h>

DriveBase::DriveBase() {
  _gyro.Calibrate();
  Rcontroller.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
  SyncSensors();

  // Dashboard Displays
  frc::SmartDashboard::PutData("field", &_fieldDisplay);
}

void DriveBase::Periodic() {
  frc::SmartDashboard::PutNumber("drivebase/heading", GetHeading().Degrees().value());
  frc::SmartDashboard::PutNumber("drivebase/velocity", GetVelocity().value());
  frc::SmartDashboard::PutNumberArray("drivebase/true swerve states",
                                      std::array{
                                          _frontLeft.GetAngle().Degrees().value(),
                                          _frontLeft.GetSpeed().value(),
                                          _frontRight.GetAngle().Degrees().value(),
                                          _frontRight.GetSpeed().value(),
                                          _backLeft.GetAngle().Degrees().value(),
                                          _backLeft.GetSpeed().value(),
                                          _backRight.GetAngle().Degrees().value(),
                                          _backRight.GetSpeed().value(),
                                      });
  _frontLeft.SendSensorsToDash();
  _frontRight.SendSensorsToDash();
  _backLeft.SendSensorsToDash();
  _backRight.SendSensorsToDash();

  UpdateOdometry();
}

frc2::CommandPtr DriveBase::Drive(std::function<frc::ChassisSpeeds()> speedsSupplier,
                                  bool fieldRelative) {
  return Run([this, speedsSupplier, fieldRelative] {
    // Calculate desired states of all swerve modules
    auto states = _kinematics.ToSwerveModuleStates(
        fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(speedsSupplier(), GetHeading())
                      : frc::ChassisSpeeds{speedsSupplier()});

    // Apply speed limit to all modules
    _kinematics.DesaturateWheelSpeeds(&states, MAX_VELOCITY);
    auto [fl, fr, bl, br] = states;

    // Dashboard Displays
    frc::SmartDashboard::PutNumberArray("drivebase/desired swerve states",
                                        std::array{
                                            fl.angle.Degrees().value(),
                                            fl.speed.value(),
                                            fr.angle.Degrees().value(),
                                            fr.speed.value(),
                                            bl.angle.Degrees().value(),
                                            bl.speed.value(),
                                            br.angle.Degrees().value(),
                                            br.speed.value(),
                                        });

    _frontLeft.SetDesiredState(fl);
    _frontRight.SetDesiredState(fr);
    _backLeft.SetDesiredState(bl);
    _backRight.SetDesiredState(br);

    // Force gyro changes in sim
    if (frc::RobotBase::IsSimulation()) {
      units::radian_t radPer20ms = speedsSupplier().omega * 20_ms;
      units::degree_t newHeading = GetHeading().RotateBy(radPer20ms).Degrees();
      _gyro.SetAngleAdjustment(-newHeading.value());  // negative to switch to CW from CCW
    }
  });
}

frc2::CommandPtr DriveBase::XboxDrive(frc2::CommandXboxController& controller) {
  return Drive(
      [&controller] {
        static frc::SlewRateLimiter<units::scalar> xLimiter{3 / 1_s};
        static frc::SlewRateLimiter<units::scalar> yLimiter{3 / 1_s};
        static frc::SlewRateLimiter<units::scalar> rotLimiter{3 / 1_s};
        const double deadband = 0.08;
        return frc::ChassisSpeeds{
            -xLimiter.Calculate(frc::ApplyDeadband(controller.GetLeftY(), deadband)) * MAX_VELOCITY,
            -yLimiter.Calculate(frc::ApplyDeadband(controller.GetLeftX(), deadband)) * MAX_VELOCITY,
            -rotLimiter.Calculate(frc::ApplyDeadband(controller.GetRightX(), deadband)) *
                MAX_ANGULAR_VELOCITY};
      },
      true);
}

// Syncs encoder values when the robot is turned on
void DriveBase::SyncSensors() {
  _frontLeft.SyncSensors();
  _frontRight.SyncSensors();
  _backLeft.SyncSensors();
  _backRight.SyncSensors();
  _gyro.Calibrate();
}

frc::Rotation2d DriveBase::GetHeading() {
  return _gyro.GetRotation2d();
}

units::meters_per_second_t DriveBase::GetVelocity() {
  auto robotDisplacement =
      _prevPose.Translation().Distance(_poseEstimator.GetEstimatedPosition().Translation());
  return units::meters_per_second_t{robotDisplacement / 20_ms};
}

void DriveBase::UpdateOdometry() {
  auto fl = _frontLeft.GetPosition();
  auto fr = _frontRight.GetPosition();
  auto bl = _backLeft.GetPosition();
  auto br = _backRight.GetPosition();

  _prevPose = _poseEstimator.GetEstimatedPosition();
  _poseEstimator.Update(GetHeading(), {fl, fr, bl, br});
  _fieldDisplay.SetRobotPose(_poseEstimator.GetEstimatedPosition());
}

frc2::CommandPtr DriveBase::DriveToPose(frc::Pose2d targetPose) {
  DisplayPose("drivebase/targetPose", targetPose);

  auto calcSpeeds = [this, targetPose] {
    Xcontroller.SetSetpoint(targetPose.X().value());
    Ycontroller.SetSetpoint(targetPose.Y().value());
    Rcontroller.SetSetpoint(targetPose.Rotation().Radians().value());

    frc::Pose2d currentPosition = _poseEstimator.GetEstimatedPosition();
    double speedX = Xcontroller.Calculate(currentPosition.X().value());
    double speedY = Ycontroller.Calculate(currentPosition.Y().value());
    double speedRot = Rcontroller.Calculate(currentPosition.Rotation().Radians().value());

    speedX = std::clamp(speedX, -0.5, 0.5);
    speedY = std::clamp(speedY, -0.5, 0.5);
    speedRot = std::clamp(speedRot, -2.0, 2.0);

    // Drive speeds are relative to your alliance wall. Flip if we are on red,
    // since we are using global coordinates (blue alliance at 0,0)
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed && frc::RobotBase::IsReal()) {
      speedX = -speedX;
      speedY = -speedY;
    }
    return frc::ChassisSpeeds{speedX * 1_mps, speedY * 1_mps, speedRot * 1_rad_per_s};
  };

  return Drive(calcSpeeds, true);
}

bool DriveBase::IsAtPose(frc::Pose2d pose) {
  auto currentPose = _poseEstimator.GetEstimatedPosition();
  auto rotError = currentPose.Rotation() - pose.Rotation();
  auto posError = currentPose.Translation().Distance(pose.Translation());

  return units::math::abs(rotError.Degrees()) < 1_deg && posError < 1_cm;
}

frc2::CommandPtr DriveBase::ResetGyroHeading(units::degree_t startingAngle) {
  return RunOnce([this, startingAngle] {
    _gyro.Reset();
    _gyro.SetAngleAdjustment(startingAngle.value());
  });
}

frc::Pose2d DriveBase::GetPose() {
  return _poseEstimator.GetEstimatedPosition();
}

void DriveBase::SetPose(frc::Pose2d pose) {
  auto fl = _frontLeft.GetPosition();
  auto fr = _frontRight.GetPosition();
  auto bl = _backLeft.GetPosition();
  auto br = _backRight.GetPosition();
  _poseEstimator.ResetPosition(GetHeading(), {fl, fr, bl, br}, pose);
}

void DriveBase::DisplayPose(std::string label, frc::Pose2d pose) {
  _fieldDisplay.GetObject(label)->SetPose(pose);
}

void DriveBase::DisplayTrajectory(std::string name, frc::Trajectory trajectory) {
  _fieldDisplay.GetObject(name)->SetTrajectory(trajectory);
}

frc2::CommandPtr DriveBase::AddVisionMeasurement(frc::Pose2d pose, units::second_t timeStamp) {
  return RunOnce([&, pose, timeStamp] { _poseEstimator.AddVisionMeasurement(pose, timeStamp); });
}

void DriveBase::SetNeutralMode(NeutralMode mode) {
  _frontLeft.SetNeutralMode(mode);
  _frontRight.SetNeutralMode(mode);
  _backLeft.SetNeutralMode(mode);
  _backRight.SetNeutralMode(mode);
}

units::degree_t DriveBase::GetPitch() {
  return _gyro.GetPitch() * 1_deg;
}