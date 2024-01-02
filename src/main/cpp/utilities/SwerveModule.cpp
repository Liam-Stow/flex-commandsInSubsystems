#include "utilities/SwerveModule.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include <frc/RobotBase.h>

using namespace ctre::phoenix6;

SwerveModule::SwerveModule(int driveMotorID, int turnMotorID, int cancoderID,
                           double cancoderMagOffset)
    : _driveMotor(driveMotorID, "Canivore"),
      _turnMotor(turnMotorID, 40_A),
      _cancoder(cancoderID, "Canivore"),
      _driveID(std::to_string(driveMotorID)),
      _turnID(std::to_string(turnMotorID)),
      _cancoderID(std::to_string(cancoderID)) {
  using namespace ctre::phoenix6::configs;
  using namespace ctre::phoenix6::signals;

  // Config CANCoder
  CANcoderConfiguration cancoderConfig;
  cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue::Signed_PlusMinusHalf;
  cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue::CounterClockwise_Positive;
  cancoderConfig.MagnetSensor.MagnetOffset = cancoderMagOffset;
  _cancoder.GetConfigurator().Apply(cancoderConfig);

  // Config Turning Motor
  _turnMotor.SetConversionFactor(TURNING_GEAR_RATIO);
  _turnMotor.EnableSensorWrapping(0, 1);
  _turnMotor.SetPIDFF(50, 0, 1);
  _turnMotor.SetInverted(true);
  _turnMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  // Config Driving Motor
  TalonFXConfiguration driveConfig;
  driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue::RotorSensor;
  driveConfig.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;
  driveConfig.ClosedLoopGeneral.ContinuousWrap = false;
  driveConfig.Slot0.kP = 0.031489;
  driveConfig.Slot0.kI = 0.0;
  driveConfig.Slot0.kD = 0.0;
  driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  driveConfig.CurrentLimits.SupplyCurrentLimit = 50;      // Amps
  driveConfig.CurrentLimits.SupplyCurrentThreshold = 60;  // Amps
  driveConfig.CurrentLimits.SupplyTimeThreshold = 0.1;    // Seconds
  driveConfig.MotorOutput.Inverted = true;  // +V should rotate the motor counter-clockwise
  driveConfig.MotorOutput.NeutralMode = NeutralModeValue::Brake;
  _driveMotor.GetConfigurator().Apply(driveConfig);
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState) {
  auto moduleAngle = GetAngle();

  // Optimize the reference state to avoid spinning further than 90 degrees
  auto targetState = frc::SwerveModuleState::Optimize(referenceState, moduleAngle);

  // Scale speed by cosine of angle error. This scales down movement
  // perpendicular to the desired direction of travel that can occur when
  // modules change directions. This results in smoother driving.
  targetState.speed *= (targetState.angle - moduleAngle).Cos();

  // Drive! These functions send targets to motors
  SetDesiredAngle(targetState.angle.Degrees());
  SetDesiredVelocity(targetState.speed);
}

frc::Rotation2d SwerveModule::GetAngle() {
  units::radian_t turnAngle = _turnMotor.GetPosition();
  return turnAngle;
}

units::meters_per_second_t SwerveModule::GetSpeed() {
  return WheelVelToRobotVel(_driveMotor.GetVelocity().GetValue());
}

units::meter_t SwerveModule::GetDistanceTravelled() {
  units::turn_t driveRotations = BaseStatusSignal::GetLatencyCompensatedValue(
      _driveMotor.GetPosition(), _driveMotor.GetVelocity());
  return WheelDistToRobotDist(driveRotations);
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  return {GetDistanceTravelled(), GetAngle()};
}

frc::SwerveModuleState SwerveModule::GetState() {
  return {GetSpeed(), GetAngle()};
}

void SwerveModule::SetDesiredAngle(units::degree_t angle) {
  _turnMotor.SetPositionTarget(angle);
}

void SwerveModule::SetDesiredVelocity(units::meters_per_second_t velocity) {
  auto ffvolts = _driveFeedForward.Calculate(velocity);
  auto velocityRequest = controls::VelocityVoltage{RobotVelToWheelVel(velocity)};
  _driveMotor.SetControl(velocityRequest.WithFeedForward(ffvolts));
}

void SwerveModule::StopMotors() {
  _driveMotor.StopMotor();
  _turnMotor.StopMotor();
}

void SwerveModule::EnableBreakMode(bool enableBreakMode) {
  configs::MotorOutputConfigs config;
  auto& configurator = _driveMotor.GetConfigurator();
  configurator.Refresh(config);
  if (enableBreakMode) {
    config.NeutralMode = signals::NeutralModeValue::Brake;
  } else {
    config.NeutralMode = signals::NeutralModeValue::Coast;
  }
  configurator.Apply(config);
}

void SwerveModule::SyncSensors() {
  _turnMotor.SetPosition(_cancoder.GetAbsolutePosition().GetValue());
}

void SwerveModule::SendSensorsToDash() {
  std::string driveName = "swerve/drive " + _driveID;
  std::string turnName = "swerve/turn " + _turnID;
  std::string encoderName = "swerve/cancoder " + _cancoderID;

  // clang-format off
  frc::SmartDashboard::PutNumber(  driveName + " target",   _driveMotor.GetClosedLoopReference().GetValue()); 
  frc::SmartDashboard::PutNumber(  driveName + " velocity", _driveMotor.GetVelocity().GetValue().value()); 
  frc::SmartDashboard::PutNumber(  driveName + " error",    _driveMotor.GetClosedLoopError().GetValue()); 
  frc::SmartDashboard::PutNumber(   turnName + " target",   _turnMotor.GetPositionTarget().value()); 
  frc::SmartDashboard::PutNumber(   turnName + " position", _turnMotor.GetPosition().value()); 
  frc::SmartDashboard::PutNumber(   turnName + " error",    _turnMotor.GetPosError().value()); 
  frc::SmartDashboard::PutNumber(encoderName + " position", _cancoder.GetAbsolutePosition().GetValue().value());
  // clang-format on
}

void SwerveModule::UpdateSim(units::second_t deltaTime) {
  // Drive Motor (TalonFX)
  auto& driveState = _driveMotor.GetSimState();
  _driveMotorSim.SetInputVoltage(driveState.GetMotorVoltage());
  _driveMotorSim.Update(deltaTime);
  driveState.SetRawRotorPosition(_driveMotorSim.GetAngularPosition() * DRIVE_GEAR_RATIO);
  driveState.SetRotorVelocity(_driveMotorSim.GetAngularVelocity() * DRIVE_GEAR_RATIO);

  // Turn Motor (Spark Max)
  auto turnVolts = _turnMotor.GetSimVoltage();
  _turnMotorSim.SetInputVoltage(_turnMotor.GetSimVoltage());
  _turnMotorSim.Update(deltaTime);
  _turnMotor.UpdateSimEncoder(_turnMotorSim.GetAngularPosition(),
                              _turnMotorSim.GetAngularVelocity());

  // CANcoders are attached directly to the mechanism, so don't account for the steer gearing
  auto& cancoderState = _cancoder.GetSimState();
  cancoderState.SetRawPosition(_turnMotorSim.GetAngularPosition());
  cancoderState.SetVelocity(_turnMotorSim.GetAngularVelocity());
}