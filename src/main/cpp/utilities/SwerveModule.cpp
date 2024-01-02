#include "utilities/SwerveModule.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include <frc/RobotBase.h>

using namespace ctre::phoenix6;

SwerveModule::SwerveModule(int driveMotorID, int turnMotorID, int cancoderID,
                           double cancoderMagOffset)
    : _driveMotor(driveMotorID, "Canivore"),
      _turnMotor(turnMotorID, "Canivore"),
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
  TalonFXConfiguration turnConfig;
  turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue::RotorSensor;
  turnConfig.Feedback.SensorToMechanismRatio = TURNING_GEAR_RATIO;
  turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
  turnConfig.Slot0.kP = 50;
  turnConfig.Slot0.kI = 0.0;
  turnConfig.Slot0.kD = 1;
  turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  turnConfig.CurrentLimits.SupplyCurrentLimit = 25;      // Amps
  turnConfig.CurrentLimits.SupplyCurrentThreshold = 40;  // Amps
  turnConfig.CurrentLimits.SupplyTimeThreshold = 0.1;    // Seconds
  turnConfig.MotorOutput.Inverted = true;  // +V should steer the wheel counter-clockwise
  turnConfig.MotorOutput.NeutralMode = NeutralModeValue::Brake;
  _turnMotor.GetConfigurator().Apply(turnConfig);

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
  units::radian_t turnAngle = BaseStatusSignal::GetLatencyCompensatedValue(
      _turnMotor.GetPosition(), _turnMotor.GetVelocity());
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
  _turnMotor.SetControl(controls::PositionVoltage{angle});
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
  frc::SmartDashboard::PutNumber(  driveName + " target",   _driveMotor.GetClosedLoopReference().GetValue()    ); 
  frc::SmartDashboard::PutNumber(  driveName + " velocity", _driveMotor.GetVelocity().GetValue().value()       ); 
  frc::SmartDashboard::PutNumber(  driveName + " error",    _driveMotor.GetClosedLoopError().GetValue()        ); 
  frc::SmartDashboard::PutNumber(   turnName + " target",   _turnMotor.GetClosedLoopReference().GetValue()     ); 
  frc::SmartDashboard::PutNumber(   turnName + " position", _turnMotor.GetPosition().GetValue().value()        ); 
  frc::SmartDashboard::PutNumber(   turnName + " error",    _turnMotor.GetClosedLoopError().GetValue()         ); 
  frc::SmartDashboard::PutNumber(encoderName + " position", _cancoder.GetAbsolutePosition().GetValue().value() );
  // clang-format on
}

void SwerveModule::UpdateSim(units::second_t deltaTime) {
  // Drive Motor
  auto& driveState = _driveMotor.GetSimState();
  _driveMotorSim.SetInputVoltage(driveState.GetMotorVoltage());
  _driveMotorSim.Update(deltaTime);
  driveState.SetRawRotorPosition(_driveMotorSim.GetAngularPosition() * DRIVE_GEAR_RATIO);
  driveState.SetRotorVelocity(_driveMotorSim.GetAngularVelocity() * DRIVE_GEAR_RATIO);

  // Turn Motor
  auto& turnState = _turnMotor.GetSimState();
  _turnMotorSim.SetInputVoltage(turnState.GetMotorVoltage());
  _turnMotorSim.Update(deltaTime);
  turnState.SetRawRotorPosition(_turnMotorSim.GetAngularPosition() * TURNING_GEAR_RATIO);
  turnState.SetRotorVelocity(_turnMotorSim.GetAngularVelocity() * TURNING_GEAR_RATIO);

  // CANcoders are attached directly to the mechanism, so don't account for the steer gearing
  auto& cancoderState = _cancoder.GetSimState();
  cancoderState.SetRawPosition(_turnMotorSim.GetAngularPosition());
  cancoderState.SetVelocity(_turnMotorSim.GetAngularVelocity());
}