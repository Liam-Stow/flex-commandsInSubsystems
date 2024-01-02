#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/sim/CANcoderSimState.hpp>
#include <ctre/phoenix6/sim/TalonFXSimState.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/simulation/DCMotorSim.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/length.h>
#include <units/angular_velocity.h>
#include <memory>
#include <numbers>

class SwerveModule {
 public:
  SwerveModule(int driveMotorID, int turnMotorID, int cancoderID,
               double cancoderMagOffset);
  void SetDesiredState(const frc::SwerveModuleState& state);
  void SyncSensors();
  void SendSensorsToDash();
  void SetDesiredAngle(units::degree_t angle);
  void SetDesiredVelocity(units::meters_per_second_t velocity);
  void StopMotors();
  void EnableBreakMode(bool enableBreakMode);
  void UpdateSim(units::second_t deltaTime);
  frc::Rotation2d GetAngle();
  units::meters_per_second_t GetSpeed();
  units::meter_t GetDistanceTravelled();
  frc::SwerveModuleState GetState();
  frc::SwerveModulePosition GetPosition();

 private:
  // Mechanical Constants
  static constexpr double TURNING_GEAR_RATIO = 150.0 / 7.0;
  static constexpr double DRIVE_GEAR_RATIO = 6.75;  // L2 - Fast kit
  static constexpr units::meter_t WHEEL_RADIUS = 0.0508_m;
  static constexpr units::meter_t WHEEL_CIRCUMFERENCE = 2 * std::numbers::pi * WHEEL_RADIUS;

  // Electronics
  ctre::phoenix6::hardware::TalonFX _driveMotor;
  ctre::phoenix6::hardware::TalonFX _turnMotor;
  ctre::phoenix6::hardware::CANcoder _cancoder;
  
  // string versions of device ids, 
  // store so we don't have to compute them every loop for logging
  std::string _turnID;
  std::string _driveID;
  std::string _cancoderID;

  // Controllers
  frc::SimpleMotorFeedforward<units::meters> _driveFeedForward{
      0.62004_V, 2.2731_V / 1_mps, 0.23244_V / 1_mps_sq};

  // Simulation
  frc::sim::DCMotorSim _driveMotorSim{frc::DCMotor::Falcon500(), DRIVE_GEAR_RATIO, 0.000001_kg_sq_m};
  frc::sim::DCMotorSim _turnMotorSim{frc::DCMotor::Falcon500(), TURNING_GEAR_RATIO, 0.000000001_kg_sq_m};

  // Conversions
  units::turns_per_second_t RobotVelToWheelVel(units::meters_per_second_t robotVel) {
    return robotVel.value() / WHEEL_CIRCUMFERENCE.value() * 1_tps;
  }
  units::meters_per_second_t WheelVelToRobotVel(units::turns_per_second_t wheelVel) {
    return wheelVel.value() * WHEEL_CIRCUMFERENCE.value() * 1_mps;
  }
  units::turn_t RobotDistToWheelDist(units::meter_t robotDist) {
    return robotDist.value() / WHEEL_CIRCUMFERENCE.value() * 1_tr;
  }
  units::meter_t WheelDistToRobotDist(units::turn_t wheelDist) {
    return wheelDist.value() * WHEEL_CIRCUMFERENCE.value() * 1_m;
  }
};
