#pragma once

#include <frc2/command/SubsystemBase.h>
#include "IDs.h"
#include "utilities/ICSparkMax.h"
#include <utilities/DigitalInputTrigger.h>
#include <frc/controller/ArmFeedforward.h>
#include <wpi/interpolating_map.h>
#include <optional>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <units/mass.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/angle.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/simulation/DIOSim.h>
#include <frc2/command/CommandPtr.h>
#include <RobotContainer.h>

class Arm : public frc2::SubsystemBase {
 public:
  static Arm& GetInstance() {
    static Arm inst;
    return inst;
  }

  struct Angles {
    units::radian_t bottomAngle;
    units::radian_t topAngle;
  };

  void Periodic() override;
  void SimulationPeriodic() override;

  void SetBottomAngle(units::radian_t angle);
  std::optional<Angles> CoordsToAngles(units::meter_t x, units::meter_t y);
  bool LocatingSwitchIsHit();
  bool OnTarget();
  frc::Translation2d GetEndEffectorPosition();
  units::radian_t GetGroundToTopArmAngle();

  // Commands
  frc2::CommandPtr DriveToCoords(frc::Translation2d coord);
  frc2::CommandPtr DriveToCoords(units::meter_t x, units::meter_t y);
  frc2::CommandPtr DriveToAngles(units::radian_t bottomAngle, units::radian_t topAngle);
  frc2::CommandPtr ToHighCone() { return DriveToCoords(125_cm, 124_cm); }
  frc2::CommandPtr ToHighCube() { return DriveToCoords(146.8_cm, 98_cm); }
  frc2::CommandPtr ToMidCone() { return DriveToCoords(95.5_cm, 74.3_cm); }
  frc2::CommandPtr ToMidCube() { return DriveToCoords(104.4_cm, 52.1_cm); }
  frc2::CommandPtr ToLowHybrid() { return DriveToCoords(45_cm, 15_cm); }
  frc2::CommandPtr ToSubstation() { return DriveToCoords(0.533_m, 0.847_m); }
  frc2::CommandPtr ToDefault() { return DriveToCoords(44_cm, 4_cm); }
  frc2::CommandPtr ToPreScore();

 private:
  Arm();

  // Data
  frc::Translation2d _endEffectorTarget{0.5_m, 0.5_m};
  static constexpr units::degree_t BOTTOM_LIMIT_ANGLE = 124.3_deg;

  static constexpr double BOTTOM_GEAR_RATIO = 218.27;
  static constexpr units::kilogram_t BOTTOM_ARM_MASS = 1_kg;
  static constexpr units::degrees_per_second_t BOTTOM_MAX_VEL = 18_deg_per_s;
  static constexpr units::degrees_per_second_squared_t BOTTOM_MAX_ACCEL = 90_deg_per_s_sq;
  static constexpr units::degree_t BOTTOM_TOLERANCE = 0.5_deg;
  static constexpr units::meter_t BOTTOM_ARM_LENGTH = 0.9_m;
  static constexpr units::degree_t BOTTOM_MIN_ANGLE = -180_deg;  // only sim
  static constexpr units::degree_t BOTTOM_MAX_ANGLE = 180_deg;   // only sim

  static constexpr double TOP_GEAR_RATIO = 155.91;
  static constexpr units::kilogram_t TOP_ARM_MASS = 1_kg;
  static constexpr units::degrees_per_second_t TOP_MAX_VEL = 24_deg_per_s;
  static constexpr units::degrees_per_second_squared_t TOP_MAX_ACCEL = 90_deg_per_s_sq;
  static constexpr units::degree_t TOP_TOLERANCE = 0.5_deg;
  static constexpr units::meter_t TOP_ARM_LENGTH = 1_m;
  static constexpr units::degree_t TOP_MIN_ANGLE = -180_deg;  // only sim
  static constexpr units::degree_t TOP_MAX_ANGLE = 180_deg;   // only sim

  // Controllers
  static constexpr double TOP_P = 0.0;
  static constexpr double TOP_I = 0.0;
  static constexpr double TOP_D = 0.0;
  static constexpr double TOP_F = 15;
  static constexpr double BOTTOM_P = 0.0;
  static constexpr double BOTTOM_I = 0.0;
  static constexpr double BOTTOM_D = 0.0;
  static constexpr double BOTTOM_F = 30;
  frc::ArmFeedforward _topArmGravityFF{0_V, -0.5_V, 0_V / 1_rad_per_s, 0_V / 1_rad_per_s_sq};
  frc::ArmFeedforward _bottomArmGravityFF{0_V, 0.5_V, 0_V / 1_rad_per_s, 0_V / 1_rad_per_s_sq};

  // Actuators
  ICSparkMax<> _bottomMotor{canid::armMotorBottom};
  ICSparkMax<> _topMotor{canid::armMotorTop};
  ICSparkMax<> _topFollower{canid::armMotorTopFollow};
  ICSparkMax<> _bottomFollower{canid::armMotorBottomFollow};

  // Sensors
  DigitalInputTrigger _topSensor{dio::armTopSensor};
  DigitalInputTrigger _bottomSensor{dio::armBottomSensor};

  // Simulation
  frc::sim::SingleJointedArmSim _topArmSim{
      frc::DCMotor::NEO(2),
      TOP_GEAR_RATIO,
      frc::sim::SingleJointedArmSim::EstimateMOI(TOP_ARM_LENGTH, TOP_ARM_MASS),
      TOP_ARM_LENGTH,
      TOP_MIN_ANGLE,
      TOP_MAX_ANGLE,
      false,  // ignore gravity, its too hard and insignificant in our double
              // jointed arm to be worth simulating
  };
  frc::sim::SingleJointedArmSim _bottomArmSim{
      frc::DCMotor::NEO(2),
      BOTTOM_GEAR_RATIO,
      frc::sim::SingleJointedArmSim::EstimateMOI(BOTTOM_ARM_LENGTH, BOTTOM_ARM_MASS),
      BOTTOM_ARM_LENGTH,
      BOTTOM_MIN_ANGLE,
      BOTTOM_MAX_ANGLE,
      false,  // ignore gravity, its too hard and insignificant in our double
              // jointed arm to be worth simulating
  };

  // Display
  frc::Mechanism2d _doubleJointedArmMech{3, 3};  // canvas width and height
  frc::MechanismRoot2d* _root = _doubleJointedArmMech.GetRoot("armRoot", 1, 1);  // root x and y
  frc::MechanismLigament2d* _bottomArmLigament = _root->Append<frc::MechanismLigament2d>(
      "bottomArmligament", BOTTOM_ARM_LENGTH.value(), 5_deg);
  frc::MechanismLigament2d* _topArmLigament = _bottomArmLigament->Append<frc::MechanismLigament2d>(
      "topArmLigament", TOP_ARM_LENGTH.value(), 5_deg);
};