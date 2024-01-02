#pragma once

#include <frc2/command/SubsystemBase.h>
#include <AHRS.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/controller/HolonomicDriveController.h>
#include <numbers>
#include "IDs.h"
#include "utilities/SwerveModule.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

class DriveBase : public frc2::SubsystemBase {
 public:
  static DriveBase& GetInstance() {
    static DriveBase inst;
    return inst;
  }

  void Periodic() override;
  void SimulationPeriodic() override;
  void DisplayTrajectory(std::string name, frc::Trajectory trajectory);
  void EnableBreakMode(bool enabled);
  void SetPose(frc::Pose2d pose);
  void DisplayPose(std::string label, frc::Pose2d pose);
  void SyncSensors();
  void Drive(frc::ChassisSpeeds speeds, bool fieldRelative);

  bool IsAtPose(frc::Pose2d pose);
  units::degree_t GetPitch();
  frc::Pose2d GetPose();
  frc::Rotation2d GetHeading();
  units::meters_per_second_t GetVelocity();
  frc::SwerveDriveKinematics<4> GetKinematics() { return _kinematics; }
  frc::ChassisSpeeds GetRobotRelativeSpeeds();

  static constexpr auto MAX_VELOCITY = 3_mps;
  static constexpr auto MAX_ANGULAR_VELOCITY = 180_deg_per_s;
  static constexpr auto MAX_ANGULAR_ACCEL = 3.14_rad_per_s;

  // Commands
  frc2::CommandPtr DriveCmd(std::function<frc::ChassisSpeeds()> speedsSupplier, bool fieldRelative);
  frc2::CommandPtr DriveToPose(frc::Pose2d targetPose);
  frc2::CommandPtr XboxDrive(frc2::CommandXboxController& controller);
  frc2::CommandPtr AddVisionMeasurement(frc::Pose2d pose, units::second_t timeStamp);
  frc2::CommandPtr ResetGyroHeading(units::degree_t startingAngle = 0_deg);

 private:
  DriveBase();
  void UpdateOdometry();

  // Data
  static constexpr double FRONT_LEFT_MAG_OFFSET = -47.812;
  static constexpr double FRONT_RIGHT_MAG_OFFSET = 58.887;
  static constexpr double BACK_LEFT_MAG_OFFSET = 104.326;
  static constexpr double BACK_RIGHT_MAG_OFFSET = -160.225;
  frc::Pose2d _prevPose;  // Used for velocity calculations

  // Actuators
  SwerveModule _frontLeft{canivore::frontLeftDrive, canivore::frontLeftTurn,
                          canivore::frontLeftEncoder, FRONT_LEFT_MAG_OFFSET};
  SwerveModule _frontRight{canivore::frontRightDrive, canivore::frontRightTurn,
                           canivore::frontRightEncoder, FRONT_RIGHT_MAG_OFFSET};
  SwerveModule _backLeft{canivore::backLeftDrive, canivore::backLeftTurn, canivore::backLeftEncoder,
                         BACK_LEFT_MAG_OFFSET};
  SwerveModule _backRight{canivore::backRightDrive, canivore::backRightTurn,
                          canivore::backRightEncoder, BACK_RIGHT_MAG_OFFSET};

  // Sensors
  AHRS _gyro{frc::SerialPort::kMXP};

  // Controls
  frc::Translation2d _frontLeftLocation{+0.281_m, +0.281_m};
  frc::Translation2d _frontRightLocation{+0.281_m, -0.281_m};
  frc::Translation2d _backLeftLocation{-0.281_m, +0.281_m};
  frc::Translation2d _backRightLocation{-0.281_m, -0.281_m};
  frc::SwerveDriveKinematics<4> _kinematics{_frontLeftLocation, _frontRightLocation,
                                            _backLeftLocation, _backRightLocation};
  frc::PIDController Xcontroller{0.5, 0, 0};
  frc::PIDController Ycontroller{0.5, 0, 0};
  frc::PIDController Rcontroller{1.8, 0, 0};

  frc::SwerveDrivePoseEstimator<4> _poseEstimator{
      _kinematics,
      _gyro.GetRotation2d(),
      {frc::SwerveModulePosition{0_m, _frontLeft.GetAngle()},
       frc::SwerveModulePosition{0_m, _frontRight.GetAngle()},
       frc::SwerveModulePosition{0_m, _backLeft.GetAngle()},
       frc::SwerveModulePosition{0_m, _backRight.GetAngle()}},
      frc::Pose2d()};

  // Displays
  frc::Field2d _fieldDisplay;
};
