// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.h"
#include "subsystems/SwerveModule.h"

#include <ctre/phoenix6/Pigeon2.hpp>

#include <chrono>

#include <frc/kinematics/SwerveDriveOdometry.h>

#include <frc2/command/SubsystemBase.h>

#include <future>

#include <iostream>

class DriveTrain : public frc2::SubsystemBase
{
public:
  DriveTrain();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

private:
  SwerveModule frontLeft, frontRight, backLeft, backRight;
  ctre::phoenix6::hardware::Pigeon2 gyro;
  frc::SwerveDriveOdometry<4U> odometer;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

public:
  void ZeroHeading();
  units::angle::degree_t GetHeading();
  frc::Rotation2d GetRotation2d();
  frc::Pose2d GetPose2d();
  void ResetOdometry(frc::Pose2d pose);
  void StopModules();
  void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);
  void Drive(frc::ChassisSpeeds& chassisSpeeds);
};
