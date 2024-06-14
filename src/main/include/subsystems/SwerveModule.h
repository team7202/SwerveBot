// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.h"

#include <ctre/phoenix6/CANcoder.hpp>

#include <frc/controller/PIDController.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <rev/CANSparkMax.h>

class SwerveModule
{

public:
  rev::CANSparkMax driveMotor, turnMotor;

private:
  rev::SparkRelativeEncoder driveEncoder, turnEncoder;
  ctre::phoenix6::hardware::CANcoder absoluteEncoder;

private:
  frc::PIDController turnPID;
  bool absoluteEncoderReversed;
  double absoluteEncoderOffset;

public:
  SwerveModule(int driveMotorId, int turnMotorId, bool driveMotorReversed, bool turnMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, bool absoluteEncoderReversed);

public:
  frc::SwerveModulePosition GetPosition();
  units::unit_t<units::velocity::meters_per_second, double, units::linear_scale> GetDriveVelocity();
  units::radians_per_second_t GetTurnVelocity();
  double GetAbsoluteEncoderPosition();
  void ResetEncoders();
  frc::SwerveModuleState GetState();
  void SetDesiredState(frc::SwerveModuleState state);
  void Stop();
};
