// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <numbers>

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OIConstants
{
    inline constexpr int kDriverControllerPort = 0;
    inline constexpr double kDeadband = 0.1;
    inline constexpr int kDriverYAxis = 1;
    inline constexpr int kDriverXAxis = 0;
    inline constexpr int kDriverRotAxis = 4;
    inline constexpr int kDriverFieldOrientedButtonIndex = 2;

} // namespace OperatorConstants

namespace ModuleConstants
{
    // https://www.swervedrivespecialties.com/products/mk4i-swerve-module
    inline constexpr units::meter_t kWheelDiameterMeters = 4_in;
    inline constexpr double kDriveMotorGearRatio = 1 / 6.12;
    inline constexpr double kTurnMotorGearRatio = 7.0 / 150.0;
    inline constexpr units::meter_t kDriveEncoderRot2Meter = units::meter_t(kDriveMotorGearRatio * std::numbers::pi * kWheelDiameterMeters.value());
    inline constexpr units::radian_t kTurnEncoderRot2Rad = units::radian_t(kTurnMotorGearRatio * 2 * std::numbers::pi);
    inline constexpr units::meters_per_second_t kDriveEncoderRPM2MeterPerSec = units::meters_per_second_t(kDriveEncoderRot2Meter.value() / 60);
    inline constexpr units::radians_per_second_t kTurnEncoderRPM2RadPerSec = units::radians_per_second_t(kTurnEncoderRot2Rad.value() / 60);
    inline constexpr double kPTurn = 0.5;

} // namespace ModuleConstants

namespace DriveConstants
{
    // Distance between right and left wheels
    inline constexpr units::meter_t kTrackWidth = 18.5_in;
    // Distance between front and back wheels
    inline constexpr units::meter_t kWheelBase = 24_in;

    inline frc::SwerveDriveKinematics kDriveKinematics(
        frc::Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        frc::Translation2d(kWheelBase / 2, kTrackWidth / 2),
        frc::Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
        frc::Translation2d(-kWheelBase / 2, kTrackWidth / 2));

    inline constexpr int kGyroPort = 62;

    inline constexpr int kFrontLeftDriveMotorPort = 13;
    inline constexpr int kBackLeftDriveMotorPort = 1;
    inline constexpr int kFrontRightDriveMotorPort = 10;
    inline constexpr int kBackRightDriveMotorPort = 9;

    inline constexpr int kFrontLeftTurnMotorPort = 12;    
    inline constexpr int kBackLeftTurnMotorPort = 2;
    inline constexpr int kFrontRightTurnMotorPort = 11;
    inline constexpr int kBackRightTurnMotorPort = 8;

    inline constexpr bool kFrontLeftTurnEncoderReversed = false;
    inline constexpr bool kBackLeftTurnEncoderReversed = false;
    inline constexpr bool kFrontRightTurnEncoderReversed = false;
    inline constexpr bool kBackRightTurnEncoderReversed = false;

    inline constexpr bool kFrontLeftDriveEncoderReversed = false;
    inline constexpr bool kBackLeftDriveEncoderReversed = true;
    inline constexpr bool kFrontRightDriveEncoderReversed = true;
    inline constexpr bool kBackRightDriveEncoderReversed = false;

    inline constexpr int kFrontLeftDriveAbsoluteEncoderPort = 29;
    inline constexpr int kBackLeftDriveAbsoluteEncoderPort = 25;
    inline constexpr int kFrontRightDriveAbsoluteEncoderPort = 27;
    inline constexpr int kBackRightDriveAbsoluteEncoderPort = 26;

    inline constexpr bool kFrontLeftDriveAbsoluteEncoderReversed = true;
    inline constexpr bool kBackLeftDriveAbsoluteEncoderReversed = true;
    inline constexpr bool kFrontRightDriveAbsoluteEncoderReversed = true;
    inline constexpr bool kBackRightDriveAbsoluteEncoderReversed = true;

    // inline constexpr double kFrontLeftDriveAbsoluteEncoderOffset = -(278.78 * (M_PI / 180));
    inline constexpr double kFrontLeftDriveAbsoluteEncoderOffset = -(0 * (M_PI / 180));
    inline constexpr double kBackLeftDriveAbsoluteEncoderOffset = -(0 * (M_PI / 180));
    inline constexpr double kFrontRightDriveAbsoluteEncoderOffset = -(0 * (M_PI / 180));
    inline constexpr double kBackRightDriveAbsoluteEncoderOffset = -(0 * (M_PI / 180));

    inline constexpr units::meters_per_second_t kPhysicalMaxSpeedMetersPerSecond = 16.6_fps;
    inline constexpr units::radians_per_second_t kPhysicalMaxAngularSpeedRadiansPerSecond = units::radians_per_second_t(4.0 * std::numbers::pi);

    inline constexpr units::meters_per_second_t kTeleDriveMaxSpeedMetersPerSecond = units::meters_per_second_t(kPhysicalMaxSpeedMetersPerSecond.value() / 4);
    inline constexpr units::radians_per_second_t kTeleDriveMaxAngularSpeedRadiansPerSecond = units::radians_per_second_t(kPhysicalMaxAngularSpeedRadiansPerSecond.value() / 4);

    inline constexpr units::meters_per_second_t kTeleDriveMaxAccelerationUnitsPerSecond = 12.45_fps;
    inline constexpr units::radians_per_second_t kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3.0_rad_per_s;

} // namespace DriveConstants

namespace AutoConstants
{
    inline constexpr units::meters_per_second_t kMaxSpeedMetersPerSecond = DriveConstants::kPhysicalMaxSpeedMetersPerSecond / 4;
    inline constexpr units::radians_per_second_t kMaxAngularSpeedRadiansPerSecond = units::radians_per_second_t(DriveConstants::kPhysicalMaxAngularSpeedRadiansPerSecond.value() / 10.0);
    inline constexpr units::meters_per_second_squared_t kMaxAccelerationMetersPerSecondSquared = 3_mps_sq;
    inline constexpr units::radians_per_second_squared_t kMaxAngularAccelerationRadiansPerSecondSquared = units::radians_per_second_squared_t(std::numbers::pi / 4.0);
    inline constexpr double kPXController = 1.5;
    inline constexpr double kPYController = 1.5;
    inline constexpr double kPThetaController = 3;

    inline auto kThetaControllerConstraints = frc::TrapezoidProfile<units::radians>::Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);

} // namespace AutoConstants