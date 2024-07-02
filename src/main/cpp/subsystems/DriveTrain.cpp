// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>
#include <frc/DriverStation.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include "subsystems/DriveTrain.h"

/**
 * Constructor for the DriveTrain subsystem.
 * Initializes the swerve modules and other components.
 */
DriveTrain::DriveTrain()
    : frontLeft(SwerveModule(
          DriveConstants::kFrontLeftDriveMotorPort,
          DriveConstants::kFrontLeftTurnMotorPort,
          DriveConstants::kFrontLeftDriveEncoderReversed,
          DriveConstants::kFrontLeftTurnEncoderReversed,
          DriveConstants::kFrontLeftDriveAbsoluteEncoderPort,
          DriveConstants::kFrontLeftDriveAbsoluteEncoderOffset,
          DriveConstants::kFrontLeftDriveAbsoluteEncoderReversed)),
      frontRight(SwerveModule(
          DriveConstants::kFrontRightDriveMotorPort,
          DriveConstants::kFrontRightTurnMotorPort,
          DriveConstants::kFrontRightDriveEncoderReversed,
          DriveConstants::kFrontRightTurnEncoderReversed,
          DriveConstants::kFrontRightDriveAbsoluteEncoderPort,
          DriveConstants::kFrontRightDriveAbsoluteEncoderOffset,
          DriveConstants::kFrontRightDriveAbsoluteEncoderReversed)),
      backLeft(SwerveModule(
          DriveConstants::kBackLeftDriveMotorPort,
          DriveConstants::kBackLeftTurnMotorPort,
          DriveConstants::kBackLeftDriveEncoderReversed,
          DriveConstants::kBackLeftTurnEncoderReversed,
          DriveConstants::kBackLeftDriveAbsoluteEncoderPort,
          DriveConstants::kBackLeftDriveAbsoluteEncoderOffset,
          DriveConstants::kBackLeftDriveAbsoluteEncoderReversed)),
      backRight(SwerveModule(
          DriveConstants::kBackRightDriveMotorPort,
          DriveConstants::kBackRightTurnMotorPort,
          DriveConstants::kBackRightDriveEncoderReversed,
          DriveConstants::kBackRightTurnEncoderReversed,
          DriveConstants::kBackRightDriveAbsoluteEncoderPort,
          DriveConstants::kBackRightDriveAbsoluteEncoderOffset,
          DriveConstants::kBackRightDriveAbsoluteEncoderReversed)),
      gyro(DriveConstants::kGyroPort),
      odometer(DriveConstants::kDriveKinematics, GetRotation2d(), {frontLeft.GetPosition(), frontRight.GetPosition(), backLeft.GetPosition(), backRight.GetPosition()})
{
    // Delay to ensure the gyro is ready
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    ZeroHeading();

    pathplanner::AutoBuilder::configureHolonomic(
        [this]()
        { return GetPose2d(); },
        [this](frc::Pose2d pose)
        { ResetOdometry(pose); },
        [this]()
        { return GetSpeeds(); },
        [this](frc::ChassisSpeeds speeds)
        { DriveRobotRelative(speeds); },
        pathplanner::HolonomicPathFollowerConfig(     // HolonomicPathFollowerConfig, this should likely live in your Constants class
            pathplanner::PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            pathplanner::PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            4.5_mps,                                  // Max module speed, in m/s
            0.4_m,                                    // Drive base radius in meters. Distance from robot center to furthest module.
            pathplanner::ReplanningConfig()           // Default path replanning config. See the API for the options here
            ),
        []()
        {
            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance)
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            return false;
        },
        this);
};

/**
 * Zeros the gyro heading.
 */
void DriveTrain::ZeroHeading()
{
    gyro.Reset();
    std::cout << "Gyro Heading Reset!";
}

/**
 * Returns the current heading of the robot.
 * The heading is the direction in which the robot is facing.
 * It is measured in degrees and is based on the gyro sensor.
 * The heading is normalized to be within the range of -180 to 180 degrees.
 * @return The current heading in degrees.
 */
units::angle::degree_t DriveTrain::GetHeading()
{
    return units::angle::degree_t(std::remainder(gyro.GetYaw().GetValue().value(), 360));
}

/**
 * Returns the current rotation of the robot as a Rotation2d object.
 * @return The current rotation.
 */
frc::Rotation2d DriveTrain::GetRotation2d()
{
    return frc::Rotation2d(GetHeading());
}

/**
 * Returns the current pose of the robot.
 * frc::Pose2d is a class that holds this information.
 * - X: The x-coordinate of the robot's position.
 * - Y: The y-coordinate of the robot's position.
 * - Rotation: The robot's orientation in radians.
 * @return The current pose.
 */
frc::Pose2d DriveTrain::GetPose2d()
{
    return odometer.GetPose();
}

/**
 * Resets the odometry to a specific pose.
 * The odometry keeps track of the robot's position and orientation on the field.
 * Resetting it allows you to set a new starting position and orientation.
 * @param pose The pose to reset the odometry to, consisting of (x, y) position and rotation.
 */
void DriveTrain::ResetOdometry(frc::Pose2d pose)
{
    odometer.ResetPosition(GetRotation2d(), {frontLeft.GetPosition(), frontRight.GetPosition(), backLeft.GetPosition(), backRight.GetPosition()}, pose);
}

/**
 * Stops all swerve modules.
 */
void DriveTrain::StopModules()
{
    frontLeft.Stop();
    frontRight.Stop();
    backLeft.Stop();
    backRight.Stop();
}

/**
 * Sets the desired state for each swerve module.
 * @param desiredStates The desired states of the swerve modules.
 */
void DriveTrain::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates)
{
    DriveConstants::kDriveKinematics.DesaturateWheelSpeeds(&desiredStates, DriveConstants::kPhysicalMaxSpeedMetersPerSecond);

    frontLeft.SetDesiredState(desiredStates[0]);
    frontRight.SetDesiredState(desiredStates[1]);
    backLeft.SetDesiredState(desiredStates[2]);
    backRight.SetDesiredState(desiredStates[3]);
}

/**
 * Drives the robot using the specified chassis speeds.
 * @param chassisSpeeds The desired chassis speeds.
 */
void DriveTrain::DriveRobotRelative(const frc::ChassisSpeeds &robotRelativeSpeeds)
{
    SetModuleStates(DriveConstants::kDriveKinematics.ToSwerveModuleStates(robotRelativeSpeeds));
}

// This method will be called once per scheduler run
void DriveTrain::Periodic()
{
    odometer.Update(GetRotation2d(), {frontLeft.GetPosition(), frontRight.GetPosition(), backLeft.GetPosition(), backRight.GetPosition()});
    frc::SmartDashboard::PutNumber("Robot Heading", GetHeading().value());
    frc::Pose2d pose = GetPose2d();
    frc::SmartDashboard::PutString("Robot Location", "X: " + std::to_string(pose.X().value()) + " Y: " + std::to_string(pose.Y().value()));
}
