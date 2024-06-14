// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveTrain.h"

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
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    ZeroHeading();
};

void DriveTrain::ZeroHeading()
{
    gyro.Reset();
}

units::angle::degree_t DriveTrain::GetHeading()
{
    return units::angle::degree_t(std::remainder(gyro.GetAngle(), 360));
}

frc::Rotation2d DriveTrain::GetRotation2d()
{
    return frc::Rotation2d(GetHeading());
}

frc::Pose2d DriveTrain::GetPose2d()
{
    return odometer.GetPose();
}

void DriveTrain::ResetOdometry(frc::Pose2d pose)
{
    odometer.ResetPosition(GetRotation2d(), {frontLeft.GetPosition(), frontRight.GetPosition(), backLeft.GetPosition(), backRight.GetPosition()}, pose);
}

void DriveTrain::StopModules()
{
    frontLeft.Stop();
    frontRight.Stop();
    backLeft.Stop();
    backRight.Stop();
}

void DriveTrain::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates)
{
    DriveConstants::kDriveKinematics.DesaturateWheelSpeeds(&desiredStates, DriveConstants::kPhysicalMaxSpeedMetersPerSecond);
    frontLeft.SetDesiredState(desiredStates[0]);
    frontRight.SetDesiredState(desiredStates[1]);
    backLeft.SetDesiredState(desiredStates[2]);
    backRight.SetDesiredState(desiredStates[3]);
}

// This method will be called once per scheduler run
void DriveTrain::Periodic()
{
    odometer.Update(GetRotation2d(), {frontLeft.GetPosition(), frontRight.GetPosition(), backLeft.GetPosition(), backRight.GetPosition()});
    frc::SmartDashboard::PutNumber("Robot Heading", GetHeading().to<double>());
    frc::Pose2d pose = GetPose2d();
    frc::SmartDashboard::PutString("Robot Location", "X: " + std::to_string(pose.X().to<double>()) + " Y: " + std::to_string(pose.Y().to<double>()));
}
