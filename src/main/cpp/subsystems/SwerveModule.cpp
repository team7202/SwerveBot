// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"

SwerveModule::SwerveModule(int driveMotorId, int turnMotorId, bool driveMotorReversed, bool turnMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, bool absoluteEncoderReversed)
    : absoluteEncoder(ctre::phoenix6::hardware::CANcoder(absoluteEncoderId)),
      driveMotor(rev::CANSparkMax(driveMotorId, rev::CANSparkLowLevel::MotorType::kBrushless)),
      turnMotor(rev::CANSparkMax(driveMotorId, rev::CANSparkLowLevel::MotorType::kBrushless)),
      driveEncoder(driveMotor.GetEncoder()),
      turnEncoder(turnMotor.GetEncoder()),
      turnPID(ModuleConstants::kPTurn, 0.0, 0.0)
{
    this->absoluteEncoderOffset = absoluteEncoderOffset;
    this->absoluteEncoderReversed = absoluteEncoderReversed;

    this->driveMotor.SetInverted(driveMotorReversed);
    this->turnMotor.SetInverted(turnMotorReversed);

    this->driveEncoder.SetPositionConversionFactor(ModuleConstants::kDriveEncoderRot2Meter.to<double>());
    this->driveEncoder.SetVelocityConversionFactor(ModuleConstants::kDriveEncoderRPM2MeterPerSec.to<double>());
    this->turnEncoder.SetPositionConversionFactor(ModuleConstants::kTurnEncoderRot2Rad.to<double>());
    this->turnEncoder.SetVelocityConversionFactor(ModuleConstants::kTurnEncoderRPM2RadPerSec.to<double>());

    turnPID.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
}

frc::SwerveModulePosition SwerveModule::GetPosition()
{
    return {units::meter_t(driveEncoder.GetPosition()), units::radian_t(turnEncoder.GetPosition())};
}

units::meters_per_second_t SwerveModule::GetDriveVelocity()
{
    return units::meters_per_second_t(driveEncoder.GetVelocity());
}

double SwerveModule::GetTurnVelocity()
{
    return turnEncoder.GetVelocity();
}

// TODO: Convert Encoder Position to Radians if it isn't already
double SwerveModule::GetAbsoluteEncoderPosition()
{
    double angle = absoluteEncoder.GetPosition().GetValueAsDouble();
    angle -= absoluteEncoderOffset;
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
}

void SwerveModule::ResetEncoders()
{
    driveEncoder.SetPosition(0);
    turnEncoder.SetPosition(GetAbsoluteEncoderPosition());
}

frc::SwerveModuleState SwerveModule::GetState()
{
    return frc::SwerveModuleState(GetDriveVelocity(), GetPosition().angle);
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState state)
{
    if (std::abs(state.speed.to<double>()) < 0.001)
    {
        Stop();
        return;
    }

    state = frc::SwerveModuleState::Optimize(state, GetState().angle);
    driveMotor.Set(state.speed.to<double>() / DriveConstants::kPhysicalMaxSpeedMetersPerSecond.to<double>());
    turnMotor.Set(turnPID.Calculate(GetPosition().angle.Radians().to<double>(), state.angle.Radians().to<double>()));
    frc::SmartDashboard::PutString("Swerve[" + std::to_string(absoluteEncoder.GetDeviceID()) + "] State", std::to_string(state.angle.Degrees().to<double>()));
}

void SwerveModule::Stop()
{
    driveMotor.Set(0);
    turnMotor.Set(0);
}