// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"

/**
 * Constructor for SwerveModule.
 * Initializes motors, encoders, and PID controllers for a swerve module.
 *
 * @param driveMotorId The ID of the drive motor controller.
 * @param turnMotorId The ID of the turn motor controller.
 * @param driveMotorReversed Whether the drive motor is reversed.
 * @param turnMotorReversed Whether the turn motor is reversed.
 * @param absoluteEncoderId The ID of the absolute encoder.
 * @param absoluteEncoderOffset The offset value for the absolute encoder position.
 * @param absoluteEncoderReversed Whether the absolute encoder values are reversed.
 */
SwerveModule::SwerveModule(int driveMotorId, int turnMotorId, bool driveMotorReversed, bool turnMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, bool absoluteEncoderReversed)
    : absoluteEncoder(ctre::phoenix6::hardware::CANcoder(absoluteEncoderId)),
      driveMotor(rev::CANSparkMax(driveMotorId, rev::CANSparkLowLevel::MotorType::kBrushless)),
      turnMotor(rev::CANSparkMax(turnMotorId, rev::CANSparkLowLevel::MotorType::kBrushless)),
      driveEncoder(driveMotor.GetEncoder()),
      turnEncoder(turnMotor.GetEncoder()),
      turnPID(ModuleConstants::kPTurn, 0.0, 0.0)
{
    // Configure absolute encoder settings
    this->absoluteEncoderOffset = absoluteEncoderOffset;
    this->absoluteEncoderReversed = absoluteEncoderReversed;

    // Configure motor inversions
    this->driveMotor.SetInverted(driveMotorReversed);
    this->turnMotor.SetInverted(turnMotorReversed);

    // Configure encoder conversion factors
    this->driveEncoder.SetPositionConversionFactor(ModuleConstants::kDriveEncoderRot2Meter.value());
    this->driveEncoder.SetVelocityConversionFactor(ModuleConstants::kDriveEncoderRPM2MeterPerSec.value());
    this->turnEncoder.SetPositionConversionFactor(ModuleConstants::kTurnEncoderRot2Rad.value());
    this->turnEncoder.SetVelocityConversionFactor(ModuleConstants::kTurnEncoderRPM2RadPerSec.value());

    // Configure PID controller for continuous angle input
    turnPID.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
}

/**
 * Gets the current position (drive distance and turn angle) of the swerve module.
 *
 * @return The current position as a SwerveModulePosition object.
 */
frc::SwerveModulePosition SwerveModule::GetPosition()
{
    return {units::meter_t(driveEncoder.GetPosition()), units::radian_t(turnEncoder.GetPosition())};
}

/**
 * Gets the current velocity of the drive motor in meters per second.
 *
 * @return The current drive motor velocity.
 */
units::meters_per_second_t SwerveModule::GetDriveVelocity()
{
    return units::meters_per_second_t(driveEncoder.GetVelocity());
}

/**
 * Gets the current velocity of the turn motor in radians per second.
 *
 * @return The current turn motor velocity.
 */
units::radians_per_second_t SwerveModule::GetTurnVelocity()
{
    return units::radians_per_second_t(turnEncoder.GetVelocity());
}

/**
 * Gets the current position of the absolute encoder, adjusted to radians.
 * Accounts for offset and reversals.
 * TODO: Convert Encoder Position to Radians if it isn't already
 * @return The current absolute encoder position in radians.
 */
double SwerveModule::GetAbsoluteEncoderPosition()
{
    double angle = absoluteEncoder.GetPosition().GetValueAsDouble();
    angle -= absoluteEncoderOffset;
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
}

/**
 * Resets the drive and turn encoders.
 * Sets the turn encoder position based on the adjusted absolute encoder position.
 */
void SwerveModule::ResetEncoders()
{
    driveEncoder.SetPosition(0);
    turnEncoder.SetPosition(GetAbsoluteEncoderPosition());
}

/**
 * Gets the current state of the swerve module (speed and angle).
 *
 * @return The current state as a SwerveModuleState object.
 */
frc::SwerveModuleState SwerveModule::GetState()
{
    return frc::SwerveModuleState(GetDriveVelocity(), GetPosition().angle);
}

/**
 * Sets the desired state (speed and angle) for the swerve module.
 * Adjusts the turn motor using a PID controller to achieve the desired angle.
 *
 * @param state The desired state as a SwerveModuleState object.
 */
void SwerveModule::SetDesiredState(frc::SwerveModuleState state)
{
    // If speed is negligible, stop the module
    if (std::abs(state.speed.value()) < 0.001)
    {
        Stop();
        return;
    }

    // Optimize the state for minimal turn angle difference
    state = frc::SwerveModuleState::Optimize(state, GetState().angle);

    // Set drive motor speed based on desired speed
    driveMotor.Set(state.speed.value() / DriveConstants::kPhysicalMaxSpeedMetersPerSecond.value());

    // Calculate turn motor output using PID controller
    turnMotor.Set(turnPID.Calculate(GetPosition().angle.Radians().value(), state.angle.Radians().value()));

    // Update SmartDashboard with current module state
    frc::SmartDashboard::PutString("Swerve[" + std::to_string(absoluteEncoder.GetDeviceID()) + "] State", std::to_string(state.angle.Degrees().value()));
}

/**
 * Stops both the drive and turn motors of the swerve module.
 */
void SwerveModule::Stop()
{
    driveMotor.Set(0);
    turnMotor.Set(0);
}