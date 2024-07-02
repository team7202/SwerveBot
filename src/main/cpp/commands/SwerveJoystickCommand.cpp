#include "commands/SwerveJoystickCommand.h"

// Slew rate limiters to prevent sudden changes in speed
frc::SlewRateLimiter<units::scalar> xSpeedLimit{3 / 1_s};
frc::SlewRateLimiter<units::scalar> ySpeedLimit{3 / 1_s};
frc::SlewRateLimiter<units::scalar> turnSpeedLimit{3 / 1_s};

/**
 * Constructor for the SwerveJoystickCommand.
 *
 * @param driveTrain Pointer to the DriveTrain subsystem.
 * @param xSpeedFunction Function to get the desired x-axis speed.
 * @param ySpeedFunction Function to get the desired y-axis speed.
 * @param turnSpeedFunction Function to get the desired turning speed.
 * @param fieldOrientedFunction Function to determine if driving is field-oriented.
 */
SwerveJoystickCommand::SwerveJoystickCommand(DriveTrain *driveTrain, std::function<double(void)> xSpeedFunction, std::function<double(void)> ySpeedFunction, std::function<double(void)> turnSpeedFunction, std::function<bool(void)> fieldOrientedFunction)
    : driveTrain(driveTrain),
      xSpeedFunction(xSpeedFunction),
      ySpeedFunction(ySpeedFunction),
      turnSpeedFunction(turnSpeedFunction),
      fieldOrientedFunction(fieldOrientedFunction)
{
  AddRequirements(driveTrain);
}

void SwerveJoystickCommand::Initialize() {}

/**
 * Executes the command. Called repeatedly when the command is scheduled to run.
 * Controls the robot's movement based on joystick input and field orientation.
 */
void SwerveJoystickCommand::Execute()
{
  // Get joystick inputs, apply deadband and slew rate limiting, and scale to max speeds
  auto xSpeed = (xSpeedLimit.Calculate(frc::ApplyDeadband(xSpeedFunction(), OIConstants::kDeadband)) * DriveConstants::kTeleDriveMaxSpeedMetersPerSecond);
  auto ySpeed = (ySpeedLimit.Calculate(frc::ApplyDeadband(ySpeedFunction(), OIConstants::kDeadband)) * DriveConstants::kTeleDriveMaxSpeedMetersPerSecond);
  auto turnSpeed = (turnSpeedLimit.Calculate(frc::ApplyDeadband(turnSpeedFunction(), OIConstants::kDeadband)) * DriveConstants::kTeleDriveMaxAngularSpeedRadiansPerSecond);

  // Determine if driving is field-oriented or robot-oriented
  if (fieldOrientedFunction())
    driveTrain->DriveFieldRelative(frc::ChassisSpeeds(xSpeed, ySpeed, turnSpeed));
  else
    driveTrain->DriveRobotRelative(frc::ChassisSpeeds(xSpeed, ySpeed, turnSpeed));
}

/**
 * Stop all swerve module when the command ends.
 * @param interrupted Whether the command was interrupted or not.
 */
void SwerveJoystickCommand::End(bool interrupted)
{
  driveTrain->StopModules();
}

bool SwerveJoystickCommand::IsFinished()
{
  return false;
}
