#include "commands/SwerveJoystickCommand.h"

frc::SlewRateLimiter<units::scalar> xSpeedLimit{3 / 1_s};
frc::SlewRateLimiter<units::scalar> ySpeedLimit{3 / 1_s};
frc::SlewRateLimiter<units::scalar> turnSpeedLimit{3 / 1_s};

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

void SwerveJoystickCommand::Execute()
{

  auto xSpeed = (xSpeedLimit.Calculate(frc::ApplyDeadband(xSpeedFunction(), OIConstants::kDeadband)) * DriveConstants::kTeleDriveMaxSpeedMetersPerSecond);
  auto ySpeed = (ySpeedLimit.Calculate(frc::ApplyDeadband(ySpeedFunction(), OIConstants::kDeadband)) * DriveConstants::kTeleDriveMaxSpeedMetersPerSecond);
  auto turnSpeed = (turnSpeedLimit.Calculate(frc::ApplyDeadband(turnSpeedFunction(), OIConstants::kDeadband)) * DriveConstants::kTeleDriveMaxAngularSpeedRadiansPerSecond);

  frc::ChassisSpeeds chassisSpeeds;

  if (fieldOrientedFunction())
    chassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, driveTrain->GetRotation2d());
  else
    chassisSpeeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(xSpeed, ySpeed, turnSpeed, driveTrain->GetRotation2d());

  auto states = DriveConstants::kDriveKinematics.ToSwerveModuleStates(chassisSpeeds);
  driveTrain->SetModuleStates(states);
}

void SwerveJoystickCommand::End(bool interrupted)
{
  driveTrain->StopModules();
}

bool SwerveJoystickCommand::IsFinished()
{
  return false;
}
