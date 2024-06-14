// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

// Initialize all of your commands and subsystems here
RobotContainer::RobotContainer()
{
  // Initialize the default command for the drive train
  m_driveTrain.SetDefaultCommand(
      SwerveJoystickCommand(
          &m_driveTrain,
          // Functions to get joystick axis values for driving
          [this]()
          { return -m_driverController.GetRawAxis(OIConstants::kDriverYAxis); },
          [this]()
          { return m_driverController.GetRawAxis(OIConstants::kDriverXAxis); },
          [this]()
          { return m_driverController.GetRawAxis(OIConstants::kDriverRotAxis); },
          // Function to determine if field-oriented driving is enabled
          [this]()
          { return !m_driverController.GetRawButton(OIConstants::kDriverFieldOrientedButtonIndex); }));
  // Configure the button bindings
  ConfigureBindings();
}

/**
 * Configures button bindings for the robot's controllers.
 */
void RobotContainer::ConfigureBindings()
{
  // Configure a button (B button) to zero the gyro heading
  m_driverController.B().OnTrue(frc2::InstantCommand([this]()
                                                     { return m_driveTrain.ZeroHeading(); })
                                    .ToPtr());
}

/**
 * Gets the autonomous command to run during autonomous mode. *
 * @return The autonomous command to execute.
 */
frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
  // Configure the trajectory for autonomous movement
  frc::TrajectoryConfig trajectoryConfig(AutoConstants::kMaxSpeedMetersPerSecond, AutoConstants::kMaxAccelerationMetersPerSecondSquared);
  trajectoryConfig.SetKinematics(DriveConstants::kDriveKinematics);

  frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      frc::Pose2d(0_m, 0_m, 0_deg),                                  // Start pose
      {frc::Translation2d(1_m, 0_m), frc::Translation2d(1_m, -1_m)}, // Waypoints
      frc::Pose2d(2_m, -1_m, 180_deg),                               // End Pose
      trajectoryConfig);

  // Configure PID controllers for position and heading control
  frc::PIDController xController(AutoConstants::kPXController, 0, 0);
  frc::PIDController yController(AutoConstants::kPYController, 0, 0);
  frc::ProfiledPIDController<units::radians> thetaController(AutoConstants::kPThetaController, 0.0, 0.0, AutoConstants::kThetaControllerConstraints);

  thetaController.EnableContinuousInput(-units::radian_t(std::numbers::pi), units::radian_t(std::numbers::pi));

  // Create the swerve controller command to follow the trajectory
  frc2::CommandPtr swerveControllerCommand = frc2::SwerveControllerCommand<4>(
                                                 trajectory,
                                                 [this]()
                                                 { return m_driveTrain.GetPose2d(); },
                                                 DriveConstants::kDriveKinematics,
                                                 xController,
                                                 yController,
                                                 thetaController,
                                                 [this](auto states)
                                                 { m_driveTrain.SetModuleStates(states); },
                                                 {&m_driveTrain})
                                                 .ToPtr();

  // Return a sequence of commands for autonomous execution
  return frc2::cmd::Sequence(
      // Reset odometry to the initial pose of the trajectory
      frc2::InstantCommand([this, initialPose = trajectory.InitialPose()]()
                           { m_driveTrain.ResetOdometry(initialPose); },
                           {&m_driveTrain})
          .ToPtr(),
      // Execute the swerve controller command for trajectory following
      std::move(swerveControllerCommand),
      // Stop all modules after the trajectory is complete
      frc2::InstantCommand([this]()
                           { m_driveTrain.StopModules(); },
                           {&m_driveTrain})
          .ToPtr());
}