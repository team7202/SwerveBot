// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer()
{
  // Initialize all of your commands and subsystems here
  m_driveTrain.SetDefaultCommand(SwerveJoystickCommand(
      &m_driveTrain,
      [this]()
      { return -m_driverController.GetRawAxis(OIConstants::kDriverYAxis); },
      [this]()
      { return m_driverController.GetRawAxis(OIConstants::kDriverXAxis); },
      [this]()
      { return m_driverController.GetRawAxis(OIConstants::kDriverRotAxis); },
      [this]()
      { return !m_driverController.GetRawButton(OIConstants::kDriverFieldOrientedButtonIndex); }));
  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
  // Configure your trigger bindings here

  m_driverController.B().OnTrue(frc2::InstantCommand([this]()
                                                     { return m_driveTrain.ZeroHeading(); })
                                    .ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
  frc::TrajectoryConfig trajectoryConfig(AutoConstants::kMaxSpeedMetersPerSecond, AutoConstants::kMaxAccelerationMetersPerSecondSquared);
  trajectoryConfig.SetKinematics(DriveConstants::kDriveKinematics);

  frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      frc::Pose2d(0_m, 0_m, 0_deg),
      {frc::Translation2d(1_m, 0_m),
       frc::Translation2d(1_m, -1_m)},
      frc::Pose2d(2_m, -1_m, 180_deg),
      trajectoryConfig);

  frc::PIDController xController(AutoConstants::kPXController, 0, 0);
  frc::PIDController yController(AutoConstants::kPYController, 0, 0);
  frc::ProfiledPIDController<units::radians> thetaController(AutoConstants::kPThetaController, 0.0, 0.0, AutoConstants::kThetaControllerConstraints);

  thetaController.EnableContinuousInput(-units::radian_t(std::numbers::pi), units::radian_t(std::numbers::pi));

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

  return frc2::cmd::Sequence(
      frc2::InstantCommand([this, initialPose = trajectory.InitialPose()]()
                           { m_driveTrain.ResetOdometry(initialPose); },
                           {&m_driveTrain})
          .ToPtr(),
      std::move(swerveControllerCommand),
      frc2::InstantCommand([this]()
                           { m_driveTrain.StopModules(); },
                           {&m_driveTrain})
          .ToPtr());
}