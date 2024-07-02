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
  return pathplanner::PathPlannerAuto("Example Auto").ToPtr();
}