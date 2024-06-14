// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.h"
#include "commands/SwerveJoystickCommand.h"
#include "subsystems/DriveTrain.h"

#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include <frc2/command/Commands.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/Trigger.h>

#include <units/angle.h>
#include <units/velocity.h>
#include <units/angular_acceleration.h>

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer
{
public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandXboxController m_driverController{
      OIConstants::kDriverControllerPort};

  // The robot's subsystems are defined here...
  DriveTrain m_driveTrain;

  void ConfigureBindings();
};
