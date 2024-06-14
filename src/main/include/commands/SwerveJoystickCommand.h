#pragma once

#include <Constants.h>
#include "subsystems/DriveTrain.h"

#include <frc/filter/SlewRateLimiter.h>
#include <frc/MathUtil.h>

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <functional>

class SwerveJoystickCommand
    : public frc2::CommandHelper<frc2::Command, SwerveJoystickCommand>
{
public:
    SwerveJoystickCommand(DriveTrain *driveTrain, std::function<double(void)> xSpeedFunction, std::function<double(void)> ySpeedFunction, std::function<double(void)> turnSpeedFunction, std::function<bool(void)> fieldOrientedFunction);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    DriveTrain *driveTrain;
    std::function<double(void)> xSpeedFunction, ySpeedFunction;
    std::function<double(void)> turnSpeedFunction;
    std::function<bool(void)> fieldOrientedFunction;
};