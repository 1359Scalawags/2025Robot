// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Tuning;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.extensions.ISparkMaxTuner;

public class Stop_Motor extends Command {
    ISparkMaxTuner tuner;

    public Stop_Motor(ISparkMaxTuner tuner) {
        this.tuner = tuner;
    }

    @Override
    public void initialize() {
        tuner.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}