package frc.robot.commands.Tuning;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.extensions.SparkMaxPIDTunerBase;

public class Reset_Values extends Command {
    SparkMaxPIDTunerBase tuner;

    public Reset_Values(SparkMaxPIDTunerBase tuner) {
        this.tuner = tuner;
    }

    @Override
    public void initialize() {
        tuner.resetTunerValues();
        System.out.println("Resetting Tuner Values");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
