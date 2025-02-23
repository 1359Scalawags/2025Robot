package frc.robot.commands.Tuning;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.extensions.SparkMaxPIDTunerBase;

public class ResetTunerValues extends Command {
    SparkMaxPIDTunerBase tuner;

    public ResetTunerValues(SparkMaxPIDTunerBase tuner) {
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
