package frc.robot.commands.Tuning;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.extensions.SparkMaxPIDTunerBase;

public class ApplyTunerValues extends Command {
    SparkMaxPIDTunerBase tuner;

    public ApplyTunerValues(SparkMaxPIDTunerBase tuner) {
        this.tuner = tuner;
    }

    @Override
    public void initialize() {
        tuner.applyTunerValues();
        System.out.println("Applying Tuner Values");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
