package frc.robot.commands.Tuning;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.extensions.SparkMaxPIDTuner;

public class ApplyTunerValues extends Command {
    SparkMaxPIDTuner tuner;

    public ApplyTunerValues(SparkMaxPIDTuner tuner) {
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
