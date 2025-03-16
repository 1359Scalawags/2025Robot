package frc.robot.commands.Tuning;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.extensions.ISparkMaxTuner;
import frc.robot.extensions.SparkMaxPIDTunerBase;

public class Apply_Values extends Command {
    ISparkMaxTuner tuner;

    public Apply_Values(ISparkMaxTuner tuner) {
        this.tuner = tuner;
    }

    @Override
    public void initialize() {
        tuner.applyTunerValues();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
