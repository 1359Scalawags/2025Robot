package frc.robot.commands.Tuning;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.extensions.ISparkMaxTuner;
import frc.robot.extensions.SparkMaxPIDTunerBase;

public class Stop_Motor extends Command {
    ISparkMaxTuner tuner;

    public Stop_Motor(ISparkMaxTuner tuner) {
        this.tuner = tuner;
    }

    @Override
    public void initialize() {
        tuner.stopMotor();
        System.out.println("Stopping Tuner Motor");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}