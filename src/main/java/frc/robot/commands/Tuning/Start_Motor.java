package frc.robot.commands.Tuning;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.extensions.ISparkMaxTuner;
import frc.robot.extensions.SparkMaxPIDTunerBase;

public class Start_Motor extends Command {
    ISparkMaxTuner tuner;

    public Start_Motor(ISparkMaxTuner tuner) {
        this.tuner = tuner;
    }

    @Override
    public void initialize() {
        tuner.startMotor();
        System.out.println("Starting Tuner Motor");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
