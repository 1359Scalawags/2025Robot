package frc.robot.commands.Tuning;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.extensions.SparkMaxPIDTunerBase;

public class StartTunerMotor extends Command {
    SparkMaxPIDTunerBase tuner;

    public StartTunerMotor(SparkMaxPIDTunerBase tuner) {
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
