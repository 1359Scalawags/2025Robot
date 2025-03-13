package frc.robot.commands.Tuning;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.extensions.SparkMaxPIDTunerBase;

public class Stop_Motor extends Command {
    SparkMaxPIDTunerBase tuner;

    public Stop_Motor(SparkMaxPIDTunerBase tuner) {
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