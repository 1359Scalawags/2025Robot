package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class NudgePulleyHeight extends Command {
    private ArmSubsystem subsystem;
    private NudgeDirection direction;

    public enum NudgeDirection {
        up,
        down
    }

    public NudgePulleyHeight(ArmSubsystem subsystem, NudgeDirection direction) {
        this.subsystem = subsystem;
        this.direction = direction;
    }

    @Override
    public void initialize() {
        double nudgeAmount = Constants.ArmSubsystem.Pulley.kPulleyNudgeUpDistance;
        if(direction == NudgeDirection.down) {
            nudgeAmount = -nudgeAmount;
        }
        subsystem.nudgePulleyMotorPosition(nudgeAmount);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
