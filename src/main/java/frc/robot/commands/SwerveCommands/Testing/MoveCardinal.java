package frc.robot.commands.SwerveCommands.Testing;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveCardinal  extends Command {

    /**
     * An enumeration of compass directions where North equates to Forward
     */
    public enum CardinalDirection {
        N,
        NE,
        E,
        SE,
        S,
        SW,
        W,
        NW
    }

    private CardinalDirection direction;
    private SwerveSubsystem subsystem;
    private Translation2d translation;
    private double rotation;
    private boolean fieldRelative;

    /**
     * Move the robot without rotation in a robot centric manner
     * @param subsystem The swerve drive subsystem
     * @param direction The cardinal direction to move in
     */
    public MoveCardinal(SwerveSubsystem subsystem, CardinalDirection direction) {
        this(subsystem, direction, false);
    }

    /**
     * Move the robot
     * @param subsystem The swerve drive subsystem
     * @param direction The cardinal direction to move in
     * @param fieldRelative Whether to be field relative or not
     */
    public MoveCardinal(SwerveSubsystem subsystem, CardinalDirection direction, boolean fieldRelative) {
        this.direction = direction;
        this.subsystem = subsystem;
        this.rotation = 0;
        this.translation = MoveCardinal.getTranslation(this.direction);
        this.fieldRelative = fieldRelative;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.drive(this.translation, this.rotation, this.fieldRelative);
    }

    /**
     * Converts cardinal direction to x and y translation.
     * @param direction The desired cardinal direction to move
     * @return an (x,y) translation vector
     */
    private static Translation2d getTranslation(CardinalDirection direction) {
        double x;
        double y;

        // x and y value conversions for translations
        int forwardX = 1; // forward
        int leftY = 1; // left
        int backwardX = -1; // backward
        int rightY = -1; // right

        switch(direction) {
            case N:
                x = forwardX;
                y = 0;
                break;
            case NE:
                x = forwardX;
                y = rightY;
                break;
            case E:
                x = 0;
                y = rightY;
                break;
            case SE:
                x = backwardX;
                y = rightY;
                break;
            case S:
                x = backwardX;
                y = 0;
                break;
            case SW:
                x = backwardX;
                y = leftY;    
                break;
            case W:
                x = 0;
                y = leftY;
                break;
            case NW:
                x = forwardX;
                y = leftY;
                break;
            default:
                x = 0;
                y = 0;
        }
        return new Translation2d(x * Constants.Testing.kSwerveTranslateSpeed, y * Constants.Testing.kSwerveTranslateSpeed);
    }

    @Override
    public boolean isFinished() {
        //NOTE: this command never ends and must be canceled explicitly
        return false;   
    }

}
