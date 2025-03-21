// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands.Testing;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class Rotate extends Command {

    /**
     * An enumeration of compass directions where North equates to Forward
     */
    public enum RotateDirection {
        CCW,
        CW
    }

    private RotateDirection direction;
    private SwerveSubsystem subsystem;
    private Translation2d translation;
    private double rotation;
    private boolean fieldRelative;
 
    /**
     * Rotate the robot without translation in a robot centric manner
     * @param subsystem The swerve drive subsystem
     * @param direction The direction to rotate in
     */
    public Rotate(SwerveSubsystem subsystem, RotateDirection direction) {
        this(subsystem, direction, false);
    }

    /**
     * Rotate the robot
     * @param subsystem The swerve drive subsystem
     * @param direction The direction to rotate in
     * @param fieldRelative Whether to be field relative or not
     */
    public Rotate(SwerveSubsystem subsystem, RotateDirection direction, boolean fieldRelative) {
        this.direction = direction;
        this.subsystem = subsystem;
        this.translation = new Translation2d(0,0);
        this.direction = direction;
        this.rotation = getRotation(this.direction);
        this.fieldRelative = fieldRelative;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.drive(this.translation, this.rotation, this.fieldRelative);
    }

    /**
     * Convert rotation direction to a number
     * @param direction
     * @return angular speed in specified direction
     */
    public static double getRotation(RotateDirection direction) {
        double omega;
        //NOTE: counter-clockwise is positive in drive command
        switch(direction) {
            case CCW:
                omega = 1; 
                break;
            case CW:
                omega = -1;
                break;
            default:
                omega = 0;
        }
        return omega * Constants.Testing.kSwerveRotateSpeed;
    }

    @Override
    public boolean isFinished() {
        //NOTE: this command never ends and must be canceled explicitly
        return false;   
    }

}
