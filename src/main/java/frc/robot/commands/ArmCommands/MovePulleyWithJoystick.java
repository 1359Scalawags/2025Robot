// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;


public class MovePulleyWithJoystick extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_subsystem;
  private DoubleSupplier joystickSupplier;

  /**
   * Creates a new MovePulleyWithJoystick.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MovePulleyWithJoystick(ArmSubsystem subsystem, DoubleSupplier joystickSupplier) {
    m_subsystem = subsystem;
    this.joystickSupplier = joystickSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double current = ArmSubsystem.getPulleyHeight();
    double target = current + joystickSupplier.getAsDouble();
    m_subsystem.goToPulleyMotorPosition(target);
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return true;
 }
}
