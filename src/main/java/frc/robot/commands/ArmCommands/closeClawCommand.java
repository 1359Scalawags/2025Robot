
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.ArmCommands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class closeClawCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public closeClawCommand(ArmSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.closeClaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
    //TODO: do we want tolerances?
  public boolean isFinished() {
    return true;
  }
}
