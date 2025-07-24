// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class autonomousOpenCLaw extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_subsystem;

  /**
   * Creates a new AutonomousOpenClaw command.
   *
   * @param subsystem The subsystem used by this command.
   */
  public autonomousOpenCLaw(ArmSubsystem subsystem) {
    m_subsystem = subsystem;
    // addRequirements(subsystem);
  }

  @Override
  public void initialize() {
      m_subsystem.autonomousOpenClaw();
  }

  @Override
  public void execute() {
}

  @Override
  public void end(boolean interrupted) {}

  @Override
    //TODO: Do we want tolerences?
  public boolean isFinished() {
    return true;
  }
}

