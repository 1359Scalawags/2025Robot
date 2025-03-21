// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class FieldCentricCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem m_subsystem;

 //private boolean fieldCentric = true;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FieldCentricCommand(SwerveSubsystem subsystem) {
    m_subsystem = subsystem; 
    //fieldCentric = state; 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }




  @Override
  public void initialize() {
    m_subsystem.SetfeildCentric(true);
   
  }


  @Override
  public void execute() {
    //m_subsystem.toggleFeildCentric();
  }


  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return true;
  }
}