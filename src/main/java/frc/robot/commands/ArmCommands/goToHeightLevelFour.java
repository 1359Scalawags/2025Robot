
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;



public class goToHeightLevelFour extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public goToHeightLevelFour(ArmSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }


  @Override
  public void initialize() {}

 
  @Override
  public void execute() {
    m_subsystem.goToHeightL4();
    m_subsystem.goToArmL4();
  }


  @Override
  public void end(boolean interrupted) {}



  
  @Override
  public boolean isFinished() {
    if (m_subsystem.getArmPosition().isNear(Constants.ArmSubsystem.Positions.klevel4)) {
      return true;
    } else {
      return false;
    }

  }
}
