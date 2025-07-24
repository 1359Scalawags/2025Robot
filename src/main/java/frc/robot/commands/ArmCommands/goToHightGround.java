
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;


public class goToHightGround extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_subsystem;

  /**
   * Creates a new goToHightGround.
   *
   * @param subsystem The subsystem used by this command.
   */
  public goToHightGround(ArmSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.goToHeightGround();
    m_subsystem.goToArmGround();
  }


  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    if (m_subsystem.getArmPosition().isNear(Constants.ArmSubsystem.Positions.kGround)) {
      return true;
    } else {
      return false;
    }
 }
}