
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;


/** An example command that uses an example subsystem. */
public class goToHeightLevelThree extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public goToHeightLevelThree(ArmSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.goToArmL3();
    m_subsystem.goToHeightL3();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  
  @Override
  public boolean isFinished() {
     if (MathUtil.isNear(Constants.ArmSubsystem.kL3Height, m_subsystem.getCalculatedHeight(), Constants.ArmSubsystem.armGoToTolerance) && 
    MathUtil.isNear(Constants.ArmSubsystem.kElbowPosL3, m_subsystem.getElbowMotorPosition(), Constants.ArmSubsystem.armGoToTolerance) &&
    MathUtil.isNear(Constants.ArmSubsystem.kWristPosL3,m_subsystem.getWristMotorPosition(), Constants.ArmSubsystem.armGoToTolerance)) {
      return true;
    } else {
      return false;
    }
  }
}
