// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands.Movment;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;


public class LockedPosition extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LockedPosition(ClimberSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }


  @Override
  public void initialize() {
    if(m_subsystem.isClimberCommandLocked() == false) {
      m_subsystem.climberLockedPosition();
    }

  }


  @Override
  public void execute() {
    
}


  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() { //TODO: maybe tune the tolerance.
    if(m_subsystem.isClimberCommandLocked() == true) {
      return true;
    }
    if(MathUtil.isNear(Constants.ClimberSubsystem.PositionMotor.kLockedPosition, m_subsystem.getClimberPostion(), 2)){
    return true; 
    } else {
      return false;
    }
  }
}
