// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands.Movment;

import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class LockClimberBar extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem m_subsystem;

  public LockClimberBar(ClimberSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }


  @Override
  public void initialize() {
    m_subsystem.lockClimberBar();
  }


  @Override
  public void execute() {
   
  }


  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    // if(MathUtil.isNear(Constants.ClimberSubsystem.LockingBarMotor.kLockedPosition, m_subsystem.getLockingMotorPosition(), 5)){
    // return true; 
    // } else {
    //   return false;
    // }
    return true;
  }
}
