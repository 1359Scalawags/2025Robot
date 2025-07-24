// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands.Functionality;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;


public class InitilizeClimberEncoders extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem m_subsystem;
  private Timer waitTimer;
  private boolean hasfinished = false;

  public InitilizeClimberEncoders(ClimberSubsystem subsystem) {
    m_subsystem = subsystem;
    waitTimer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }


  @Override
  public void initialize() {
    m_subsystem.initializeClimber();
    waitTimer.reset();
    waitTimer.start();
  }

 
  @Override
  public void execute() {
    if(waitTimer.get() > Constants.ClimberSubsystem.kIntializeDelay) {

      hasfinished = true;
    }

  }


  @Override
  public boolean isFinished() {
    return hasfinished;
  }
}
