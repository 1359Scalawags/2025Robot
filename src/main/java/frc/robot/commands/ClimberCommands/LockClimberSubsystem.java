// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

import java.util.concurrent.locks.Lock;

import com.fasterxml.jackson.core.type.TypeReference;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;


public class LockClimberSubsystem extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem m_subsystem;

  public LockClimberSubsystem(ClimberSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override

  public void initialize() {
    m_subsystem.lockClimberSubsystem();

  }

  @Override //This allows the command to runs when disabled.
  public boolean runsWhenDisabled() {
    return true;
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
  public boolean isFinished() {
    return true;
  }
}
