// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;


public class InitilizeArmEncoders extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_subsystem;
  private Timer waitTimer;
  private boolean hasfinished = false;


  public InitilizeArmEncoders(ArmSubsystem subsystem) {
    m_subsystem = subsystem;
    waitTimer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    waitTimer.reset();
    waitTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(waitTimer.get() > Constants.ArmSubsystem.kIntializeDelay) {
      m_subsystem.initializeArm();   
      hasfinished = true;   
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasfinished;
  }
}
