// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class ZeroClaw extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_subsystem;
  //private Timer waitTimer;

  public ZeroClaw(ArmSubsystem subsystem) {
    m_subsystem = subsystem;
    // waitTimer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {
    // waitTimer.reset();
    // waitTimer.start();
  }

  @Override
  public void execute() {
    // if(waitTimer.get() > 1.0) {
      double currentPosition = m_subsystem.getClawMotorPosition();
      m_subsystem.goToClawMotorPosition(currentPosition + Constants.ArmSubsystem.Claw.kHomingPositionIncrement, true);      
    // }
  }

  @Override
  public boolean isFinished() {
    if(m_subsystem.isClawAtHome()) {
      return true;
    }
    return false;
  }
}
