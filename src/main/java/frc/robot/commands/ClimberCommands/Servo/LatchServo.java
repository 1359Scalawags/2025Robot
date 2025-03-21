// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands.Servo;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class LatchServo extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem m_subsystem;
  private Timer safetyTimer;
  private boolean hasFinished = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LatchServo(ClimberSubsystem subsystem) {
    m_subsystem = subsystem;
    safetyTimer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }


  @Override
  public void initialize() {
    safetyTimer.reset();
    safetyTimer.start();
    

      
  }


  @Override
  public void execute() {
    if(m_subsystem.isClimberCommandLocked() == false) {
      if (MathUtil.isNear(Constants.ClimberSubsystem.PositionMotor.kLockingPosition, m_subsystem.getClimberPostion(), 5)) {
        m_subsystem.latchCLimber();   
        hasFinished = true; 
      }      
    }


  }


  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    if(m_subsystem.isClimberCommandLocked() == true) {
      return true;
    }
    // stop a stuck servo to avoid burnout
    if(safetyTimer.get() > Constants.ClimberSubsystem.LatchServo.kNaxActuateTime) {
      m_subsystem.setServoValue(m_subsystem.getServoValue());
      return true;
    } 
    return hasFinished;
  }
}
