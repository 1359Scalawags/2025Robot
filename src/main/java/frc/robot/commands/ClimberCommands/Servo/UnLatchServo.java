// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands.Servo;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;


public class UnLatchServo extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem m_subsystem;
  private Timer safetyTimer;

  public UnLatchServo(ClimberSubsystem subsystem) {
    m_subsystem = subsystem;
    safetyTimer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }


  @Override
  public void initialize() {
    safetyTimer.reset();
    safetyTimer.start();
    m_subsystem.unLatchCLimber();  
  }


  @Override
  public void execute() {

  }


  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {

    // stop a stuck servo to avoid burnout
    if(safetyTimer.get() > Constants.ClimberSubsystem.LatchServo.kNaxActuateTime) {
      m_subsystem.setServoValue(m_subsystem.getServoValue());
      return true;
    }

    if(MathUtil.isNear(Constants.ClimberSubsystem.LatchServo.unLatchedValue, m_subsystem.getServoAngle(), 5)){
    return true; 
    } else {
      return false;
    }
  }
}
