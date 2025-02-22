// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class LatchServo extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem m_subsystem;
  private Timer safetyTimer;

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

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    safetyTimer.reset();
    safetyTimer.start();
    m_subsystem.latchCLimber();
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
    
    // stop a stuck servo to avoid burnout
    if(safetyTimer.get() > Constants.ClimberSubsystem.LatchServo.kNaxActuateTime) {
      m_subsystem.setServoValue(m_subsystem.getServoValue());
      return true;
    }

    if(MathUtil.isNear(Constants.ClimberSubsystem.LatchServo.latchedValue, m_subsystem.getServoValue(), 5)){
      return true; 
    } else {
      return false;
    }
  }
}
