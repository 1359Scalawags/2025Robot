// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands.Movment;

import frc.robot.subsystems.ClimberSubsystem;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class MoveClimber extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem m_subsystem;
  private final DoubleSupplier assistYSupplier;


  public MoveClimber(ClimberSubsystem subsystem, DoubleSupplier assistY) {
    m_subsystem = subsystem;
    assistYSupplier = assistY;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }


  @Override
  public void initialize() {
   
  }


  @Override
  public void execute() {  
    if(m_subsystem.isClimberCommandLocked() == false ) {
     m_subsystem.changeClimberPosition(assistYSupplier.getAsDouble()*Constants.ClimberSubsystem.PositionMotor.kMaxJoystickSpeed);
  }
  }


  @Override
  public boolean isFinished() {
    return true;
  }
}
