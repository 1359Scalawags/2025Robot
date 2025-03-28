// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TestingCommands;

import frc.robot.subsystems.TestElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class InitializeTestElevatorEncoders extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TestElevatorSubsystem m_subsystem;


  public InitializeTestElevatorEncoders(TestElevatorSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_subsystem.initializeEncoders();  
  }

  @Override
  public void execute() {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
