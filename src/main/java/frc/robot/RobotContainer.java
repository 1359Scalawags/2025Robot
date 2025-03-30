// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.TestArmSubsystem;
import frc.robot.subsystems.TestElevatorSubsystem;
import frc.robot.commands.TestingCommands.MoveElevatorJoystick;
import frc.robot.commands.TestingCommands.InitializeTestArmEncoders;
import frc.robot.commands.TestingCommands.InitializeTestElevatorEncoders;
import frc.robot.commands.TestingCommands.MoveArmJoystick;

import java.util.ArrayList;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class RobotContainer {
  // The robot's subsystems and commands are defined here

  private final TestArmSubsystem m_TestArm;
  private final TestElevatorSubsystem m_TestElevator;
  private final CommandJoystick m_LogitechAttack = new CommandJoystick(Constants.Operator.TestJoystick.kPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    DogLogOptions logOptions = new DogLogOptions().withCaptureConsole(true).withCaptureDs(true).withLogExtras(true);
    DogLog.setOptions(logOptions);

    if(Constants.TestArm.kEnabled) {
      m_TestArm = new TestArmSubsystem();
    } else {
      m_TestArm = null;
    }

    if(Constants.TestElevator.kEnabled) {
      m_TestElevator = new TestElevatorSubsystem(m_TestArm);
    } else {
      m_TestElevator = null;
    }


    // Configure the trigger bindings
    configureBindings();
    setDefaultCommands();

  }

  private void setDefaultCommands() {

    if(m_TestElevator != null) {
      m_TestElevator.setDefaultCommand(
          new MoveElevatorJoystick(
              m_TestElevator, 
              this::testGetForward));
    }

    if(m_TestArm != null) {
      m_TestArm.setDefaultCommand(
          new MoveArmJoystick(
              m_TestArm, 
              this::testGetRight));
    }

  }

  // XXX: a basic implementation of a deadband
  private static double linearDeadband(double raw, double deadband)
  {
      if (Math.abs(raw) < deadband) 
        return 0;
     return Math.signum(raw) * (Math.abs(raw)-deadband) / (1-deadband);
  }

  public double testGetRight() {
    return -m_LogitechAttack.getX();
  }

  public double testGetForward() {
    return linearDeadband(-m_LogitechAttack.getY(), 0.025);
  }

  public double testGetZ() {
    return m_LogitechAttack.getZ();
  }

  public double testGetThrottle() {
    return m_LogitechAttack.getThrottle();
  }

  private void configureBindings() {

  }

  public Command initializeTestSystemEncoders() {
    if(m_TestElevator != null && m_TestArm != null) {
      return Commands.sequence(new InitializeTestElevatorEncoders(m_TestElevator), new InitializeTestArmEncoders(m_TestArm));
    } else if(m_TestElevator != null) {
      return new InitializeTestElevatorEncoders(m_TestElevator);
    } else if(m_TestArm != null) {
      return new InitializeTestArmEncoders(m_TestArm);
    } else {
      return new WaitCommand(0.02);
    }
    
  }

}
