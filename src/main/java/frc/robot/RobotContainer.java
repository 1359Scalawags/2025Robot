// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TestSubsystem;
import frc.robot.commands.ArmCommands.ZeroPulley;
import frc.robot.commands.ArmCommands.InitilizeArmEncoders;
import frc.robot.commands.ArmCommands.goToHeightHome;
import frc.robot.commands.ArmCommands.goToHeightHumanStation;
import frc.robot.commands.ArmCommands.goToHeightLevelFour;
import frc.robot.commands.ArmCommands.goToHeightLevelThree;
import frc.robot.commands.ArmCommands.goToHeightLevelTwo;
import frc.robot.commands.ArmCommands.goToHightGround;
import frc.robot.commands.ClimberCommands.Functionality.InitilizeClimberEncoders;
import frc.robot.commands.ClimberCommands.Functionality.LockClimberSubsystem;
import frc.robot.commands.ClimberCommands.Functionality.UnlockClimberSubsystem;
import frc.robot.commands.ClimberCommands.Movment.LockedPosition;
import frc.robot.commands.ClimberCommands.Movment.LockingPosition;
import frc.robot.commands.ClimberCommands.Movment.DeployClimber;
import frc.robot.commands.ClimberCommands.Movment.LockClimberBar;
import frc.robot.commands.ClimberCommands.Movment.MoveClimber;
import frc.robot.commands.ClimberCommands.Movment.RetractClimber;
import frc.robot.commands.ClimberCommands.Movment.UnLockClimberBar;
import frc.robot.commands.ClimberCommands.Servo.LatchServo;
import frc.robot.commands.ClimberCommands.Servo.UnLatchServo;
import frc.robot.commands.SwerveCommands.AbsoluteFieldDrive;
import frc.robot.commands.SwerveCommands.FieldCentricCommand;
import frc.robot.commands.SwerveCommands.RobotCentricCommand;
import frc.robot.commands.SwerveCommands.ZeroGyroCommand;
import frc.robot.commands.SwerveCommands.Testing.MoveCardinal;
import frc.robot.commands.SwerveCommands.Testing.MoveCardinal.CardinalDirection;
import frc.robot.commands.SwerveCommands.Testing.Rotate;
import frc.robot.commands.SwerveCommands.Testing.Rotate.RotateDirection;
import frc.robot.commands.TestingCommands.ControlTestMotorWithJoystick;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import java.io.File;
import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  // The robot's subsystems and commands are defined here

  private final SwerveSubsystem m_SwerveSubsystem;

  // uncomment calls in Robot.java
  private final ArmSubsystem m_ArmSubsystem;
  private final ClimberSubsystem m_ClimberSubsystem;
  private final TestSubsystem m_TestSubsystem;
  // private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private final CommandJoystick m_DriverJoystick = new CommandJoystick(Constants.Operator.DriverJoystick.kPort);
  private final CommandJoystick m_AssistantJoystick = new CommandJoystick(Constants.Operator.AssistJoystick.kPort);
  private final CommandJoystick m_LogitechAttack = new CommandJoystick(Constants.Operator.TestJoystick.kPort);

  SendableChooser<Command> autoChooser;
  SendableChooser<Command> pipelineChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    DogLogOptions logOptions = new DogLogOptions().withCaptureConsole(true).withCaptureDs(true).withLogExtras(true);
    DogLog.setOptions(logOptions);

    if(Constants.TestSystem.kEnabled) {
      m_TestSubsystem = new TestSubsystem();
    } else {
      m_TestSubsystem = null;
    }

    if(Constants.SwerveSubsystem.kEnabled) {
      m_SwerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "YAGSLConfigJSON/Pearl")); 
    } else {
      m_SwerveSubsystem = null;
    }

    if (Constants.ClimberSubsystem.kEnabled) {
      m_ClimberSubsystem = new ClimberSubsystem();
    } else {
      m_ClimberSubsystem = null;
    }
    if (Constants.ArmSubsystem.kEnabled) {
      m_ArmSubsystem = new ArmSubsystem();
    } else {
      m_ArmSubsystem = null;
    }

    if(m_SwerveSubsystem != null) {
      autoChooser = AutoBuilder.buildAutoChooser(); // This will populate all the autos in the project.      
    } else {
      autoChooser = new SendableChooser<Command>();
    }
    SmartDashboard.putData("Auto Chooser ", autoChooser);    

    pipelineChooser = new SendableChooser<Command>();
    SmartDashboard.putData("Pipeline Chooser", pipelineChooser);


    // Configure the trigger bindings
    configureBindings();
    setDefaultCommands();

  }

  private void setDefaultCommands() {
    if(m_SwerveSubsystem != null) {
      m_SwerveSubsystem.setDefaultCommand(
          new AbsoluteFieldDrive(
              m_SwerveSubsystem,
              this::driverGetForward,
              this::driverGetRight,
              this::driverGetZ,
              this::driverGetThrottle,
              m_SwerveSubsystem::getFeildCentric,
              false));      
    }

    if (m_ClimberSubsystem != null) {
      m_ClimberSubsystem.setDefaultCommand(
          new MoveClimber(
              m_ClimberSubsystem,
              this::assistantGetY));
    }

    if(m_TestSubsystem != null) {
      m_TestSubsystem.setDefaultCommand(
          new ControlTestMotorWithJoystick(
              m_TestSubsystem, 
              this::testGetForward));
    }

  }

  // Configure remote movements
  public double assistantGetY() {
    return -m_AssistantJoystick.getY();
  }

  public double assistantGetX() {
    return -m_AssistantJoystick.getX();
  }

  public double assistantGetZ() {
    return -m_AssistantJoystick.getZ();
  }

  public double driverGetRight() {
    return -m_DriverJoystick.getX();
  }

  public double driverGetForward() {
    return -m_DriverJoystick.getY();
  }

  public double driverGetZ() {
    return m_DriverJoystick.getZ();
  }

  public double driverGetThrottle() {
    return m_DriverJoystick.getThrottle();
  }

  public double testGetRight() {
    return -m_LogitechAttack.getX();
  }

  public double testGetForward() {
    return -m_LogitechAttack.getY();
  }

  public double testGetZ() {
    return m_LogitechAttack.getZ();
  }

  public double testGetThrottle() {
    return m_LogitechAttack.getThrottle();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void configureBindings() {

    if(m_ArmSubsystem != null) {
      m_AssistantJoystick.button(8).onTrue(new goToHeightHumanStation(m_ArmSubsystem));
      m_AssistantJoystick.button(10).onTrue(new goToHeightLevelFour(m_ArmSubsystem));
      m_AssistantJoystick.button(7).onTrue(new goToHeightLevelThree(m_ArmSubsystem));
      m_AssistantJoystick.button(6).onTrue(new goToHeightLevelTwo(m_ArmSubsystem));
      m_AssistantJoystick.button(9).onTrue(new goToHightGround(m_ArmSubsystem));      
    }

    if (m_ClimberSubsystem != null) {
      m_AssistantJoystick.button(16).onTrue(new UnlockClimberSubsystem(m_ClimberSubsystem));

      m_AssistantJoystick.button(15).onTrue(new LockClimberBar(m_ClimberSubsystem));
      m_AssistantJoystick.button(4).onTrue(new UnLockClimberBar(m_ClimberSubsystem));

      m_AssistantJoystick.button(12).onTrue(new LatchServo(m_ClimberSubsystem));
      m_AssistantJoystick.button(14).onTrue(new UnLatchServo(m_ClimberSubsystem));

      m_AssistantJoystick.button(13).onTrue(new LockingPosition(m_ClimberSubsystem));
      m_AssistantJoystick.button(11).onTrue(new LockedPosition(m_ClimberSubsystem));

      m_AssistantJoystick.button(3).onTrue(new DeployClimber(m_ClimberSubsystem));
    }

    if(m_SwerveSubsystem != null) {
      m_DriverJoystick.button(1).onTrue(new ZeroGyroCommand(m_SwerveSubsystem));
      m_DriverJoystick.button(2).onTrue(new FieldCentricCommand(m_SwerveSubsystem));
      m_DriverJoystick.button(3).onTrue(new RobotCentricCommand(m_SwerveSubsystem));
      m_DriverJoystick.button(12).whileTrue(new MoveCardinal(m_SwerveSubsystem, CardinalDirection.N));
      m_DriverJoystick.button(15).whileTrue(new MoveCardinal(m_SwerveSubsystem, CardinalDirection.S));
      m_DriverJoystick.button(16).whileTrue(new MoveCardinal(m_SwerveSubsystem, CardinalDirection.E));
      m_DriverJoystick.button(13).whileTrue(new MoveCardinal(m_SwerveSubsystem, CardinalDirection.W));
      m_DriverJoystick.button(8).whileTrue(new MoveCardinal(m_SwerveSubsystem, CardinalDirection.SE));
      m_DriverJoystick.button(11).whileTrue(new Rotate(m_SwerveSubsystem, RotateDirection.CW));
      m_DriverJoystick.button(5).whileTrue(new Rotate(m_SwerveSubsystem, RotateDirection.CCW));      
    }
 

  }

  public Command lockClimberSubsystemWhenDisabled() {
    if (m_ClimberSubsystem == null) {
      return new WaitCommand(0.01);      
    }
    return new LockClimberSubsystem(m_ClimberSubsystem);
  }

  public Command initializeClimberEncoders() {
    if (m_ClimberSubsystem == null) {
      return new WaitCommand(0.01);      
    }
    return new InitilizeClimberEncoders(m_ClimberSubsystem).ignoringDisable(true);
  }

  public Command initializeClimberPosition() {
    if (m_ClimberSubsystem == null) {
      return new WaitCommand(0.01);      
    }
    return new RetractClimber(m_ClimberSubsystem);
  }

  public Command intializeArmEncoders() {
    if(m_ArmSubsystem == null) {
      return new WaitCommand(0.01);  
    }
    Command initializeArmEncoders = new InitilizeArmEncoders(m_ArmSubsystem);
    return initializeArmEncoders.ignoringDisable(true);
  }

  public Command intializeTheArm() {
    if(m_ArmSubsystem == null) {
      return new WaitCommand(0.01);  
    }
    Command zeroPulley = new ZeroPulley(m_ArmSubsystem);
    Command homePulley = new goToHeightHome(m_ArmSubsystem);
    return zeroPulley;
    //return Commands.sequence(zeroPulley, new WaitCommand(0.5), homePulley);
  }

  public ArrayList<Runnable> getSimulationPeriodicMethods() {
    ArrayList<Runnable> simPeriodicList = new ArrayList<Runnable>();
    if(m_TestSubsystem != null) {
      simPeriodicList.add(m_TestSubsystem::simulationPeriodic);
    }
    return simPeriodicList;
  }
  // public Runnable getArmFastSimPeriodic() {
  //   return m_ArmSubsystem::fastSimulationPeriodic;
  // }


}
