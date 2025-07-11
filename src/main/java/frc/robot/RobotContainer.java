// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.ArmCommands.ZeroClaw;
import frc.robot.commands.ArmCommands.ZeroPulley;
import frc.robot.commands.ArmCommands.autonomousOpenCLaw;
import frc.robot.commands.ArmCommands.InitilizeArmEncoders;
import frc.robot.commands.ArmCommands.closeClawCommand;
import frc.robot.commands.ArmCommands.goToHeightHome;
import frc.robot.commands.ArmCommands.goToHeightHumanStation;
import frc.robot.commands.ArmCommands.goToHeightLevelFour;
import frc.robot.commands.ArmCommands.goToHeightLevelThree;
import frc.robot.commands.ArmCommands.goToHeightLevelTwo;
import frc.robot.commands.ArmCommands.goToHightGround;
import frc.robot.commands.ArmCommands.openClawCommand;
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
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import java.io.File;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // TODO: we need to intialize the arm.
  // The robot's subsystems and commands are defined here

  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "YAGSLConfigJSON/Pearl"));
  // TODO: This needs to be enabled when ready. Also fix initializeArm() below and
  // uncomment calls in Robot.java
  private final ArmSubsystem m_ArmSubsystem;
  private final ClimberSubsystem m_ClimberSubsystem;
  // private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private final CommandJoystick m_DriverJoystick = new CommandJoystick(Constants.Operator.DriverJoystick.kPort);
  private final CommandJoystick m_AssistantJoystick = new CommandJoystick(Constants.Operator.AssistJoystick.kPort);

  SendableChooser<Command> autoChooser;
  SendableChooser<Command> pipelineChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
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
    autoChooser = AutoBuilder.buildAutoChooser(); // This will populate all the autos in the project.
    pipelineChooser = new SendableChooser<Command>();

    SmartDashboard.putData("Pipeline Chooser", pipelineChooser);
    SmartDashboard.putData("Auto Chooser ", autoChooser);

    // Configure the trigger bindings
    configureBindings();
    setDefaultCommands();

    if (m_ArmSubsystem != null) {
      NamedCommands.registerCommand("moveL2", new goToHeightLevelTwo(m_ArmSubsystem));
      NamedCommands.registerCommand("Open Claw", new autonomousOpenCLaw(m_ArmSubsystem));
    }
  }

  private void setDefaultCommands() {
    m_SwerveSubsystem.setDefaultCommand(
        new AbsoluteFieldDrive(m_SwerveSubsystem,
            this::driverGetForward,
            this::driverGetRight,
            this::driverGetZ,
            this::driverGetThrottle,
            m_SwerveSubsystem::getFeildCentric,
            false));

    if (m_ClimberSubsystem != null) {
      m_ClimberSubsystem.setDefaultCommand(
          new MoveClimber(m_ClimberSubsystem,
              this::assistantGetY));
    }

    // This is only for testing the pulley motor directly. It will interact badly
    // with the climber when it is unlocked
    // m_ArmSubsystem.setDefaultCommand(new MovePulleyWithJoystick(m_ArmSubsystem,
    // this::assistantGetY));

  }

  // TODO: Are deadbands implemented for joysticks?
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

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    // new JoystickButton(assistantJoystick,
    // Constants.Operator.AssistJoystick.deployClimberButton)
    // .onTrue(new DeployClimber(m_ClimberSubsystem));
    // }

    // Bindin Arm Commands
    if(m_ArmSubsystem != null) {
      m_AssistantJoystick.button(1).onFalse(new closeClawCommand(m_ArmSubsystem));
      m_AssistantJoystick.button(1).onTrue(new openClawCommand(m_ArmSubsystem));

      m_AssistantJoystick.button(8).onTrue(new goToHeightHumanStation(m_ArmSubsystem));
      m_AssistantJoystick.button(10).onTrue(new goToHeightLevelFour(m_ArmSubsystem));
      m_AssistantJoystick.button(7).onTrue(new goToHeightLevelThree(m_ArmSubsystem));
      m_AssistantJoystick.button(6).onTrue(new goToHeightLevelTwo(m_ArmSubsystem));
      m_AssistantJoystick.button(9).onTrue(new goToHightGround(m_ArmSubsystem));      
    }


    // Binding Climber Commands
    // can we make this simpler (sequential command)?
    // proposed solution
    // if (useClimber == false) {
    // -
    // - insert all ARM commands
    // - insert the deufault command for climber movement
    // } else {
    // - all of the CLIMBER commands
    // - climber default movment command
    // }
    // then just make a button that flips that variable. talk to drive team about
    // it.

    if (m_ClimberSubsystem != null) {
      m_AssistantJoystick.button(16).onTrue(new UnlockClimberSubsystem(m_ClimberSubsystem));

      //// m_AssistantJoystick.button(14).onTrue(new
      //// RetractClimber(m_ClimberSubsystem));

      m_AssistantJoystick.button(15).onTrue(new LockClimberBar(m_ClimberSubsystem));
      m_AssistantJoystick.button(4).onTrue(new UnLockClimberBar(m_ClimberSubsystem));

      m_AssistantJoystick.button(12).onTrue(new LatchServo(m_ClimberSubsystem));
      m_AssistantJoystick.button(14).onTrue(new UnLatchServo(m_ClimberSubsystem));

      m_AssistantJoystick.button(13).onTrue(new LockingPosition(m_ClimberSubsystem));
      m_AssistantJoystick.button(11).onTrue(new LockedPosition(m_ClimberSubsystem));

      m_AssistantJoystick.button(3).onTrue(new DeployClimber(m_ClimberSubsystem));
    }

    // //TODO: Make a sequential command to lock the arm motor.

    // m_AssistantJoystick.button(1).onTrue(Commands.sequence(new
    // LockingPosition(m_ClimberSubsystem),
    // new LatchServo(m_ClimberSubsystem),
    // new WaitCommand(1.5),
    // new LockedPosition(m_ClimberSubsystem)));

    m_DriverJoystick.button(1).onTrue(new ZeroGyroCommand(m_SwerveSubsystem));
    m_DriverJoystick.button(2).onTrue(new FieldCentricCommand(m_SwerveSubsystem));
    m_DriverJoystick.button(3).onTrue(new RobotCentricCommand(m_SwerveSubsystem));

    // if (Constants.kDebug) {
    // new JoystickButton(m_DriverJoystick,
    // Constants.Operator.DriverJoystick.driveForwardButton)
    // .onTrue(new DriveForwardCommand(m_SwerveSubsystem));

    // new JoystickButton(m_DriverJoystick,
    // Constants.Operator.DriverJoystick.driveRightButton)
    // .onTrue(new DriveRightCommand(m_SwerveSubsystem));

    // new JoystickButton(m_DriverJoystick,
    // Constants.Operator.DriverJoystick.rotateCCWButton)
    // .onTrue(new RotateCCWCommand(m_SwerveSubsystem));
    // }
    // }

    // XXX: These are for testing only. They should be commented after testing is
    // completed.
    m_DriverJoystick.button(12).whileTrue(new MoveCardinal(m_SwerveSubsystem, CardinalDirection.N));
    m_DriverJoystick.button(15).whileTrue(new MoveCardinal(m_SwerveSubsystem, CardinalDirection.S));
    m_DriverJoystick.button(16).whileTrue(new MoveCardinal(m_SwerveSubsystem, CardinalDirection.E));
    m_DriverJoystick.button(13).whileTrue(new MoveCardinal(m_SwerveSubsystem, CardinalDirection.W));
    m_DriverJoystick.button(8).whileTrue(new MoveCardinal(m_SwerveSubsystem, CardinalDirection.SE));
    m_DriverJoystick.button(11).whileTrue(new Rotate(m_SwerveSubsystem, RotateDirection.CW));
    m_DriverJoystick.button(5).whileTrue(new Rotate(m_SwerveSubsystem, RotateDirection.CCW));

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

    // return homePulley;

    return Commands.sequence(zeroPulley, new WaitCommand(0.5), homePulley);
    // return initializeArm.Commands.sequence(homeClaw.andThen(homePulley));
  }

  public Command homeClaw() {
    if(m_ArmSubsystem == null) {
      return new WaitCommand(0.01);
    }
    Command homeclaw = new ZeroClaw(m_ArmSubsystem);
      return homeclaw;
  }

  // public Command intializeJustTheArm() {
  // return new InitilizeArm(m_ArmSubsystem);
  // }

  // public Command homeTheClaw() {
  // return new HomeTheClaw(m_ArmSubsystem);
  // }

  // public Command homeThePulley() {
  // return new HomeThePulley(m_ArmSubsystem);
  // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
