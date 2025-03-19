// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;



/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  UsbCamera armCamera;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    enableLiveWindowInTest(true);
    SmartDashboard.putData(CommandScheduler.getInstance());
    PowerDistribution powerDistribution = new PowerDistribution(20, ModuleType.kRev);
    Shuffleboard.getTab("Arm").add(powerDistribution);
    
    
    DriverStation.silenceJoystickConnectionWarning(true);
    // UsbCamera camera = CameraServer.startAutomaticCapture();
    // camera.setResolution(640, 480);
    // //camera.setResolution(320, 240);
    // camera.setFPS(15);
  }

  @Override
  public void robotInit() {
    super.robotInit();
    CommandScheduler.getInstance().schedule(m_robotContainer.initializeClimberEncoders());
    CommandScheduler.getInstance().schedule(m_robotContainer.intializeArmEncoders());
    // CommandScheduler.getInstance().schedule(m_robotContainer.intializeTheClimber());
    //TODO: Enable this when arm is enabled
    // CommandScheduler.getInstance().schedule(m_robotContainer.intializeJustTheArm());
    // CommandScheduler.getInstance().schedule(m_robotContainer.intializeTheArm());
    // armCamera = CameraServer.startAutomaticCapture(0);
    // // armCamera.isValid();
    // armCamera.setResolution(640, 360);
    // armCamera.setFPS(15);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().schedule(m_robotContainer.lockClimberSubsystemWhenDisabled());
    CommandScheduler.getInstance().schedule(m_robotContainer.intializeArmEncoders());

  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    //XXX: Is this necessary here? If so, also include the arm?
    CommandScheduler.getInstance().schedule(m_robotContainer.initializeClimberEncoders());
    CommandScheduler.getInstance().schedule(m_robotContainer.initializeClimberPosition());    
    CommandScheduler.getInstance().schedule(m_robotContainer.intializeArmEncoders());
    CommandScheduler.getInstance().schedule(m_robotContainer.intializeTheArm());

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //XXX: Is this necessary here? If so, also include the arm?
    CommandScheduler.getInstance().schedule(m_robotContainer.initializeClimberEncoders());
    CommandScheduler.getInstance().schedule(m_robotContainer.initializeClimberPosition());    
    CommandScheduler.getInstance().schedule(m_robotContainer.intializeArmEncoders());
    CommandScheduler.getInstance().schedule(m_robotContainer.intializeTheArm());

    // CommandScheduler.getInstance().schedule(m_robotContainer.intializeJustTheArm());
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    //TODO: Is this necessary here? If so, also include the arm?
    // CommandScheduler.getInstance().schedule(m_robotContainer.initializeClimberEncoders());
    // CommandScheduler.getInstance().schedule(m_robotContainer.intializeArmEncoders());
    // CommandScheduler.getInstance().schedule(m_robotContainer.intializeTheArm());
    // CommandScheduler.getInstance().schedule(m_robotContainer.intializeJustTheArm());
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  //TODO: Is this necessary here? If so, also include the arm?
  CommandScheduler.getInstance().schedule(m_robotContainer.initializeClimberEncoders());
  CommandScheduler.getInstance().schedule(m_robotContainer.initializeClimberPosition());    
  CommandScheduler.getInstance().schedule(m_robotContainer.intializeArmEncoders());
  CommandScheduler.getInstance().schedule(m_robotContainer.intializeTheArm());
  // CommandScheduler.getInstance().schedule(m_robotContainer.intializeJustTheArm());
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
