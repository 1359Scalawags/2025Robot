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
    super(Constants.kRobotLoopTime);

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    for(Runnable r : m_robotContainer.getSimulationPeriodicMethods()) {
      //addPeriodic(r, Constants.kSimulationLoopTime);
    }
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
    CommandScheduler.getInstance().schedule(m_robotContainer.initializeTestElevator().ignoringDisable(true));

    try {
      armCamera = CameraServer.startAutomaticCapture(0);
      // armCamera.isValid();
      armCamera.setResolution(640, 360);
      armCamera.setFPS(20);      
    } catch (Exception ex) {
      System.out.println("The USB camera could not be initialized.");
    }


  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }


  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().schedule(m_robotContainer.lockClimberSubsystemWhenDisabled());
    CommandScheduler.getInstance().schedule(m_robotContainer.intializeArmEncoders());

  }

  @Override
  public void disabledPeriodic() {}


  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    CommandScheduler.getInstance().schedule(m_robotContainer.initializeClimberEncoders());
    CommandScheduler.getInstance().schedule(m_robotContainer.initializeClimberPosition());    
    CommandScheduler.getInstance().schedule(m_robotContainer.intializeArmEncoders());
    CommandScheduler.getInstance().schedule(m_robotContainer.intializeTheArm());

  }


  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    CommandScheduler.getInstance().schedule(m_robotContainer.initializeClimberEncoders());
    CommandScheduler.getInstance().schedule(m_robotContainer.initializeClimberPosition());    
    CommandScheduler.getInstance().schedule(m_robotContainer.intializeArmEncoders());
    CommandScheduler.getInstance().schedule(m_robotContainer.intializeTheArm());


  }


  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {
    CommandScheduler.getInstance().schedule(m_robotContainer.initializeClimberEncoders());
    CommandScheduler.getInstance().schedule(m_robotContainer.initializeClimberPosition());    
    CommandScheduler.getInstance().schedule(m_robotContainer.intializeArmEncoders());
    CommandScheduler.getInstance().schedule(m_robotContainer.intializeTheArm());
  }

  @Override
  public void simulationPeriodic() {}
}
