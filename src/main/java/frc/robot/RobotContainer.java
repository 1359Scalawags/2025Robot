// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Operator;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import java.io.File;
import java.util.concurrent.locks.Lock;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here

  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem(
    new File(Filesystem.getDeployDirectory(), "YAGSLConfigJSON/Flipper"));
   //private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
   private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
   private final CommandJoystick m_DriverJoystick = new CommandJoystick(Constants.Operator.DriverJoystick.kPort);
   private final CommandJoystick m_AssistantJoystick = new CommandJoystick(Constants.Operator.AssistJoystick.kPort);

  SendableChooser<Command> autoChooser;
  SendableChooser<Command> pipelineChooser;  


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    pipelineChooser = new SendableChooser<Command>();

    SmartDashboard.putData("Pipeline Chooser", pipelineChooser);
    SmartDashboard.putData("Auto Chooser ", autoChooser);

    // Configure the trigger bindings
    configureBindings();
    setDefaultCommands();     
  }

  private void setDefaultCommands() {
  
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




      //Which one of these did i use?
//   public Command getAutonomousCommandForChooser() {
//     return m_SwereveSubsystem.getAutonomousCommand(autoChooser.getSelected().getName());
//   }

// // Do i need .getName()?

//     public Command getAutonomousCommand(String exampleAuto){
//     //return m_SwerveSubsystem.getAutonomousCommand(autoChooser.getSelected().getName());
//     return m_SwereveSubsystem.getAutonomousCommand(exampleAuto);
//   }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    return null;
  }
}
