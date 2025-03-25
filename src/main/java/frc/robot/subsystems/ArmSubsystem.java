// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.extensions.ArmPosition;
import frc.robot.extensions.GravityAssistedFeedForward;

public class ArmSubsystem extends SubsystemBase {

  private SparkMax pulleyMotor, elbowMotor;
  private double pulleyMotorTarget, elbowMotorTarget;
  private static double ARM_HEIGHT;

  private DigitalInput homeLimitSwitch;

  private boolean pulleyInitialized = false;
  private boolean initialized = false;
  private boolean elbowError = true;

  private GravityAssistedFeedForward elbowFF;

  // Trapezoidal profiling for elbow
  private TrapezoidProfile elbowProfile;
  private State elbowStateGoal;
  private State elbowStateSetpoint;

  private TrapezoidProfile pulleyProfile;
  private State pulleyStateGoal;
  private State pulleyStateSetpoint;

  // For simulation
  private DCMotor pulleySimMotor, elbowSimMotor;
  private SparkMaxSim pulleySim, elbowSim;
  private ElevatorSim pulleyElevatorSim;
  private SingleJointedArmSim elbowArmSim;


  public ArmSubsystem() {

    pulleyMotor = new SparkMax(Constants.ArmSubsystem.Pulley.kMotorID, MotorType.kBrushless);
    elbowMotor = new SparkMax(Constants.ArmSubsystem.Elbow.kMotorID, MotorType.kBrushless);

    configureElbowMotor();
    configurePulleyMotor();

    homeLimitSwitch = new DigitalInput(Constants.ArmSubsystem.Pulley.kHomeLimitSwitchID);

    elbowFF = new GravityAssistedFeedForward(Constants.ArmSubsystem.Elbow.PIDF.kMINGravityFF,
        Constants.ArmSubsystem.Elbow.PIDF.kGravityFF, Constants.ArmSubsystem.Elbow.kHorizontalAngle);

    elbowProfile = new TrapezoidProfile(new Constraints(Constants.ArmSubsystem.Elbow.kSlewRate, Constants.ArmSubsystem.Elbow.kAccelerationRate));
    elbowStateSetpoint = new State();
    elbowStateGoal = new State();

    pulleyProfile = new TrapezoidProfile(new Constraints(Constants.ArmSubsystem.Pulley.kSlewRate, Constants.ArmSubsystem.Pulley.kAccelerationRate));
    pulleyStateSetpoint = new State();
    pulleyStateGoal = new State();

    if(Robot.isSimulation()) {
      pulleySimMotor = DCMotor.getNeo550(1);
      elbowSimMotor = DCMotor.getNeo550(1);

      pulleySim = new SparkMaxSim(pulleyMotor, pulleySimMotor);
      elbowSim = new SparkMaxSim(elbowMotor, elbowSimMotor);

      pulleyElevatorSim = new ElevatorSim(
                          pulleySimMotor,
                          64/1.0,
                          10.0,
                          0.0254,
                          0,
                          1.32,
                          true,
                          0.1,
                          0.01, 0.0);        

       //estimated on 5 lbs and 15 inches (thinking of wrist and claw as stationary)       
      elbowArmSim = new SingleJointedArmSim(
                          elbowSimMotor, 
                          64/1.0, 
                          0.027435,
                          0.3810,
                          -1.0472,
                          1.7453,
                          true,
                          0, //horizontal
                          0.01, 0.0);
    }
  }

  public void initializeArm() {

    pulleyMotorTarget = pulleyMotor.getEncoder().getPosition();
    elbowMotorTarget = elbowMotor.getAbsoluteEncoder().getPosition();

    elbowStateGoal = new State(elbowMotorTarget, 0);
    elbowStateSetpoint = new State(elbowMotorTarget, 0);

    pulleyStateGoal = new State(pulleyMotorTarget, 0);
    pulleyStateSetpoint = new State(pulleyMotorTarget, 0);

    if (MathUtil.isNear(elbowMotorTarget, 0, 2)) {
      elbowError = true;
      System.out.println("------ELBOW ERROR---------");
      DriverStation.reportError("------ELBOW ERROR---------", false);
    } else {
      elbowError = false;
      elbowMotorTarget = Constants.ArmSubsystem.Positions.kHome.elbow;    
    }

    if (Constants.kDebug) {
      System.out.println("--------------Reported Positions at Intialization: --------------");
      System.out.println("  Pulley: " + getPulleyHeight());
      System.out.println("  Elbow: " + getElbowMotorPosition());
    }

    initialized = true;
  }

  private void configureElbowMotor() {
    SparkMaxConfig elbowMotorConfig = new SparkMaxConfig();

    elbowMotorConfig
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .openLoopRampRate(1.0)
        .closedLoopRampRate(1.0)
        .smartCurrentLimit(20, 20, 720);

    elbowMotorConfig.absoluteEncoder
        .zeroOffset(Constants.ArmSubsystem.Elbow.kMotorOffset)
        .positionConversionFactor(Constants.ArmSubsystem.Elbow.kConversionFactor);

    elbowMotorConfig.closedLoop
        .pid(Constants.ArmSubsystem.Elbow.PIDF.kP,
             Constants.ArmSubsystem.Elbow.PIDF.kI,
             Constants.ArmSubsystem.Elbow.PIDF.kD)

        .iZone(Constants.ArmSubsystem.Elbow.PIDF.kIZone)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    // apply configuration
    elbowMotor.configure(elbowMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void configurePulleyMotor() {
    SparkMaxConfig pulleyMotorConfig = new SparkMaxConfig();

    pulleyMotorConfig
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .openLoopRampRate(1.0)
        .closedLoopRampRate(1.0)
        .smartCurrentLimit(20, 20, 1000);

    pulleyMotorConfig.encoder
        .positionConversionFactor(Constants.ArmSubsystem.Pulley.kConversionFactor);

    pulleyMotorConfig.closedLoop // TODO: do we want a second slot for the upper part of the Pulley?
        .pid(Constants.ArmSubsystem.Pulley.PIDF.kP,
             Constants.ArmSubsystem.Pulley.PIDF.kI,
             Constants.ArmSubsystem.Pulley.PIDF.kD)
        .iZone(Constants.ArmSubsystem.Pulley.PIDF.kIZone);

    // .pid(0.045f, 0.00001f, 0.045, ClosedLoopSlot.kSlot1)
    // .iZone(2, ClosedLoopSlot.kSlot1);

    // apply configuration
    pulleyMotor.configure(pulleyMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void goToPulleyMotorPosition(double pulleyMotorPosition, boolean isHoming) {
    if(isHoming) {
      pulleyMotorTarget = pulleyMotorPosition;
    } else {
      pulleyMotorTarget = MathUtil.clamp(pulleyMotorPosition, Constants.ArmSubsystem.Pulley.kMinLimit,
          Constants.ArmSubsystem.Pulley.kMaxLimit);      
    }

  }

  public void goToElbowMotorPosition(double elbowMotorPosition) {
    elbowMotorTarget = MathUtil.clamp(elbowMotorPosition, Constants.ArmSubsystem.Elbow.kMinLimit,
        Constants.ArmSubsystem.Elbow.kMaxLimit);
  }

  public ArmPosition getArmPosition() {
    return new ArmPosition(getPulleyHeight(), getElbowMotorPosition());
  }

  // Sets arm height to the ground
  public void goToHeightGround() {
    goToPulleyMotorPosition(Constants.ArmSubsystem.Positions.kGround.pulley, false);
  }

  // Sets arm height to Level Two
  public void goToHeightL2() {
    goToPulleyMotorPosition(Constants.ArmSubsystem.Positions.kLevel2.pulley, false);
  }

  // Sets arm height to Level Three
  public void goToHeightL3() {
    goToPulleyMotorPosition(Constants.ArmSubsystem.Positions.klevel3.pulley, false);
  }

  // Sets arm height to Level Four
  public void goToHeightL4() {
    goToPulleyMotorPosition(Constants.ArmSubsystem.Positions.klevel4.pulley, false);
  }

  // Sets arm height to the Human Station
  public void goToHeightHumanStation() {
    goToPulleyMotorPosition(Constants.ArmSubsystem.Positions.kHumanStation.pulley, false);

  }

  public void goToHeightHome() {
    goToPulleyMotorPosition(Constants.ArmSubsystem.Positions.kHome.pulley,  false);
  }

  public void goToArmHome() {
    goToElbowMotorPosition(Constants.ArmSubsystem.Positions.kHome.elbow);
  }

  // Sets arm position to the Ground
  public void goToArmGround() {
    goToElbowMotorPosition(Constants.ArmSubsystem.Positions.kGround.elbow);
  }

  // Sets arm position to Level Two
  public void goToArmL2() {
    goToElbowMotorPosition(Constants.ArmSubsystem.Positions.kLevel2.elbow);
  }

  // Sets arm postion to Level Three
  public void goToArmL3() {
    goToElbowMotorPosition(Constants.ArmSubsystem.Positions.klevel3.elbow);
  }

  // Sets arm position to Level Four
  public void goToArmL4() {
    goToElbowMotorPosition(Constants.ArmSubsystem.Positions.klevel4.elbow);
  }

  // Sets arm position to the Human Station
  public void goToArmHumanStation() {
    goToElbowMotorPosition(Constants.ArmSubsystem.Positions.kHumanStation.elbow);
  }

  public double getPulleyHeight() {
    return this.pulleyMotor.getEncoder().getPosition();
  }

  public static double getSpeedMultiplier() {
    if(ARM_HEIGHT >= 40) {
      return 0.25;
    } else if(ARM_HEIGHT < 10) {
      return 1.0;
    } else {
      return -0.025*ARM_HEIGHT + 1.25;
    }
  }

  public double getElbowMotorPosition() {
    return elbowMotor.getAbsoluteEncoder().getPosition();
  }


  public boolean isPulleyAtHome() {
    return homeLimitSwitch.get() == Constants.ArmSubsystem.Pulley.kLimitSwitchPressedState;
  }

  public double pulleyMotorFF() {
    if (getPulleyHeight() <= Constants.ArmSubsystem.Pulley.kStageTwoPulleyPosition) {
      return Constants.ArmSubsystem.Pulley.PIDF.kStageOneFF;
    } else {
      return Constants.ArmSubsystem.Pulley.PIDF.kStageTwoFF;
    }
  }

  public boolean isPulleyInitialized() {
    return pulleyInitialized;
  }

  int counter = 0;
  // TODO: moving slow when within the range of the limit switch?
  @SuppressWarnings("unused")
  @Override
  public void periodic() {

    // update static variable accessible to other systems
    ARM_HEIGHT = getPulleyHeight();
   
    if (initialized && RobotState.isEnabled() && !RobotState.isTest()) {

      //check if pulley is home
      if (pulleyMotor.get() < 0) {
        if (homeLimitSwitch.get() == Constants.ArmSubsystem.Pulley.kLimitSwitchPressedState) {
          pulleyMotor.set(0);
          pulleyMotor.getEncoder().setPosition(0);
          pulleyMotorTarget = Constants.ArmSubsystem.Positions.kHome.pulley;
          pulleyInitialized = true;
        }
      }

      if (elbowError == false) { 
        elbowStateGoal = new State(elbowMotorTarget, 0);
        elbowStateSetpoint = elbowProfile.calculate(Constants.kRobotLoopTime, elbowStateSetpoint, elbowStateGoal);
        elbowMotor.getClosedLoopController().setReference(elbowStateSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, elbowFF.calculate(getElbowMotorPosition()));
      }
    

      pulleyStateGoal = new State(pulleyMotorTarget, 0);
      pulleyStateSetpoint = pulleyProfile.calculate(Constants.kRobotLoopTime, pulleyStateSetpoint, pulleyStateGoal);
      pulleyMotor.getClosedLoopController().setReference(pulleyStateSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, pulleyMotorFF());
      
      // Display values when debugging  
      if(Constants.kDebug) {
        counter++;
        if(counter > 25) {
            System.out.println("Pulley Target: " + pulleyMotorTarget + "  Actual: " + getPulleyHeight());
            //System.out.println("WristAngle: " + getRelativeWristAngle() + " FF: " + wristFF.calculate(getRelativeWristAngle()) + " Output: " + wristMotor.getAppliedOutput());
            //System.out.println("ElbowAngle: " + getElbowMotorPosition() + " FF: " + elbowFF.calculate(getElbowMotorPosition()) + " Output: " + elbowMotor.getAppliedOutput());
            counter=0;
        }     
      }   
    }
   
  }

  long totalRuns = 0;
  public void simulationPeriodic() {

    //update the WPILIB elevator simulator
    pulleyElevatorSim.setInput(pulleySim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    pulleyElevatorSim.update(Constants.kSimulationLoopTime);

    //update the SparkMax simulator
    pulleySim.iterate(pulleyElevatorSim.getVelocityMetersPerSecond(), RoboRioSim.getVInVoltage(), 0.02);

    elbowArmSim.setInput(elbowSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    elbowArmSim.update(Constants.kSimulationLoopTime);

    //update battery voltage
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(pulleySim.getMotorCurrent()));

    // totalRuns++;
    // if(totalRuns % 50 == 0) {
    //   //System.out.println("RUN: ArmSubsystem::simulationPeriodic() x " + 50);
    // }
  }

}
