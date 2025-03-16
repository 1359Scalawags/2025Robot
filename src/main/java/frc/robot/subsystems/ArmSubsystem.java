package frc.robot.subsystems;

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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.ArmPosition;
import frc.robot.extensions.GravityAssistedFeedForward;

public class ArmSubsystem extends SubsystemBase {

  //private SimableSparkMax pulleyMotor, elbowMotor, wristMotor, clawMotor;
  private SparkMax pulleyMotor, elbowMotor, wristMotor, clawMotor;
  private double pulleyMotorTarget, elbowMotorTarget, wristMotorTarget, clawMotorTarget;
  private static double ARM_HEIGHT;

  private DigitalInput homeLimitSwitch;
  private DigitalInput clawLimitSwitch;

  // private boolean clawInitialized = false;
  // private boolean pulleyInitialized = false;
  private boolean initialized = false;
  private boolean elbowError = true;
  private boolean wristError = true;

  private GravityAssistedFeedForward elbowFF;
  private GravityAssistedFeedForward wristFF;

  // Trapezoidal profiling for elbow
  private TrapezoidProfile elbowProfile;
  private State elbowStateGoal;
  private State elbowStateSetpoint;

  // Trapezoidal profiling for wrist
  private TrapezoidProfile wristProfile;
  private State wristStateGoal;
  private State wristStateSetpoint;


  // Trapezoidal profiling for claw
  private TrapezoidProfile clawProfile;
  private State clawStateGoal;
  private State clawStateSetpoint;

  // Trapezoidal profiling for pulley
  private TrapezoidProfile pulleyProfile;
  private State pulleyStateGoal;
  private State pulleyStateSetpoint;

  
  public ArmSubsystem() {

    pulleyMotor = new SparkMax(Constants.ArmSubsystem.Pulley.kMotorID, MotorType.kBrushless);
    elbowMotor = new SparkMax(Constants.ArmSubsystem.Elbow.kMotorID, MotorType.kBrushless);
    wristMotor = new SparkMax(Constants.ArmSubsystem.Wrist.kMotorID, MotorType.kBrushless);
    clawMotor = new SparkMax(Constants.ArmSubsystem.Claw.kMotorID, MotorType.kBrushless);

    configureWristMotor();
    configureElbowMotor();
    configurePulleyMotor();
    configureClawMotor();

    homeLimitSwitch = new DigitalInput(Constants.ArmSubsystem.Pulley.kHomeLimitSwitchID);
    clawLimitSwitch = new DigitalInput(Constants.ArmSubsystem.Claw.kLimitSwitchID);

    elbowFF = new GravityAssistedFeedForward(Constants.ArmSubsystem.Elbow.PIDF.kMINGravityFF,
        Constants.ArmSubsystem.Elbow.PIDF.kGravityFF, Constants.ArmSubsystem.Elbow.kHorizontalAngle);
    
    wristFF =  new GravityAssistedFeedForward(Constants.ArmSubsystem.Wrist.PIDF.kMinGravityFF,
        Constants.ArmSubsystem.Wrist.PIDF.kGravityFF, Constants.ArmSubsystem.Wrist.kHorizontalAngle);

    // trapezoidal profiling for the elbow
    elbowProfile = new TrapezoidProfile(new Constraints(Constants.ArmSubsystem.Elbow.kSlewRate, Constants.ArmSubsystem.Elbow.kAccelerationRate));
    elbowStateSetpoint = new State();
    elbowStateGoal = new State();

    // trapezoidal profiling for the elbow
    wristProfile = new TrapezoidProfile(new Constraints(Constants.ArmSubsystem.Wrist.kSlewRate, Constants.ArmSubsystem.Wrist.kAccelerationRate));
    wristStateSetpoint = new State();
    wristStateGoal = new State();

    // trapezoidal profiling for the elbow
    clawProfile = new TrapezoidProfile(new Constraints(Constants.ArmSubsystem.Claw.kSlewRate, Constants.ArmSubsystem.Claw.kAccelerationRate));
    clawStateSetpoint = new State();
    clawStateGoal = new State();

    // trapezoidal profiling for the pulley
    pulleyProfile = new TrapezoidProfile(new Constraints(Constants.ArmSubsystem.Claw.kSlewRate, Constants.ArmSubsystem.Claw.kAccelerationRate));
    pulleyStateSetpoint = new State();
    pulleyStateGoal = new State();

  }

  public void initializeArm() {

    pulleyMotorTarget = pulleyMotor.getEncoder().getPosition();
    wristMotorTarget = wristMotor.getAbsoluteEncoder().getPosition();
    elbowMotorTarget = elbowMotor.getAbsoluteEncoder().getPosition();
    clawMotorTarget = clawMotor.getEncoder().getPosition();

    elbowStateGoal = new State(elbowMotorTarget, 0);
    elbowStateSetpoint = new State(elbowMotorTarget, 0);

    wristStateGoal = new State(wristMotorTarget, 0);
    wristStateSetpoint = new State(wristMotorTarget, 0);

    clawStateGoal = new State(clawMotorTarget, 0);
    clawStateSetpoint = new State(clawMotorTarget, 0);

    pulleyStateGoal = new State(pulleyMotorTarget, 0);
    pulleyStateSetpoint = new State(pulleyMotorTarget, 0);

    // this is now true as soon as encoders and limiters are initialized
    initialized = true;
  }

  private void configureWristMotor() {
    SparkMaxConfig wristMotorConfig = new SparkMaxConfig();

    wristMotorConfig
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .openLoopRampRate(1.0)
        .closedLoopRampRate(1.0)
        .smartCurrentLimit(20, 20, 720);
    
    wristMotorConfig.absoluteEncoder
        .zeroOffset(Constants.ArmSubsystem.Wrist.kMotorOffset)
        .positionConversionFactor(Constants.ArmSubsystem.Wrist.kConversionFactor);

    wristMotorConfig.closedLoop
        .pid(Constants.ArmSubsystem.Wrist.PIDF.kP,
             Constants.ArmSubsystem.Wrist.PIDF.kI,
             Constants.ArmSubsystem.Wrist.PIDF.kD)
        .iZone(Constants.ArmSubsystem.Wrist.PIDF.kIZone)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    // apply configuration
    wristMotor.configure(wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

    // apply configuration
    pulleyMotor.configure(pulleyMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void configureClawMotor() {
    SparkMaxConfig clawMotorConfig = new SparkMaxConfig();

    clawMotorConfig
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .openLoopRampRate(1.0)
        .closedLoopRampRate(1.0)
        .smartCurrentLimit(20, 20, 600);


    clawMotorConfig.closedLoop
        .pid(Constants.ArmSubsystem.Claw.PIDF.kP,
             Constants.ArmSubsystem.Claw.PIDF.kI,
             Constants.ArmSubsystem.Claw.PIDF.kD)
        .iZone(Constants.ArmSubsystem.Claw.PIDF.kIZone);

    // apply configuration
    clawMotor.configure(clawMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  //XXX: Only called in closeClawCommand.java, do we really need it?
  public void closeClaw() {
    goToClawMotorPosition(Constants.ArmSubsystem.Claw.kCloseClaw);
  }

  //XXX: Only called in openClawCommand.java, do we really need it?
  public void openClaw() {
    goToClawMotorPosition(Constants.ArmSubsystem.Claw.kOpenClaw);
  }

  public void goToPulleyMotorPosition(double pulleyMotorPosition) {
    pulleyMotorTarget = MathUtil.clamp(pulleyMotorPosition, Constants.ArmSubsystem.Pulley.kMinLimit,
        Constants.ArmSubsystem.Pulley.kMaxLimit);
  }

  public void goToElbowMotorPosition(double elbowMotorPosition) {
    elbowMotorTarget = MathUtil.clamp(elbowMotorPosition, Constants.ArmSubsystem.Elbow.kMinLimit,
        Constants.ArmSubsystem.Elbow.kMaxLimit);
  }

  public void goToWristMotorPosition(double wristMotorPosition) {
    wristMotorTarget = MathUtil.clamp(wristMotorPosition, getAbsoluteWristAngleMin(),
    getAbsoluteWristAngleMax());
  }

  public void goToClawMotorPosition(double clawMotorPosition) {
    clawMotorTarget = MathUtil.clamp(clawMotorPosition, Constants.ArmSubsystem.Claw.kMinLimit,
        Constants.ArmSubsystem.Claw.kMaxLimit);
  }

  public ArmPosition getArmPosition() {
    return new ArmPosition(getPulleyHeight(), getElbowMotorPosition(), getWristMotorPosition());
  }

  ///XXX: All these goToHeightX functions are only used in one command, are these really needed?
  /// Short answer....No.
  // Sets arm height to the ground
  public void goToHeightGround() {
    goToPulleyMotorPosition(Constants.ArmSubsystem.Positions.kGround.pulley);
  }

  // Sets arm height to Level Two
  public void goToHeightL2() {
    goToPulleyMotorPosition(Constants.ArmSubsystem.Positions.kLevel2.pulley);
  }

  // Sets arm height to Level Three
  public void goToHeightL3() {
    goToPulleyMotorPosition(Constants.ArmSubsystem.Positions.klevel3.pulley);
  }

  // Sets arm height to Level Four
  public void goToHeightL4() {
    goToPulleyMotorPosition(Constants.ArmSubsystem.Positions.klevel4.pulley);
  }

  // Sets arm height to the Human Station
  public void goToHeightHumanStation() {
    goToPulleyMotorPosition(Constants.ArmSubsystem.Positions.kHumanStation.pulley);

  }

  public void goToHeightHome() {
    goToPulleyMotorPosition(Constants.ArmSubsystem.Positions.kHome.pulley);
  }

  public void goToArmHome() {
    goToElbowMotorPosition(Constants.ArmSubsystem.Positions.kHome.elbow);
    goToWristMotorPosition(Constants.ArmSubsystem.Positions.kHome.wrist);
  }

  // Sets arm position to the Ground
  public void goToArmGround() {
    goToElbowMotorPosition(Constants.ArmSubsystem.Positions.kGround.elbow);
    goToWristMotorPosition(Constants.ArmSubsystem.Positions.kGround.wrist);
  }

  // Sets arm position to Level Two
  public void goToArmL2() {
    goToElbowMotorPosition(Constants.ArmSubsystem.Positions.kLevel2.elbow);
    goToWristMotorPosition(Constants.ArmSubsystem.Positions.kLevel2.wrist);
  }

  // Sets arm postion to Level Three
  public void goToArmL3() {
    goToElbowMotorPosition(Constants.ArmSubsystem.Positions.klevel3.elbow);
    goToWristMotorPosition(Constants.ArmSubsystem.Positions.klevel3.wrist);
  }

  // Sets arm position to Level Four
  public void goToArmL4() {
    goToElbowMotorPosition(Constants.ArmSubsystem.Positions.klevel4.elbow);
    goToWristMotorPosition(Constants.ArmSubsystem.Positions.klevel4.wrist);
  }

  // Sets arm position to the Human Station
  public void goToArmHumanStation() {
    goToElbowMotorPosition(Constants.ArmSubsystem.Positions.kHumanStation.elbow);
    goToWristMotorPosition(Constants.ArmSubsystem.Positions.kHumanStation.wrist);
  }

  public static double getPulleyHeight() {
    return ARM_HEIGHT;
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

  public double getClawMotorPosition() {
    return clawMotor.getEncoder().getPosition();
  }

  public boolean isClawAtHome() {
    return clawLimitSwitch.get() == Constants.ArmSubsystem.Claw.kLimitSwitchPressedState;
  }

  public boolean isPulleyAtHome() {
    return homeLimitSwitch.get() == Constants.ArmSubsystem.Claw.kLimitSwitchPressedState;
  }

  public double getPulleyMotorFF() {
    if (getPulleyHeight() <= Constants.ArmSubsystem.Pulley.kStageTwoPulleyPosition) {
      return Constants.ArmSubsystem.Pulley.PIDF.kStageOneFF;
    } else {
      return Constants.ArmSubsystem.Pulley.PIDF.kStageTwoFF;
    }
  }

  public double getWristMotorPosition() {
    return wristMotor.getAbsoluteEncoder().getPosition();
  }

  public double getCalculatedHeight() {
    return pulleyMotor.getEncoder().getPosition();
  }

  public double getRelativeWristAngle() {
    double wristAngle = wristMotor.getAbsoluteEncoder().getPosition() - Constants.ArmSubsystem.Wrist.kHorizontalAngle;
    return wristAngle;
  }

  public double getAbsoluteWristAngleMax() {
    double elbowDiff = getElbowMotorPosition() - Constants.ArmSubsystem.Elbow.kHorizontalAngle;
    return Constants.ArmSubsystem.Wrist.kMaxLimit + elbowDiff;
  }

  public double getAbsoluteWristAngleMin() {
    double elbowDiff = getElbowMotorPosition() - Constants.ArmSubsystem.Elbow.kHorizontalAngle;
    return Constants.ArmSubsystem.Wrist.kMinLimit + elbowDiff;
  }

  @Override
  public void periodic() {

    // if tuning, do not interfere
    if(Constants.kTuning) {
      return;
    }

    // update static variable accessible to other systems
    ARM_HEIGHT = getCalculatedHeight();
    
    // run system only when enabled and initialized in Auto or Teleop
    if (initialized && RobotState.isEnabled() && !RobotState.isTest()) {

      //check if pulley is home
      if (pulleyMotor.get() < 0) {
        if (homeLimitSwitch.get() == Constants.ArmSubsystem.Pulley.kLimitSwitchPressedState) {
          pulleyMotor.set(0);
          pulleyMotor.getEncoder().setPosition(0);
          
          // reset profile at 0
          pulleyStateGoal = new State(Constants.ArmSubsystem.Positions.kHome.pulley, 0);
          pulleyStateSetpoint = new State(0, 0);

        }
      }
  
      // check if claw is home
      if (clawMotor.get() > 0) {
        if (clawLimitSwitch.get() == Constants.ArmSubsystem.Claw.kLimitSwitchPressedState) {
          clawMotor.set(0);
          clawMotor.getEncoder().setPosition(0);

          // reset profile at 0
          clawStateGoal = new State(0,0);
          clawStateSetpoint = new State(0, 0);
        }
      }

      // precalculate safe wrist positions
      double wristSafeTarget = MathUtil.clamp(wristMotorTarget, getAbsoluteWristAngleMin(), getAbsoluteWristAngleMax());

      // XXX: if one or the other of the wrist is not working, then should we use any of it?
      if (wristError == false || elbowError == false) {
        // move the wrist towards its goal
        wristStateGoal = new State(wristSafeTarget, 0); //MUST USE SAFE TARGET
        wristStateSetpoint = wristProfile.calculate(Constants.kRobotLoopTime, wristStateSetpoint, wristStateGoal);
        wristMotor.getClosedLoopController().setReference(wristStateSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, wristFF.calculate(getWristMotorPosition()));

        // move the elbow towards its goal
        elbowStateGoal = new State(elbowMotorTarget, 0);
        elbowStateSetpoint = elbowProfile.calculate(Constants.kRobotLoopTime, elbowStateSetpoint, elbowStateGoal);
        elbowMotor.getClosedLoopController().setReference(elbowStateSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, elbowFF.calculate(getElbowMotorPosition()));
      }

      // move the claw towards its goal
      clawStateGoal = new State(clawMotorTarget, 0);
      clawStateSetpoint = clawProfile.calculate(Constants.kRobotLoopTime, clawStateSetpoint, clawStateGoal);
      clawMotor.getClosedLoopController().setReference(clawStateSetpoint.position, ControlType.kPosition);


      // move the pulley towards its goal
      pulleyStateGoal = new State(pulleyMotorTarget, 0);
      pulleyStateSetpoint = pulleyProfile.calculate(Constants.kRobotLoopTime, pulleyStateSetpoint, pulleyStateGoal);
      pulleyMotor.getClosedLoopController().setReference(pulleyStateSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, getPulleyMotorFF());
      
      
    }
  }
}
