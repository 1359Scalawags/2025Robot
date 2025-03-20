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
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.ArmPosition;
import frc.robot.extensions.GravityAssistedFeedForward;
import frc.robot.extensions.SimableSparkMax;
import frc.robot.extensions.SparkMaxPIDTunerArmPosition;
import frc.robot.extensions.SparkMaxPIDTunerPosition;
import frc.robot.extensions.SparkMaxPIDTunerBase.Verbosity;

public class ArmSubsystem extends SubsystemBase {

  //private SimableSparkMax pulleyMotor, elbowMotor, wristMotor, clawMotor;
  private SparkMax pulleyMotor, elbowMotor, wristMotor, clawMotor;
  private SlewRateLimiter pulleyLimiter, elbowLimiter, wristLimiter, clawLimiter;
  private double pulleyMotorTarget, elbowMotorTarget, wristMotorTarget, clawMotorTarget;
  private static double ARM_HEIGHT;

  private DigitalInput homeLimitSwitch;
  private DigitalInput clawLimitSwitch;

  private boolean clawInitialized = false;
  private boolean pulleyInitialized = false;
  private boolean initialized = false;
  private boolean elbowError = true;
  private boolean wristError = true;

  private GravityAssistedFeedForward elbowFF;
  private GravityAssistedFeedForward wristFF;

  private SparkMaxPIDTunerArmPosition elbowTuner;
  private SparkMaxPIDTunerArmPosition wristTuner;
  private SparkMaxPIDTunerPosition clawTuner;

  // Trapezoidal profiling for elbow
  private TrapezoidProfile elbowProfile;
  private State elbowStateGoal;
  private State elbowStateSetpoint;

  private TrapezoidProfile pulleyProfile;
  private State pulleyStateGoal;
  private State pulleyStateSetpoint;

  private TrapezoidProfile wristProfile;
  private State wristStateGoal;
  private State wristStateSetpoint;

  public ArmSubsystem() {

    // NetworkTableInstance.getDefault().getTable("SparkMaxData");

    // pulleyMotor = new SimableSparkMax(Constants.ArmSubsystem.Pulley.kMotorID, MotorType.kBrushless);
    // elbowMotor = new SimableSparkMax(Constants.ArmSubsystem.Elbow.kMotorID, MotorType.kBrushless);
    // wristMotor = new SimableSparkMax(Constants.ArmSubsystem.Wrist.kMotorID, MotorType.kBrushless);
    // clawMotor = new SimableSparkMax(Constants.ArmSubsystem.Claw.kMotorID, MotorType.kBrushless);

    pulleyMotor = new SparkMax(Constants.ArmSubsystem.Pulley.kMotorID, MotorType.kBrushless);
    elbowMotor = new SparkMax(Constants.ArmSubsystem.Elbow.kMotorID, MotorType.kBrushless);
    wristMotor = new SparkMax(Constants.ArmSubsystem.Wrist.kMotorID, MotorType.kBrushless);
    clawMotor = new SparkMax(Constants.ArmSubsystem.Claw.kMotorID, MotorType.kBrushless);

    // TODO: Change slewrate limiter constants from 0
    pulleyLimiter = new SlewRateLimiter(Constants.ArmSubsystem.Pulley.kSlewRate);
    elbowLimiter = new SlewRateLimiter(Constants.ArmSubsystem.Elbow.kSlewRate);
    wristLimiter = new SlewRateLimiter(Constants.ArmSubsystem.Wrist.kSlewRate);
    clawLimiter = new SlewRateLimiter(Constants.ArmSubsystem.Claw.kSlewRate);

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

    if(Constants.kTuning) {
      elbowTuner = new SparkMaxPIDTunerArmPosition("Elbow Motor", elbowMotor, ControlType.kPosition, elbowFF);
      elbowTuner.setSafeReferenceRange(185, 300);
      elbowTuner.addToShuffleboard();
      elbowTuner.setVerbosity(Verbosity.all);
      elbowTuner.setDebugInterval(2.0);

      wristTuner = new SparkMaxPIDTunerArmPosition("Wrist Motor", wristMotor, ControlType.kPosition, wristFF);
      wristTuner.addToShuffleboard();

      clawTuner = new SparkMaxPIDTunerPosition("Claw Motor", clawMotor, ControlType.kPosition);
      clawTuner.setSafeReferenceRange(Constants.ArmSubsystem.Claw.kMinLimit, Constants.ArmSubsystem.Claw.kMaxLimit);
      clawTuner.addToShuffleboard();
    }


    // trapezoidal profiling for the elbow
    elbowProfile = new TrapezoidProfile(new Constraints(Constants.ArmSubsystem.Elbow.kSlewRate, Constants.ArmSubsystem.Elbow.kAccelerationRate));
    elbowStateSetpoint = new State();
    elbowStateGoal = new State();

    pulleyProfile = new TrapezoidProfile(new Constraints(Constants.ArmSubsystem.Pulley.kSlewRate, Constants.ArmSubsystem.Pulley.kAccelerationRate));
    pulleyStateSetpoint = new State();
    pulleyStateGoal = new State();

    wristProfile = new TrapezoidProfile(new Constraints(Constants.ArmSubsystem.Wrist.kSlewRate, Constants.ArmSubsystem.Wrist.kAccelerationRate));
    wristStateSetpoint = new State();
    wristStateGoal = new State();

    // Shuffleboard.getTab("Arm").add("ArmLimitSwitch", homeLimitSwitch);
    Shuffleboard.getTab("Arm").add("ClawLimitSwitch", clawLimitSwitch);

    // Shuffleboard.getTab("Arm").add("IsIntialized", initialized);
    // Shuffleboard.getTab("Arm").add("Pulley Motor", pulleyMotor);
    // Shuffleboard.getTab("Arm").add("Wrist Motor", wristMotor);
    // Shuffleboard.getTab("Arm").add("Elbow Motor", elbowMotor);
    // Shuffleboard.getTab("Arm").add("Claw Motor", clawMotor);

    // Shuffleboard.getTab("Arm").add("Elbow Output", elbowMotor.getAppliedOutput());

    // Shuffleboard.getTab("Arm").addNumber("Elbow Absolute", elbowMotor.getAppliedOutput());
    // Shuffleboard.getTab("Arm").add("Wrist Absolute", wristMotor.getAbsoluteEncoder());
  }

  public void initializeArm() {

    pulleyMotorTarget = pulleyMotor.getEncoder().getPosition();
    wristMotorTarget = wristMotor.getAbsoluteEncoder().getPosition();
    elbowMotorTarget = elbowMotor.getAbsoluteEncoder().getPosition();
    clawMotorTarget = clawMotor.getEncoder().getPosition();

    elbowStateGoal = new State(elbowMotorTarget, 0);
    elbowStateSetpoint = new State(elbowMotorTarget, 0);

    pulleyStateGoal = new State(pulleyMotorTarget, 0);
    pulleyStateSetpoint = new State(pulleyMotorTarget, 0);

    wristStateGoal = new State(wristMotorTarget, 0);
    wristStateSetpoint = new State(wristMotorTarget, 0);

    if (MathUtil.isNear(wristMotorTarget, 0, 2)) {
      wristError = true;
      System.out.println("------WRIST ERROR---------");
      DriverStation.reportError("------WRIST ERROR---------", false);
    } else {
      wristError = false;
      wristMotorTarget = Constants.ArmSubsystem.Positions.kHome.wrist;    
    }

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
      System.out.println("  Wrist: " + getWristMotorPosition());
      System.out.println("  Claw: " + getClawMotorPosition());
      System.out.println("  calculated writst max: " + getAbsoluteWristAngleMax());
      System.out.println("  calculated writst min: " + getAbsoluteWristAngleMin());
    }

    pulleyLimiter.reset(pulleyMotorTarget);
    elbowLimiter.reset(elbowMotorTarget);
    wristLimiter.reset(wristMotorTarget);
    clawLimiter.reset(clawMotorTarget);

    //XXX: Removed old init code that involved telling motors to move here instead of periodic

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
   
    //XXX: Should we use soft limits for the elbow? I've put a template here in case we decide to
    // elbowMotorConfig.softLimit
    //   .forwardSoftLimit(max)
    //   .forwardSoftLimitEnabled(true)
    //   .reverseSoftLimit(min)
    //   .reverseSoftLimitEnabled(true);

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
    goToClawMotorPosition(Constants.ArmSubsystem.Claw.kCloseClaw, false);
  }

  //XXX: Only called in openClawCommand.java, do we really need it?
  public void openClaw() {
    goToClawMotorPosition(Constants.ArmSubsystem.Claw.kOpenClaw, false);
  }

  public void halfOpenClaw() {
    goToClawMotorPosition(Constants.ArmSubsystem.Claw.kHalfClaw, false);
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

  public void goToClawMotorPosition(double clawMotorPosition, boolean isHoming) {
    if(isHoming) {
      clawMotorTarget = clawMotorPosition;
    } else {
      clawMotorTarget = MathUtil.clamp(clawMotorPosition, Constants.ArmSubsystem.Claw.kMinLimit,
          Constants.ArmSubsystem.Claw.kMaxLimit);      
    }

  }

  public ArmPosition getArmPosition() {
    return new ArmPosition(getPulleyHeight(), getElbowMotorPosition(), getWristMotorPosition());
  }

  ///XXX: All these goToHeightX functions are only used in one command, are these really needed?
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

  public double pulleyMotorFF() {
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

  //XXX: Removed commented out implementations of getRelativeWristAngle

  //XXX: Why does this work?
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

  public boolean isClawInitialized() {
    return clawInitialized;
  }

  int counter = 0;
  // TODO: moving slow when within the range of the limit switch?
  @Override
  public void periodic() {

    // if tuning, do nothing
    if(Constants.kTuning == true) {
      SmartDashboard.putNumber("Elbow motor Position", elbowMotor.getAbsoluteEncoder().getPosition());

      elbowTuner.periodic();
      wristTuner.periodic();
      wristTuner.setSafeReferenceRange(getAbsoluteWristAngleMin(), getAbsoluteWristAngleMax());
      clawTuner.periodic();
      return;
    }

    // update static variable accessible to other systems
    ARM_HEIGHT = getCalculatedHeight();
   
    if (initialized && RobotState.isEnabled() && !RobotState.isTest()) {

      //check if pulley is home
      if (pulleyMotor.get() < 0) {
        if (homeLimitSwitch.get() == Constants.ArmSubsystem.Pulley.kLimitSwitchPressedState) {
          pulleyMotor.set(0);
          pulleyMotor.getEncoder().setPosition(0);
          pulleyLimiter.reset(0);
          //XXX: Removed set reference position
          // pulleyMotor.getClosedLoopController().setReference(pulleyLimiter.calculate(Constants.ArmSubsystem.Positions.kHome.pulley), ControlType.kPosition);
          pulleyMotorTarget = Constants.ArmSubsystem.Positions.kHome.pulley;
          pulleyInitialized = true;
        }
      }
  
      // check if claw is home
      if (clawMotor.get() > 0) {
        if (clawLimitSwitch.get() == Constants.ArmSubsystem.Claw.kLimitSwitchPressedState) {
          clawMotor.set(0);
          clawMotor.getEncoder().setPosition(0);
          clawLimiter.reset(0);
          //XXX: Removed set reference position
          // clawMotor.getClosedLoopController().setReference(clawLimiter.calculate(Constants.ArmSubsystem.Claw.kCloseClaw), ControlType.kPosition);
          clawMotorTarget = Constants.ArmSubsystem.Claw.kCloseClaw;
          clawInitialized = true;
        }
      }

      double wristSafeTarget = MathUtil.clamp(wristMotorTarget, getAbsoluteWristAngleMin(), getAbsoluteWristAngleMax());

      if (wristError == false) {
        wristStateGoal = new State(wristSafeTarget, 0);
        wristStateSetpoint = wristProfile.calculate(Constants.kRobotLoopTime, wristStateSetpoint, wristStateGoal);
        wristMotor.getClosedLoopController().setReference(wristStateSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, wristFF.calculate(getWristMotorPosition()));

        // wristMotor.getClosedLoopController().setReference(wristLimiter.calculate(wristSafeTarget), ControlType.kPosition, ClosedLoopSlot.kSlot0, wristFF.calculate(getWristMotorPosition()));
      }

      if (elbowError == false) { 
        // XXX: Uncomment to use trapezoidal profiling for the elbow 
        elbowStateGoal = new State(elbowMotorTarget, 0);
        elbowStateSetpoint = elbowProfile.calculate(Constants.kRobotLoopTime, elbowStateSetpoint, elbowStateGoal);
        elbowMotor.getClosedLoopController().setReference(elbowStateSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, elbowFF.calculate(getElbowMotorPosition()));

        // elbowMotor.getClosedLoopController().setReference(elbowLimiter.calculate(elbowMotorTarget),
        //     ControlType.kPosition, ClosedLoopSlot.kSlot0, elbowFF.calculate(getElbowMotorPosition()));
      }
    
      double clawSlewTarget = clawLimiter.calculate(clawMotorTarget);
      clawMotor.getClosedLoopController().setReference(clawSlewTarget, ControlType.kPosition);


      pulleyStateGoal = new State(pulleyMotorTarget, 0);
      pulleyStateSetpoint = pulleyProfile.calculate(Constants.kRobotLoopTime, pulleyStateSetpoint, pulleyStateGoal);
      pulleyMotor.getClosedLoopController().setReference(pulleyStateSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, pulleyMotorFF());

      // pulleyMotor.getClosedLoopController().setReference(pulleyLimiter.calculate(pulleyMotorTarget), ControlType.kPosition, ClosedLoopSlot.kSlot0, pulleyMotorFF());
      
      // Display values when debugging  
      if(Constants.kDebug) {
        clawInitialized = SmartDashboard.getBoolean("Variable: clawInitialized", clawInitialized);
        counter++;
        if(counter > 25) {
            //System.out.println("WristAngle: " + getRelativeWristAngle() + " FF: " + wristFF.calculate(getRelativeWristAngle()) + " Output: " + wristMotor.getAppliedOutput());
            //System.out.println("ElbowAngle: " + getElbowMotorPosition() + " FF: " + elbowFF.calculate(getElbowMotorPosition()) + " Output: " + elbowMotor.getAppliedOutput());
            System.out.println("Claw - Position: " + clawMotor.getEncoder().getPosition() + "  Target: " + clawMotorTarget + "  Slew Target: " + clawSlewTarget);
            counter=0;
        }     
      }   
    }
   
  }

}
