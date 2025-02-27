package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.ArmPosition;
import frc.robot.extensions.GravityAssistedFeedForward;
import frc.robot.extensions.SimableSparkMax;

public class ArmSubsystem extends SubsystemBase {

  private SimableSparkMax pulleyMotor, elbowMotor, wristMotor, clawMotor;
  private SlewRateLimiter pulleyLimiter, elbowLimiter, wristLimiter, clawLimiter;
  private double pulleyMotorTarget, elbowMotorTarget, wristMotorTarget, clawMotorTarget;
  private static double ARM_HEIGHT;

  private DigitalInput homeLimitSwitch;
  private DigitalInput clawLimitSwitch;

  private boolean clawInitialized = false;
  private boolean pulleyInitialized = false;
  private boolean initialized = false;

  private GravityAssistedFeedForward elbowFF;

  public ArmSubsystem() {
    pulleyMotor = new SimableSparkMax(Constants.ArmSubsystem.Pulley.kMotorID, MotorType.kBrushless);
    elbowMotor = new SimableSparkMax(Constants.ArmSubsystem.Elbow.kMotorID, MotorType.kBrushless);
    wristMotor = new SimableSparkMax(Constants.ArmSubsystem.Wrist.kMotorID, MotorType.kBrushless);
    clawMotor = new SimableSparkMax(Constants.ArmSubsystem.Claw.kMotorID, MotorType.kBrushless);

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

    elbowFF = new GravityAssistedFeedForward(Constants.ArmSubsystem.Elbow.kMINGravityFF,
        Constants.ArmSubsystem.Elbow.kGravityFF, 232);

    Shuffleboard.getTab("Arm").add("ArmLimitSwitch", homeLimitSwitch);
    Shuffleboard.getTab("Arm").add("ClawLimitSwitch", clawLimitSwitch);
    Shuffleboard.getTab("Arm").add("IsIntialized", initialized);
  }

  public void initializeArm() {
    pulleyMotorTarget = pulleyMotor.getEncoder().getPosition();
    elbowMotorTarget = elbowMotor.getAbsoluteEncoder().getPosition();
    wristMotorTarget = wristMotor.getAbsoluteEncoder().getPosition();
    clawMotorTarget = clawMotor.getEncoder().getPosition();

    System.out.println("Reported Positions at Intialization: ");
    System.out.println("  Pulley: " + pulleyMotorTarget);
    System.out.println("  Elbow: " + elbowMotorTarget);
    System.out.println("  Wrist: " + wristMotorTarget);
    System.out.println("  Claw: " + clawMotorTarget);

    pulleyLimiter.reset(pulleyMotorTarget);
    elbowLimiter.reset(elbowMotorTarget);
    wristLimiter.reset(wristMotorTarget);
    clawLimiter.reset(clawMotorTarget);

    pulleyMotor.setReferencePosition(pulleyLimiter, pulleyMotorTarget);
    elbowMotor.setReferencePosition(elbowLimiter, elbowMotorTarget);

    double safeWritsTarget = MathUtil.clamp(wristMotorTarget, getAbsoluteWristAngleMin(), getAbsoluteWristAngleMax());
    wristMotor.setReferencePosition(wristLimiter, safeWritsTarget);
    clawMotor.setReferencePosition(clawLimiter, clawMotorTarget);

    // !!Homing moved to separate commands
    // if (this.homeLimitSwitch.get() == Constants.ArmSubsystem.Pulley.kLimitSwitchPressedState) {
    //   pulleyMotorTarget = Constants.ArmSubsystem.Positions.kHome.pulley;
    //   pulleyMotor.getEncoder().setPosition(0);
    //   pulleyLimiter.reset(0);
    //   pulleyInitialized = true;
    // } else {
    //   pulleyMotor.getClosedLoopController().setReference(Constants.ArmSubsystem.Pulley.kHomingVelocity, ControlType.kVelocity);
    // }

    // if(this.clawLimitSwitch.get() == Constants.ArmSubsystem.Claw.kLimitSwitchPressedState) {
    //    clawMotorTarget = Constants.ArmSubsystem.Claw.kCloseClaw;
    //    clawMotor.getEncoder().setPosition(0);
    //    clawLimiter.reset(0);
    //    clawInitialized = true;
    // } else {
    //   clawMotor.getClosedLoopController().setReference(Constants.ArmSubsystem.Claw.kHomingVelocity, ControlType.kVelocity);
    // }

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
        .smartCurrentLimit(20, 20, 120);
    
    wristMotorConfig.absoluteEncoder
        .zeroOffset(Constants.ArmSubsystem.Wrist.kMotorOffset)
        .positionConversionFactor(Constants.ArmSubsystem.Wrist.kConversionFactor);

    wristMotorConfig.closedLoop
        .pid(0.003, 0.0000003, 0.003)// (0.006, 0.0000006, 0.006)
        .iZone(2)
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
        .smartCurrentLimit(20, 20, 120);
   
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
        .pid(0.0125/2, 0.000015, 0.025)// (0.025, 0.00003, 0.05)
        .iZone(2)
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
        .smartCurrentLimit(20, 20, 120);

    pulleyMotorConfig.encoder
        .positionConversionFactor(Constants.ArmSubsystem.Pulley.kConversionFactor);

    pulleyMotorConfig.closedLoop // TODO: do we want a second slot for the upper part of the Pulley?
        .pid(0.0225f, 0.000005f, 0.0225)// .pid(0.045f, 0.00001f, 0.045, ClosedLoopSlot.kSlot0)
        .iZone(2);

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
        .smartCurrentLimit(20, 20, 120);

    clawMotorConfig.absoluteEncoder
        .zeroOffset(Constants.ArmSubsystem.Claw.kMotorOffset)
        .positionConversionFactor(Constants.ArmSubsystem.Claw.kConversionFactor);

    clawMotorConfig.closedLoop
        .pid(0, 0, 0)// (0.05, 0.0001, 0.03)
        .iZone(1.5);

    // apply configuration
    clawMotor.configure(clawMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void closeClaw() {
    goToClawMotorPosition(Constants.ArmSubsystem.Claw.kCloseClaw);
  }

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
    wristMotorTarget = MathUtil.clamp(wristMotorPosition, Constants.ArmSubsystem.Wrist.kMinLimit,
        Constants.ArmSubsystem.Wrist.kMaxLimit);
  }

  public void goToClawMotorPosition(double clawMotorPosition) {
    clawMotorTarget = MathUtil.clamp(clawMotorPosition, Constants.ArmSubsystem.Claw.kMinLimit,
        Constants.ArmSubsystem.Claw.kMaxLimit);
  }

  public ArmPosition getArmPosition() {
    return new ArmPosition(getPulleyHeight(), getElbowMotorPosition(), getWristMotorPosition());
  }

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
      return Constants.ArmSubsystem.Pulley.kStageOneFF;
    } else {
      return Constants.ArmSubsystem.Pulley.kStageTwoFF;
    }
  }

  public double getWristMotorPosition() {
    return wristMotor.getAbsoluteEncoder().getPosition();
  }

  public double getCalculatedHeight() {
    return pulleyMotor.getEncoder().getPosition();
  }

  // TODO: figure out way to make this work
  public double getRelativeWristAngle() {
    double wristAngle = wristMotor.getAbsoluteEncoder().getPosition() + elbowMotor.getAbsoluteEncoder().getPosition();
    return wristAngle;
  }

  // XXX:WRIST: Is this really needed?
  // public double getRelativeWristAngle() {
  //   double elbowDiff = getElbowMotorPosition() - Constants.ArmSubsystem.Elbow.kHorizontalAngle;
  //   double wristAngle = wristMotor.getAbsoluteEncoder().getPosition() - elbowDiff; 
  //   return wristAngle;
  // }

  public double getAbsoluteWristAngleMax() {
    double elbowDiff = getElbowMotorPosition() - Constants.ArmSubsystem.Elbow.kHorizontalAngle;
    return Constants.ArmSubsystem.Wrist.kMaxLimit + elbowDiff;
  }

  public double getAbsoluteWristAngleMin() {
    double elbowDiff = getElbowMotorPosition() - Constants.ArmSubsystem.Elbow.kHorizontalAngle;
    return Constants.ArmSubsystem.Wrist.kMinLimit + elbowDiff;
  }


  // TODO: moving slow when within the range of the limit switch?
  @Override
  public void periodic() {

    ARM_HEIGHT = getCalculatedHeight();

    // Limit switch for pully?

    ARM_HEIGHT = getCalculatedHeight();
    double wristSafeTarget = MathUtil.clamp(wristMotorTarget, getAbsoluteWristAngleMin(), getAbsoluteWristAngleMax());

    if (pulleyMotor.get() < 0) {
      if (homeLimitSwitch.get() == Constants.ArmSubsystem.Pulley.kLimitSwitchPressedState) {
        pulleyMotor.set(0);
        pulleyMotor.getEncoder().setPosition(0);
        pulleyLimiter.reset(0);
        pulleyMotor.setReferencePosition(pulleyLimiter, Constants.ArmSubsystem.Positions.kHome.pulley); // TODO: do                                                                                                     // once
        pulleyInitialized = true;                                                                                                 // check
      }
    }

    // Claw limit switch
    if (clawMotor.get() > 0) {
      if (clawLimitSwitch.get() == Constants.ArmSubsystem.Claw.kLimitSwitchPressedState) {
        clawMotor.set(0);
        clawMotor.getEncoder().setPosition(0);
        clawLimiter.reset(0);
        clawMotor.setReferencePosition(clawLimiter, Constants.ArmSubsystem.Claw.kCloseClaw);
        clawInitialized = true;
      }
    }


    if (initialized) {
      // run motors
      // if ((RobotState.isTeleop() || RobotState.isAutonomous()) && RobotState.isEnabled()) {
      if (RobotState.isEnabled()) {
        // This method will be called once per scheduler run
        //elbowMotor.getClosedLoopController().setReference(elbowLimiter.calculate(elbowMotorTarget),
            //ControlType.kPosition, ClosedLoopSlot.kSlot0, elbowFF.calculate(getElbowMotorPosition())); // must change
            
        //pulleyMotor.getClosedLoopController().setReference(pulleyLimiter.calculate(elbowMotorTarget),
            //ControlType.kPosition, ClosedLoopSlot.kSlot0, Constants.ArmSubsystem.Pulley.kStageOneFF / 2);
        elbowMotor.setReferencePosition(elbowLimiter, elbowMotorTarget);

        //XXX:WRIST: prevent wrist from going outside valid bounds
        wristMotor.setReferencePosition(wristLimiter, wristSafeTarget);
        //NOT SAFE: wristMotor.setReferencePosition(wristLimiter, wristMotorTarget);
        clawMotor.setReferencePosition(clawLimiter, clawMotorTarget);
        pulleyMotor.setReferencePosition(pulleyLimiter, pulleyMotorTarget);
        System.out.println("message AHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH??");


      }
    } else {
      if ((RobotState.isTeleop() || RobotState.isAutonomous()) && RobotState.isEnabled()) {
       // wristMotor.setReferencePosition(wristLimiter, wristSafeTarget);
      }
    }
  }

}
