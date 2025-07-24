// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.extensions.SparkMaxPIDTunerPosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  private SparkMax lockingBarMotor;
  private SparkMax positionMotor;

  private AbsoluteEncoder lockingBarEncoder;
  private AbsoluteEncoder positionEncoder;

  private Servo latchingServo;

  private double lockingTargetPosition;
  private double climberTargetPosition;

  private SlewRateLimiter positionLimiter;
  private SlewRateLimiter lockingPositionLimiter;
  private SparkMaxPIDTunerPosition positionTuner;

  @SuppressWarnings("unused") 
  private TrapezoidProfile positionProfile;
  @SuppressWarnings("unused")
  private State positionCurrentState;
  @SuppressWarnings("unused")
  private State positionGoalState;

  // TODO: make this make sense to the drivers when it is in the dashboard
  private boolean moveClimberCommandLock = true;

  private Timer debugTimer;

  private boolean isInitialized = false;

  public ClimberSubsystem() {
    // lockingBarMotor = new
    // SimableSparkMax(Constants.ClimberSubsystem.LockingBarMotor.kMotorID,
    // MotorType.kBrushless, "LockingBarMotor");
    // positionMotor = new
    // SimableSparkMax(Constants.ClimberSubsystem.PositionMotor.kMotorID,
    // MotorType.kBrushless, "postionMotor");
    lockingBarMotor = new SparkMax(Constants.ClimberSubsystem.LockingBarMotor.kMotorID, MotorType.kBrushless);
    positionMotor = new SparkMax(Constants.ClimberSubsystem.PositionMotor.kMotorID, MotorType.kBrushless);

    latchingServo = new Servo(Constants.ClimberSubsystem.LatchServo.kServoID);
    lockingBarEncoder = lockingBarMotor.getAbsoluteEncoder();
    positionEncoder = positionMotor.getAbsoluteEncoder();

    configureLockingBarMotor();
    configurePositionMotor();

    positionLimiter = new SlewRateLimiter(Constants.ClimberSubsystem.PositionMotor.kSlewRate);
    lockingPositionLimiter = new SlewRateLimiter(Constants.ClimberSubsystem.LockingBarMotor.kSlewRate);

    positionProfile = new TrapezoidProfile(new Constraints(Constants.ClimberSubsystem.PositionMotor.kMaxVelocity, Constants.ClimberSubsystem.PositionMotor.kMaxAcceleration));
    positionCurrentState = new State();
    positionGoalState = new State();

    Shuffleboard.getTab("climber").add("Position Motor Position", positionMotor.getEncoder().getPosition());

    debugTimer = new Timer();
    debugTimer.start();

    if (Constants.kTuning) {
      positionTuner = new SparkMaxPIDTunerPosition("Climber Position", positionMotor, ControlType.kPosition);
      positionTuner.setSafeReferenceRange(Constants.ClimberSubsystem.PositionMotor.kMinAngle,
          Constants.ClimberSubsystem.PositionMotor.kMaxAngle);
      positionTuner.addToShuffleboard();
    }

  }

  public void initializeClimber() {
    climberTargetPosition = Constants.ClimberSubsystem.PositionMotor.kHomeAngle;
    lockingTargetPosition = Constants.ClimberSubsystem.LockingBarMotor.kMinLimit;
    unLatchCLimber();

    double currentPosition = getClimberPostion();
    double currentLockbar = getLockingMotorPosition();

    positionLimiter.reset(currentPosition);
    lockingPositionLimiter.reset(currentLockbar);

    positionCurrentState = new State(currentPosition, 0);
    positionGoalState = new State(currentPosition, 0);

    System.out.println("Reported Positions at Intialization: ");
    System.out.println("  Climber Position: " + currentPosition);
    System.out.println("  Lockbar Position: " + currentLockbar);

    // positionMotor.getClosedLoopController().setReference(getClimberPostion(),
    // ControlType.kPosition);
    // lockingBarMotor.getClosedLoopController().setReference(getLockingMotorPosition(),
    // ControlType.kPosition);

    isInitialized = true;
  }

  // Run in robots disabled init.
  public void setIntialize() {
    isInitialized = true;
  }

  public double getLockingMotorPosition() {
    return lockingBarEncoder.getPosition();
  }

  public double getClimberPostion() {
    return positionEncoder.getPosition();
  }

  public double getServoAngle() {
    return latchingServo.getAngle();
  }

  private void configurePositionMotor() {
    // create a new sparkmax config
    SparkMaxConfig positionMotorConfig = new SparkMaxConfig();

    positionMotorConfig
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        // TODO: This isn't how ramp rate should be used. Ramp rate is the number of
        // seconds it takes to go from 0 to 100% power.
        .openLoopRampRate(1)
        .closedLoopRampRate(1)
        .smartCurrentLimit(70, 30, 720);

    positionMotorConfig.absoluteEncoder
        .zeroOffset(Constants.ClimberSubsystem.PositionMotor.kEncoderOffset)
        .positionConversionFactor(Constants.ClimberSubsystem.PositionMotor.kConversionFactor)
        .inverted(true);

    positionMotorConfig.closedLoop
        // 0.05, 0.0000001, 0.08
        .p(Constants.ClimberSubsystem.PositionMotor.PIDF.kP)
        .i(Constants.ClimberSubsystem.PositionMotor.PIDF.kI)
        .d(Constants.ClimberSubsystem.PositionMotor.PIDF.kD)
        .iZone(Constants.ClimberSubsystem.PositionMotor.PIDF.kIZone)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    // apply configuration
    positionMotor.configure(positionMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void configureLockingBarMotor() {
    // create a new sparkmax config
    SparkMaxConfig lockingMotorConfig = new SparkMaxConfig();

    lockingMotorConfig
        .idleMode(IdleMode.kBrake)
        .inverted(true)
        // TODO: This isn't how ramp rate should be used. Ramp rate is the number of
        // seconds it takes to go from 0 to 100% power.
        .openLoopRampRate(Constants.ClimberSubsystem.LockingBarMotor.kSlewRate)
        .closedLoopRampRate(Constants.ClimberSubsystem.LockingBarMotor.kSlewRate)
        .smartCurrentLimit(70, 30, 1000);

    lockingMotorConfig.softLimit
        .reverseSoftLimit(Constants.ClimberSubsystem.LockingBarMotor.kMinLimit)
        .forwardSoftLimit(Constants.ClimberSubsystem.LockingBarMotor.kMaxLimit)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(true);

    lockingMotorConfig.absoluteEncoder
        .inverted(false)
        .zeroOffset(Constants.ClimberSubsystem.LockingBarMotor.kEncoderOffset)
        .positionConversionFactor(Constants.ClimberSubsystem.LockingBarMotor.kConversionFactor)
        .velocityConversionFactor(Constants.ClimberSubsystem.LockingBarMotor.kConversionFactor);

    lockingMotorConfig.closedLoop
        .p(0.01 / 2)
        .i(0)// 0.000001/2
        .d(0.01)// 0.007/2
        .iZone(5)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(false)
        .positionWrappingInputRange(0, 360);

    lockingMotorConfig.closedLoop.maxMotion
        .maxVelocity(5)
        .maxAcceleration(1)
        .allowedClosedLoopError(0.01);

    // apply configuration
    lockingBarMotor.configure(lockingMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setBarAngle(double angle) {
    lockingTargetPosition = MathUtil.clamp(angle, Constants.ClimberSubsystem.LockingBarMotor.kMinLimit,
        Constants.ClimberSubsystem.LockingBarMotor.kMaxLimit);
  }

  public void lockClimberBar() {
    double targetpostion = Constants.ClimberSubsystem.LockingBarMotor.kLockedPosition;
    setBarAngle(targetpostion);
  }

  public void unlockClimberBar() {
    double targetpostion = Constants.ClimberSubsystem.LockingBarMotor.kUnLockedPosition;
    setBarAngle(targetpostion);
  }

  // used to set climber using position buttons
  public void setClimberAngle(double angle) {
    climberTargetPosition = MathUtil.clamp(angle, Constants.ClimberSubsystem.PositionMotor.kMinAngle,
        Constants.ClimberSubsystem.PositionMotor.kMaxAngle);
  }

  public void extendClimber() {
    double targetpostion = Constants.ClimberSubsystem.PositionMotor.kDeployedAngle;
    setClimberAngle(targetpostion);
  }

  public void retractClimber() {
    double targetpostion = Constants.ClimberSubsystem.PositionMotor.kHomeAngle;
    setClimberAngle(targetpostion);
  }

  // Puts the climber in the position to be locked with the servo
  public void climberLockingPosition() {
    double targetpostion = Constants.ClimberSubsystem.PositionMotor.kLockingPosition;
    setClimberAngle(targetpostion);
  }

  // moves the climber (with the servo) to lock the climber to prepare for the
  // loss of power.
  public void climberLockedPosition() {
    double targetpostion = Constants.ClimberSubsystem.PositionMotor.kLockedPosition;
    setClimberAngle(targetpostion);
  }

  // operater controll of the climber
  public void changeClimberPosition(double delta) {
    double newPosition = getClimberPostion() + delta;
    setClimberAngle(newPosition);
  }

  // TODO: This should be private so that you can't bypass the intended methods
  // below
  public void setServoValue(double newValue) {
    newValue = MathUtil.clamp(newValue, Constants.ClimberSubsystem.LatchServo.minLimit,
        Constants.ClimberSubsystem.LatchServo.maxLimit);
    latchingServo.set(newValue);

  }

  public double getServoValue() {
    return latchingServo.get();
  }

  //
  public void latchCLimber() {
    setServoValue(Constants.ClimberSubsystem.LatchServo.latchedValue);
  }

  public void unLatchCLimber() {
    setServoValue(Constants.ClimberSubsystem.LatchServo.unLatchedValue);
  }

  public void moveCLimberCommandLock() {
    moveClimberCommandLock = true;
  }

  public void moveCLimberCommandUnLock() {
    moveClimberCommandLock = false;
  }

  public boolean isClimberCommandLocked() {
    return moveClimberCommandLock;
  }

  public boolean unlockClimberSubsystem() {
    moveClimberCommandLock = false;
    return moveClimberCommandLock;
  }

  public boolean lockClimberSubsystem() {
    moveClimberCommandLock = true;
    return moveClimberCommandLock;
  }

  @Override
  public void periodic() {

    // if tuning, do nothing
    if (Constants.kTuning) {
      positionTuner.periodic();
      return;
    }
    // if (Constants.kDebug == true){
    // SmartDashboard.putNumber("Locking Motor Position",
    // getLockingMotorPosition());
    // SmartDashboard.putNumber("Climber Motor Position", getClimberPostion());
    // SmartDashboard.putNumber("Servo position", getServoAngle());
    // SmartDashboard.putBoolean("Is climber Unlocked", moveClimberCommandLock);
    // if(debugTimer.get() > 1.5) {
    // System.out.println("Applied Position Motor Output: " +
    // positionMotor.getAppliedOutput());
    // System.out.println("Current Absolute Angle: " +
    // positionMotor.getAbsoluteEncoder().getPosition());
    // debugTimer.reset();
    // }
    // }

    SmartDashboard.putNumber("Locking Motor Position", getLockingMotorPosition());
    SmartDashboard.putNumber("Climber Motor Position", getClimberPostion());
    SmartDashboard.putBoolean("Is Climber locked?", moveClimberCommandLock);
    SmartDashboard.putBoolean("Is Climber Initialized", isInitialized);

    if ((RobotState.isTeleop() || RobotState.isAutonomous()) && isInitialized == true) {
      double immediateTargetAngle = MathUtil.clamp(positionLimiter.calculate(climberTargetPosition),
          Constants.ClimberSubsystem.PositionMotor.kMinAngle, Constants.ClimberSubsystem.PositionMotor.kMaxAngle);
      
      // XXX: comment if using trapezoidal profile
      positionMotor.getClosedLoopController().setReference(immediateTargetAngle, ControlType.kPosition);

      // XXX: uncomment for trapezoidal profile
      // positionGoalState = new State(climberTargetPosition, 0);
      // positionCurrentState = positionProfile.calculate(Constants.kRobotLoopTime, positionCurrentState, positionGoalState);
      // positionMotor.getClosedLoopController().setReference(positionCurrentState.position, ControlType.kPosition);

      double lockingImmediateTargetAngle = MathUtil.clamp(lockingPositionLimiter.calculate(lockingTargetPosition),
          Constants.ClimberSubsystem.LockingBarMotor.kMinLimit, Constants.ClimberSubsystem.LockingBarMotor.kMaxLimit);
      lockingBarMotor.getClosedLoopController().setReference(lockingImmediateTargetAngle, ControlType.kPosition);
    }
  }
}
