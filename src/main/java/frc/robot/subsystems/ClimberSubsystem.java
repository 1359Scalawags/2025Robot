package frc.robot.subsystems;

<<<<<<< HEAD
=======
import com.revrobotics.AbsoluteEncoder;
>>>>>>> ccb55d266ba6a39506bcf2d4de652e18d5527e12
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

<<<<<<< HEAD
=======
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.RobotState;
>>>>>>> ccb55d266ba6a39506bcf2d4de652e18d5527e12
import edu.wpi.first.wpilibj.Servo;
import frc.robot.extensions.SimableSparkMax;
import frc.robot.extensions.SparkMaxPIDTunerPosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase{
  // private SimableSparkMax lockingBarMotor;  
  // private SimableSparkMax positionMotor;
  private SparkMax lockingBarMotor;  
  private SparkMax positionMotor;

  private Servo latchingServo;

  private SparkMaxPIDTunerPosition lockBarTuner;
  private SparkMaxPIDTunerPosition positionMotorTuner;


  public ClimberSubsystem() {
    lockingBarMotor = new SimableSparkMax(Constants.ClimberSubsystem.LockingBarMotor.kMotorID, MotorType.kBrushless, "LockingBarMotor");
    positionMotor = new SimableSparkMax(Constants.ClimberSubsystem.PositionMotor.kMotorID, MotorType.kBrushless, "postionMotor");
    latchingServo = new Servo(Constants.ClimberSubsystem.LatchServo.kServoID);

    configureLockingBarMotor();
    configurePositionMotor();
    
    lockBarTuner = new SparkMaxPIDTunerPosition("C: Lock Bar", lockingBarMotor, ControlType.kPosition);
    //lockBarTuner = new SparkMaxPIDTuner("C: Lock Bar", lockingBarMotor, ControlType.kMAXMotionPositionControl);
    positionMotorTuner = new SparkMaxPIDTunerPosition("C: Position", positionMotor, ControlType.kPosition);

  }

<<<<<<< HEAD
=======
    public ClimberSubsystem() {
      // lockingBarMotor = new SimableSparkMax(Constants.ClimberSubsystem.LockingBarMotor.kMotorID, MotorType.kBrushless, "LockingBarMotor");
      // positionMotor = new SimableSparkMax(Constants.ClimberSubsystem.PositionMotor.kMotorID, MotorType.kBrushless, "postionMotor");
      lockingBarMotor = new SparkMax(Constants.ClimberSubsystem.LockingBarMotor.kMotorID, MotorType.kBrushless);
      positionMotor = new SparkMax(Constants.ClimberSubsystem.PositionMotor.kMotorID, MotorType.kBrushless);

      latchingServo = new Servo(Constants.ClimberSubsystem.LatchServo.kServoID);
      lockingBarEncoder = lockingBarMotor.getAbsoluteEncoder();
      positionEncoder = positionMotor.getAbsoluteEncoder();
>>>>>>> ccb55d266ba6a39506bcf2d4de652e18d5527e12

  private void configurePositionMotor(){
    // create a new sparkmax config
    SparkMaxConfig positionMotorConfig = new SparkMaxConfig();

    positionMotorConfig
      .idleMode(IdleMode.kCoast)
      .inverted(false)
      .openLoopRampRate(Constants.ClimberSubsystem.PositionMotor.kSlewRate)
      .closedLoopRampRate(Constants.ClimberSubsystem.PositionMotor.kSlewRate)
      .smartCurrentLimit(70, 30, 120);

    positionMotorConfig.absoluteEncoder
    .zeroOffset(Constants.ClimberSubsystem.PositionMotor.kEncoderOffset)
    .positionConversionFactor(Constants.ClimberSubsystem.PositionMotor.kConversionFactor)
    .inverted(true);
      
<<<<<<< HEAD
    positionMotorConfig.closedLoop
    .p(0.01f)
    .i(0.0)
    .d(0.0)
    .iZone(0.001)
    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
=======
      Shuffleboard.getTab("climber").add("Position Motor", positionMotor);
      Shuffleboard.getTab("climber").add("Latching Servo", latchingServo); 
      Shuffleboard.getTab("climber").add("Locking Bar", lockingBarMotor);
      Shuffleboard.getTab("climber").add("Position Motor Position",positionMotor.getEncoder().getPosition());
      Shuffleboard.getTab("climber").add("Applied output", positionMotor.getAppliedOutput());
>>>>>>> ccb55d266ba6a39506bcf2d4de652e18d5527e12

    // apply configuration
    positionMotor.configure(positionMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

<<<<<<< HEAD
  private void configureLockingBarMotor() {
    // create a new sparkmax config
    SparkMaxConfig lockingMotorConfig = new SparkMaxConfig();

    lockingMotorConfig
      .idleMode(IdleMode.kBrake)
      .inverted(true)
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
      .p(0.005)
      .i(0.0)
      .d(0.0)
      .iZone(5)
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .positionWrappingEnabled(false)
      .positionWrappingInputRange(0,360);
=======
    public void initializeClimber() {
        climberTargetPosition = Constants.ClimberSubsystem.PositionMotor.kHomeAngle;
        lockingTargetPosition = Constants.ClimberSubsystem.LockingBarMotor.kMinLimit;
        unLatchCLimber();

        double currentPosition = getClimberPostion();
        double currentLockbar = getLockingMotorPosition();
        positionLimiter.reset(currentPosition);
        lockingPositionLimiter.reset(currentLockbar);
        System.out.println("Reported Positions at Intialization: ");
        System.out.println("  Climber Position: " + currentPosition);
        System.out.println("  Lockbar Position: " + currentLockbar);

        positionMotor.getClosedLoopController().setReference(getClimberPostion(), ControlType.kPosition);
        lockingBarMotor.getClosedLoopController().setReference(getLockingMotorPosition(), ControlType.kPosition);

        isInitialized = true;
    }

        //Run in robots disabled init.
    public void deInitialize() {
      isInitialized = false;
    }

    public double getLockingMotorPosition(){
      return lockingBarEncoder.getPosition();
    }

    public double getClimberPostion(){
      return positionEncoder.getPosition();
    }

    public double getServoAngle(){
      return latchingServo.getAngle();
    }

    private void configurePositionMotor(){
      // create a new sparkmax config
      SparkMaxConfig positionMotorConfig = new SparkMaxConfig();

      positionMotorConfig
        .idleMode(IdleMode.kCoast)
        .inverted(false)
        .openLoopRampRate(Constants.ClimberSubsystem.PositionMotor.kSlewRate)
        .closedLoopRampRate(Constants.ClimberSubsystem.PositionMotor.kSlewRate)
        .smartCurrentLimit(70, 30, 120);

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
>>>>>>> ccb55d266ba6a39506bcf2d4de652e18d5527e12

    lockingMotorConfig.closedLoop.maxMotion
      .maxVelocity(5)
      .maxAcceleration(1)
      .allowedClosedLoopError(0.01);

<<<<<<< HEAD
    // apply configuration
    lockingBarMotor.configure(lockingMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }


  @Override
  public void periodic() {
    lockBarTuner.updateEncoderValues();
    positionMotorTuner.updateEncoderValues();
=======
    private void configureLockingBarMotor() {
      // create a new sparkmax config
      SparkMaxConfig lockingMotorConfig = new SparkMaxConfig();
  
      lockingMotorConfig
        .idleMode(IdleMode.kBrake)
        .inverted(true)
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
        .p(Constants.ClimberSubsystem.LockingBarMotor.PIDF.kP)
        .i(Constants.ClimberSubsystem.LockingBarMotor.PIDF.kI)
        .d(Constants.ClimberSubsystem.LockingBarMotor.PIDF.kD)
        .iZone(Constants.ClimberSubsystem.LockingBarMotor.PIDF.kIZone)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(false)
        .positionWrappingInputRange(0,360);
  
      lockingMotorConfig.closedLoop.maxMotion
        .maxVelocity(5)
        .maxAcceleration(1)
        .allowedClosedLoopError(0.01);
  
      // apply configuration
      lockingBarMotor.configure(lockingMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  
    public void setBarAngle(double angle){
      lockingTargetPosition = MathUtil.clamp(angle, Constants.ClimberSubsystem.LockingBarMotor.kMinLimit, Constants.ClimberSubsystem.LockingBarMotor.kMaxLimit);
    }

    public void lockClimberBar(){
      double targetpostion = Constants.ClimberSubsystem.LockingBarMotor.kLockedPosition;
      setBarAngle(targetpostion);
    }

    public void unlockClimberBar(){
      double targetpostion = Constants.ClimberSubsystem.LockingBarMotor.kUnLockedPosition;
      setBarAngle(targetpostion);
    }
      //used to set climber using position buttons
    public void setClimberAngle(double angle) {
      climberTargetPosition = MathUtil.clamp(angle, Constants.ClimberSubsystem.PositionMotor.kMinAngle,  Constants.ClimberSubsystem.PositionMotor.kMaxAngle);
    } 

    public void extendClimber(){
      double targetpostion = Constants.ClimberSubsystem.PositionMotor.kDeployedAngle;
      setClimberAngle(targetpostion);
    }

    public void retractClimber(){
      double targetpostion = Constants.ClimberSubsystem.PositionMotor.kHomeAngle;
      setClimberAngle(targetpostion);
    }
      //Puts the climber in the position to be locked with the servo
    public void climberLockingPosition() {
      double targetpostion = Constants.ClimberSubsystem.PositionMotor.kLockingPosition;
      setClimberAngle(targetpostion);
    }
      //moves the climber (with the servo) to lock the climber to prepare for the loss of power.
    public void climberLockedPosition() {
      double targetpostion = Constants.ClimberSubsystem.PositionMotor.kLockedPosition;
      setClimberAngle(targetpostion);
    }

        // operater controll of the climber 
    public void changeClimberPosition(double delta){
      double newPosition = getClimberPostion() + delta;
      setClimberAngle(newPosition);
    }
    
    //TODO: This should be private so that you can't bypass the intended methods below
    public void setServoValue(double newValue){
      newValue = MathUtil.clamp(newValue, Constants.ClimberSubsystem.LatchServo.minLimit, Constants.ClimberSubsystem.LatchServo.maxLimit);
      latchingServo.set(newValue);
      
    }

    public double getServoValue() {
      return latchingServo.get();
    }

    //
    public void latchCLimber(){
      setServoValue(Constants.ClimberSubsystem.LatchServo.latchedValue);
    }

    public void unLatchCLimber(){
      setServoValue(Constants.ClimberSubsystem.LatchServo.unLatchedValue);
    }

    public void moveCLimberCommandLock(){
      moveClimberCommandLock = true;
    }
    
    public void moveCLimberCommandUnLock(){
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

      // if (Constants.kDebug == true){
      // SmartDashboard.putNumber("Locking Motor Position", getLockingMotorPosition());
      // SmartDashboard.putNumber("Climber Motor Position", getClimberPostion());
      // SmartDashboard.putNumber("Servo position", getServoAngle());
      // SmartDashboard.putBoolean("Is climber Unlocked", moveClimberCommandLock);
      // if(debugTimer.get() > 1.5) {
      //   System.out.println("Applied Position Motor Output: " + positionMotor.getAppliedOutput());
      //   System.out.println("Current Absolute Angle: " + positionMotor.getAbsoluteEncoder().getPosition());
      //   debugTimer.reset();
      // }
    //}
    
       SmartDashboard.putNumber("Locking Motor Position", getLockingMotorPosition());
       SmartDashboard.putNumber("Climber Motor Position", getClimberPostion());
       SmartDashboard.putBoolean("Is Climber locked?", moveClimberCommandLock);

      if ((RobotState.isTeleop() || RobotState.isAutonomous()) && isInitialized == true){
        double immediateTargetAngle = MathUtil.clamp(positionLimiter.calculate(climberTargetPosition), Constants.ClimberSubsystem.PositionMotor.kMinAngle, Constants.ClimberSubsystem.PositionMotor.kMaxAngle);
        positionMotor.getClosedLoopController().setReference(immediateTargetAngle, ControlType.kPosition);
  
        double lockingImmediateTargetAngle = MathUtil.clamp(lockingPositionLimiter.calculate(lockingTargetPosition), Constants.ClimberSubsystem.LockingBarMotor.kMinLimit, Constants.ClimberSubsystem.LockingBarMotor.kMaxLimit);
        lockingBarMotor.getClosedLoopController().setReference(lockingImmediateTargetAngle, ControlType.kPosition);
      }
>>>>>>> ccb55d266ba6a39506bcf2d4de652e18d5527e12
  }     
}
