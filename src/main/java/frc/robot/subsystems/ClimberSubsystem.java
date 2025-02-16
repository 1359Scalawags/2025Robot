package frc.robot.subsystems;

import static edu.wpi.first.units.Units.derive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.extensions.SimableSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase{
  private SimableSparkMax lockingBarMotor;  
  private SimableSparkMax positionMotor;

  private AbsoluteEncoder lockingBarEncoder;
  private AbsoluteEncoder positionEncoder;

  private Servo latchingServo;

  private double lockingTargetPosition;
  private double climberTargetPosition;

  private SlewRateLimiter positionLimiter;
  private SlewRateLimiter lockingPositionLimiter;

    //TODO: make this make sense to the drivers when it is in the dashboard
  private boolean moveClimberCommandLock = true;

  private Timer debugTimer;

  private boolean isInitialized = false;

    public ClimberSubsystem() {
      lockingBarMotor = new SimableSparkMax(Constants.ClimberSubsystem.LockingBarMotor.kMotorID, MotorType.kBrushless, "LockingBarMotor");
      positionMotor = new SimableSparkMax(Constants.ClimberSubsystem.PositionMotor.kMotorID, MotorType.kBrushless, "postionMotor");
      latchingServo = new Servo(Constants.ClimberSubsystem.LatchServo.kServoID);
      lockingBarEncoder = lockingBarMotor.getAbsoluteEncoder();
      positionEncoder = positionMotor.getAbsoluteEncoder();

      positionLimiter = new SlewRateLimiter(Constants.ClimberSubsystem.PositionMotor.kSlewRate);
      lockingPositionLimiter = new SlewRateLimiter(Constants.ClimberSubsystem.LockingBarMotor.kSlewRate);

      configureLockingBarMotor();
      configurePositionMotor();
      
      Shuffleboard.getTab("climber").add("Position Motor", positionMotor);
      Shuffleboard.getTab("climber").add("Latching Servo", latchingServo); 
      Shuffleboard.getTab("climber").add("Locking Bar", lockingBarMotor);
      Shuffleboard.getTab("climber").add("Position Motor Position",positionMotor.getEncoder().getPosition());

      debugTimer = new Timer();
      debugTimer.start();
    }

    public void initializeClimber() {
      if (isInitialized == false) {
      climberTargetPosition = Constants.ClimberSubsystem.PositionMotor.kHomeAngle;
      lockingTargetPosition = Constants.ClimberSubsystem.LockingBarMotor.kMinLimit;

      positionLimiter.reset(getClimberPostion());
      lockingPositionLimiter.reset(getLockingMotorPosition());

      positionMotor.getClosedLoopController().setReference(getClimberPostion(), ControlType.kPosition);
      lockingBarMotor.getClosedLoopController().setReference(getLockingMotorPosition(), ControlType.kPosition);


      isInitialized = true;
      moveClimberCommandLock = false;
      }
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
      .p(0.01f)
      .i(0.0)
      .d(0.0)
      .iZone(0.001)
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

      // apply configuration
      positionMotor.configure(positionMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureLockingBarMotor() {
      // create a new sparkmax config
      SparkMaxConfig lockingMotorConfig = new SparkMaxConfig();

      lockingMotorConfig
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .openLoopRampRate(1.0)
        .closedLoopRampRate(1.0)
        .smartCurrentLimit(70, 30, 120);

      lockingMotorConfig.absoluteEncoder
        .zeroOffset(Constants.ClimberSubsystem.LockingBarMotor.kEncoderOffset)
        .positionConversionFactor(Constants.ClimberSubsystem.LockingBarMotor.kConversionFactor);

      lockingMotorConfig.closedLoop
        .p(0.00001f)
        .i(0.0f)
        .d(0.0)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

      // apply configuration
      lockingBarMotor.configure(lockingMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setBarAngle(double angle){
      lockingTargetPosition = MathUtil.clamp(angle, Constants.ClimberSubsystem.LockingBarMotor.kMinLimit, Constants.ClimberSubsystem.LockingBarMotor.kMaxLimit);
      lockingBarMotor.getClosedLoopController().setReference(angle, ControlType.kPosition);
    }

    public void lockClimberBar(){
      double targetpostion = Constants.ClimberSubsystem.LockingBarMotor.kLockedPosition;
      setBarAngle(targetpostion);
    }

    public void unlockClimberBar(){
      double targetpostion = Constants.ClimberSubsystem.LockingBarMotor.kUnLockedPosition;
      setBarAngle(targetpostion);
    }
      //used to set climber using the  buttons
      //TODO: add clamps to specific functions.
    public void setClimberAngle(double angle) {
      climberTargetPosition = MathUtil.clamp(angle, Constants.ClimberSubsystem.PositionMotor.kMinAngle,  Constants.ClimberSubsystem.PositionMotor.kMaxAngle);
      // if (angle < Constants.ClimberSubsystem.PositionMotor.kMaxAngle && angle > Constants.ClimberSubsystem.PositionMotor.kMinAngle) {
      //positionMotor.getClosedLoopController().setReference(angle, ControlType.kPosition);
      // }
    } 

    public void extendClimber(){
      double targetpostion = Constants.ClimberSubsystem.PositionMotor.kDeployedAngle;
      setClimberAngle(targetpostion);
    }

    public void retractClimber(){
      double targetpostion = Constants.ClimberSubsystem.PositionMotor.kHomeAngle;
      setClimberAngle(targetpostion);
    }

        // operater controll of the climber 
    public void changeClimberPosition(double delta){
      // positionMotor.getClosedLoopController().setReference(newPosition, ControlType.kPosition);
      //MathUtil.clamp(newPosition, Constants.ClimberSubsystem.PositionMotor.kMinAngle, Constants.ClimberSubsystem.PositionMotor.kMaxAngle);
      double newPosition = getClimberPostion() + delta;
      setClimberAngle(newPosition);
    }
    

    public void setServoAngle(double newAngle){
      if ((newAngle <= Constants.ClimberSubsystem.LatchServo.maxLimit) && (newAngle >= Constants.ClimberSubsystem.LatchServo.minLimit)){
        latchingServo.setAngle(newAngle);
      }
    }

    public void latchCLimber(){
      setServoAngle(Constants.ClimberSubsystem.LatchServo.latchedAngle);
    }

    public void unLatchCLimber(){
      setServoAngle(Constants.ClimberSubsystem.LatchServo.unLatchedAngle);
    }

    public void moveCLimberCommandLock(){
      moveClimberCommandLock = false;
    }
    
    public void moveCLimberCommandUnLock(){
      moveClimberCommandLock = true;
    }

    public boolean isClimberCommandLocked() {
      return moveClimberCommandLock;
    }

    public boolean isClimberLocked() {
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

      if (Constants.kDebug == true){
      SmartDashboard.putNumber("Locking Motor Position", getLockingMotorPosition());
      SmartDashboard.putNumber("Climber Motor Position", getClimberPostion());
      SmartDashboard.putNumber("Servo position", getServoAngle());
      SmartDashboard.putBoolean("Is climber Unlocked", moveClimberCommandLock);
      if(debugTimer.get() > 1.5) {
        System.out.println("Applied Position Motor Output: " + positionMotor.getAppliedOutput());
        System.out.println("Current Absolute Angle: " + positionMotor.getAbsoluteEncoder().getPosition());
        debugTimer.reset();
      }
    }
    
      if ((RobotState.isTeleop() || RobotState.isAutonomous()) && isInitialized == true){
        double immediateTargetAngle = positionLimiter.calculate(climberTargetPosition);
        positionMotor.getClosedLoopController().setReference(immediateTargetAngle, ControlType.kPosition);
  
        double lockingImmediateTargetAngle = lockingPositionLimiter.calculate(lockingTargetPosition);
        lockingBarMotor.getClosedLoopController().setReference(lockingImmediateTargetAngle, ControlType.kPosition);
      }
  }     
}
