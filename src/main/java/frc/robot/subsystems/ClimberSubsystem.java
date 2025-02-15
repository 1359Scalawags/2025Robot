package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
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
  private double positionTargetPosition;

  private boolean moveClimberCommandLock = false;

  private double curretPosition;

    public ClimberSubsystem() {
      lockingBarMotor = new SimableSparkMax(Constants.ClimberSubsystem.kLockingBarMotorID, MotorType.kBrushless, "LockingBarMotor");
      positionMotor = new SimableSparkMax(Constants.ClimberSubsystem.kPositionMotorID, MotorType.kBrushless, "postionMotor");
      latchingServo = new Servo(Constants.ClimberSubsystem.kLatchingServoID);
      lockingBarEncoder = lockingBarMotor.getAbsoluteEncoder();
      positionEncoder = positionMotor.getAbsoluteEncoder();
   
      configureLockingBarMotor();
      configurePositionMotor();
    }

    public double getLockingMotorPosition(){
      return lockingBarEncoder.getPosition();
    }

    public double getPositionMotorPostion(){
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
        .openLoopRampRate(1.0)
        .closedLoopRampRate(1.0)
        .smartCurrentLimit(70, 30, 120);

      positionMotorConfig.absoluteEncoder
      .zeroOffset(Constants.ClimberSubsystem.kPositionEncoderOffset)
      .positionConversionFactor(Constants.ClimberSubsystem.kPositionConversionFactor);
       
      positionMotorConfig.closedLoop
      .p(0.1f)
      .i(0.0f)
      .d(0.0)
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
        .zeroOffset(Constants.ClimberSubsystem.kLockingMotorOffset)
        .positionConversionFactor(Constants.ClimberSubsystem.kLockingBarMotorConversionFactor);

      lockingMotorConfig.closedLoop
        .p(0.1f)
        .i(0.0f)
        .d(0.0)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

      // apply configuration
      lockingBarMotor.configure(lockingMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void lockClimberBar(){
      lockingBarMotor.getClosedLoopController().setReference(Constants.ClimberSubsystem.barLockedPosition, ControlType.kPosition);
    }

    public void unlockClimber(){
      lockingBarMotor.getClosedLoopController().setReference(Constants.ClimberSubsystem.unlockedBarPosition, ControlType.kPosition);
    }

    public void setClimberAngle(double angle) {
      if (angle < Constants.ClimberSubsystem.maxClimberAngle && angle > Constants.ClimberSubsystem.minClimberAngle) {
        positionMotor.getClosedLoopController().setReference(angle, ControlType.kPosition);
      }
    } 

    public void extendClimber(){
      double targetpostion = Constants.ClimberSubsystem.deployedClimberAngle;
      setClimberAngle(targetpostion);
    }

    public void retractClimber(){
      double targetpostion = Constants.ClimberSubsystem.retractedClimberAngle;
      setClimberAngle(targetpostion);
    }

        // operater controll of the climber 
    public void changeClimberPosition(double delta){
      double newPosition = curretPosition + delta; 
      newPosition = MathUtil.clamp(newPosition, Constants.ClimberSubsystem.minClimberAngle, Constants.ClimberSubsystem.maxClimberAngle);
      positionMotor.getClosedLoopController().setReference(newPosition, ControlType.kPosition);
    }
    

    public void setServoAngle(double newAngle){
      if ((newAngle <= Constants.ClimberSubsystem.maxServoLimit) && (newAngle >= Constants.ClimberSubsystem.minServoLimit)){
        latchingServo.setAngle(newAngle);
      }
    }

    public void latchCLimber(){
      setServoAngle(Constants.ClimberSubsystem.servoLatchedAngle);
    }

    public void unLatchCLimber(){
      setServoAngle(Constants.ClimberSubsystem.servoUnLatchedAngle);
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

    @Override
    public void periodic() {
      SmartDashboard.putNumber("Locking Motor Position", getLockingMotorPosition());
      SmartDashboard.putNumber("Climber Motor Position", getPositionMotorPostion());
      SmartDashboard.putNumber("Servo position", getServoAngle());

  }
}
