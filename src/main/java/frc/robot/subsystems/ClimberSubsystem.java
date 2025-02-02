package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.extensions.SendableCANSparkMax;
import frc.robot.extensions.SimableSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase{
  private SimableSparkMax lockingMotor;  
  private SimableSparkMax positionMotor;

  private AbsoluteEncoder lockingEncoder;
  private AbsoluteEncoder positionEncoder;

  private Servo latchingServo;

  private double lockingTargetPosition;
  private double positionTargetPosition;

  private double curretPosition = positionEncoder.getPosition();

    public ClimberSubsystem() {
      lockingMotor = new SimableSparkMax(Constants.ClimberSubsystem.kLockingMotorID, MotorType.kBrushless, "LockingMotor");
      positionMotor = new SimableSparkMax(Constants.ClimberSubsystem.kPositionMotorID, MotorType.kBrushless, "postionMotor");
      latchingServo = new Servo(Constants.ClimberSubsystem.kLatchingServoID);
      lockingEncoder = lockingMotor.getAbsoluteEncoder();
      positionEncoder = positionMotor.getAbsoluteEncoder();
   
      configureLockingMotor();
      configurePositionMotor();
    }

    public double getLockingMotorPosition(){
      return lockingEncoder.getPosition();
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
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .openLoopRampRate(1.0)
        .closedLoopRampRate(1.0)
        .smartCurrentLimit(70, 30, 120);

      positionMotorConfig.absoluteEncoder
      .zeroOffset(Constants.ClimberSubsystem.kPositionEncoderOffset)
      .positionConversionFactor(Constants.ClimberSubsystem.kPositionConversionFactor);
       
      positionMotorConfig.closedLoop
      .p(1.0f)
      .i(0.0f)
      .d(0.0);

      // apply configuration
      positionMotor.configure(positionMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureLockingMotor() {
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
        .positionConversionFactor(Constants.ClimberSubsystem.kLockingMotorConversionFactor);

      lockingMotorConfig.closedLoop
        .p(1.0f)
        .i(0.0f)
        .d(0.0);

      // apply configuration
      lockingMotor.configure(lockingMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void lockClimber(){
      lockingMotor.getClosedLoopController().setReference(Constants.ClimberSubsystem.lockedPosition, ControlType.kPosition);
    }

    public void unlockClimber(){
      lockingMotor.getClosedLoopController().setReference(Constants.ClimberSubsystem.unlockedPosition, ControlType.kPosition);
    }

    public void setClimberAngle(double angle) {
      if (angle < Constants.ClimberSubsystem.maxClimberAngle && angle > Constants.ClimberSubsystem.minClimberAngle) {
        positionMotor.getClosedLoopController().setReference(angle, ControlType.kPosition);
      }
    } 

    public void extendClimber(){
      double targetpostion = Constants.ClimberSubsystem.extendedClimberAngle;
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


    @Override
    public void periodic() {
      SmartDashboard.putNumber("Locking Motor Position", getLockingMotorPosition());
      SmartDashboard.putNumber("Climber Motor Position", getPositionMotorPostion());
  }
}
