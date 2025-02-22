package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.extensions.SimableSparkMax;
import frc.robot.extensions.SparkMaxPIDTuner;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase{
  private SimableSparkMax lockingBarMotor;  
  private SimableSparkMax positionMotor;

  private Servo latchingServo;

  private SparkMaxPIDTuner lockBarTuner;
  private SparkMaxPIDTuner positionMotorTuner;


  public ClimberSubsystem() {
    lockingBarMotor = new SimableSparkMax(Constants.ClimberSubsystem.LockingBarMotor.kMotorID, MotorType.kBrushless, "LockingBarMotor");
    positionMotor = new SimableSparkMax(Constants.ClimberSubsystem.PositionMotor.kMotorID, MotorType.kBrushless, "postionMotor");
    latchingServo = new Servo(Constants.ClimberSubsystem.LatchServo.kServoID);

    configureLockingBarMotor();
    configurePositionMotor();
    
    lockBarTuner = new SparkMaxPIDTuner("C: Lock Bar", lockingBarMotor, ControlType.kPosition);
    //lockBarTuner = new SparkMaxPIDTuner("C: Lock Bar", lockingBarMotor, ControlType.kMAXMotionPositionControl);
    positionMotorTuner = new SparkMaxPIDTuner("C: Position", positionMotor, ControlType.kPosition);

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

    lockingMotorConfig.closedLoop.maxMotion
      .maxVelocity(5)
      .maxAcceleration(1)
      .allowedClosedLoopError(0.01);

    // apply configuration
    lockingBarMotor.configure(lockingMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }


  @Override
  public void periodic() {
    lockBarTuner.updateEncoderValues();
    positionMotorTuner.updateEncoderValues();
  }     
}
