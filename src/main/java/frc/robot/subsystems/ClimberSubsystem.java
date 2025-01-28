package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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

    public void setServoAngle(double newAngle){
      latchingServo.setAngle(newAngle);
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
      SparkMaxConfig babyLockingMotorConfig = new SparkMaxConfig();

      babyLockingMotorConfig
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .openLoopRampRate(1.0)
        .closedLoopRampRate(1.0)
        .smartCurrentLimit(70, 30, 120);

      babyLockingMotorConfig.absoluteEncoder
        .zeroOffset(Constants.ClimberSubsystem.kBabyLockingMotorOffset)
        .positionConversionFactor(Constants.ClimberSubsystem.kBabyLockingMotorConversionFactor);

      babyLockingMotorConfig.closedLoop
        .p(1.0f)
        .i(0.0f)
        .d(0.0);

      // apply configuration
      lockingMotor.configure(babyLockingMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void deployLockingMotorPosition(double lockingMotorPosition) {
      if (lockingMotorPosition < Constants.ClimberSubsystem.maxLockLimit && lockingMotorPosition > Constants.ClimberSubsystem.minLockLimit) {
        lockingMotor.getClosedLoopController().setReference(lockingMotorPosition, ControlType.kPosition);
      }
    } 

    public void deployPositioMotor(double lockingMotorPosition) {
      if (lockingMotorPosition < Constants.ClimberSubsystem.maxLockLimit && lockingMotorPosition > Constants.ClimberSubsystem.minLockLimit) {
        lockingMotor.getClosedLoopController().setReference(lockingMotorPosition, ControlType.kPosition);
      }
    } 

    @Override
    public void periodic() {
      SmartDashboard.putNumber("Locking Motor Position", getLockingMotorPosition());
      SmartDashboard.putNumber("Climber Motor Position", getPositionMotorPostion());
  }
}
