package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Servo;
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


    public ClimberSubsystem() {
      lockingMotor = new SimableSparkMax(Constants.ClimberSubsystem.kLockingMotorID, MotorType.kBrushless);
      positionMotor = new SimableSparkMax(Constants.ClimberSubsystem.kPositionMotorID, MotorType.kBrushless, "postionMotor");
      latchingServo = new Servo(1);
      lockingEncoder = lockingMotor.getAbsoluteEncoder();
      positionEncoder = positionMotor.getAbsoluteEncoder();

      configureLockingMotor();
      configurePositionMotor();
    }

    public double getServoAngle(){
      return latchingServo.getAngle();
    }

    public void setServoAngle(double newAngle){
      if ((newAngle <= Constants.ClimberSubsystem.maxServoLimit) && (newAngle >= Constants.ClimberSubsystem.minServoLimit)){
        latchingServo.setAngle(newAngle);
      }
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

    @Override
    public void periodic() {

      }
}
