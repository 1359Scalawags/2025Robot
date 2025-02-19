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
        .openLoopRampRate(Constants.ClimberSubsystem.LockingBarMotor.kSlewRate)
        .closedLoopRampRate(Constants.ClimberSubsystem.LockingBarMotor.kSlewRate)
        .smartCurrentLimit(70, 30, 120);

      lockingMotorConfig.absoluteEncoder 
        .inverted(true)
        .zeroOffset(Constants.ClimberSubsystem.LockingBarMotor.kEncoderOffset)
        .positionConversionFactor(Constants.ClimberSubsystem.LockingBarMotor.kConversionFactor);

      lockingMotorConfig.closedLoop
        .p(0.005)
        .i(0.0)
        .d(0.0)
        .iZone(0.0)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true);

      // apply configuration
      lockingBarMotor.configure(lockingMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    @Override
    public void periodic() {

    }     
}
