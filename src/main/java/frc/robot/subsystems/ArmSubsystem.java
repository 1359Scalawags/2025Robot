package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.SimableSparkMax;
import frc.robot.extensions.SparkMaxPIDTuner;


public class ArmSubsystem extends SubsystemBase {

    private SimableSparkMax pulleyMotor,elbowMotor, wristMotor, clawMotor;

    private DigitalInput homeLimitSwitch;

    private SparkMaxPIDTuner pulleyTuner, elbowTuner, wristTuner, clawTuner;


    public ArmSubsystem() {
      pulleyMotor = new SimableSparkMax(Constants.ArmSubsystem.Pulley.kMotorID, MotorType.kBrushless);     
      elbowMotor = new SimableSparkMax(Constants.ArmSubsystem.Elbow.kMotorID, MotorType.kBrushless);       
      wristMotor = new SimableSparkMax(Constants.ArmSubsystem.Wrist.kMotorID, MotorType.kBrushless);
      clawMotor = new SimableSparkMax(Constants.ArmSubsystem.Claw.kMotorID, MotorType.kBrushless);

      configureWristMotor();
      configureElbowMotor();
      configurePulleyMotor();
      configureClawMotor();

      pulleyTuner = new SparkMaxPIDTuner("A: Pulley", pulleyMotor, ControlType.kPosition);
      elbowTuner = new SparkMaxPIDTuner("A: Elbow", elbowMotor, ControlType.kPosition);
      wristTuner = new SparkMaxPIDTuner("A: Wrist", wristMotor, ControlType.kPosition);
      clawTuner = new SparkMaxPIDTuner("A: Claw", clawMotor, ControlType.kPosition);

      homeLimitSwitch = new DigitalInput(Constants.ArmSubsystem.kHomeLimitSwitchID);

    }

    private void configureWristMotor() {
    SparkMaxConfig wristMotorConfig = new SparkMaxConfig();

      wristMotorConfig
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .openLoopRampRate(1.0)
        .closedLoopRampRate(1.0)
        .smartCurrentLimit(70, 30, 120);

      wristMotorConfig.absoluteEncoder
      .zeroOffset(Constants.ArmSubsystem.Wrist.kMotorOffset)
      .positionConversionFactor(Constants.ArmSubsystem.Wrist.kConversionFactor);
     
     
      wristMotorConfig.closedLoop
      .p(1.0f)
      .i(0.0f)
      .d(0.0)
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

      // apply configuration
      wristMotor.configure(wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

      //TODO: make sure to add ".feedbackSensor(FeedbackSensor.kAbsoluteEncoder)" to any motor with a absolute encoder
    private void configureElbowMotor() {
      SparkMaxConfig elbowMotorConfig = new SparkMaxConfig();
  
        elbowMotorConfig
          .idleMode(IdleMode.kBrake)
          .inverted(false)
          .openLoopRampRate(1.0)
          .closedLoopRampRate(1.0)
          .smartCurrentLimit(70, 30, 120);
  
        elbowMotorConfig.absoluteEncoder
        .zeroOffset(Constants.ArmSubsystem.Elbow.kMotorOffset)
        .positionConversionFactor(Constants.ArmSubsystem.Elbow.kConversionFactor);
       
       
        elbowMotorConfig.closedLoop
        .p(1.0f)
        .i(0.0f)
        .d(0.0)
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
        .smartCurrentLimit(70, 30, 120);

      pulleyMotorConfig.absoluteEncoder
      .zeroOffset(Constants.ArmSubsystem.Pulley.kMotorOffset)
      .positionConversionFactor(Constants.ArmSubsystem.Pulley.kConversionFactor);
     
     
      pulleyMotorConfig.closedLoop
      .p(1.0f)
      .i(0.0f)
      .d(0.0);

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
          .smartCurrentLimit(70, 30, 120);
  
        clawMotorConfig.absoluteEncoder
        .zeroOffset(Constants.ArmSubsystem.Claw.kMotorOffset)
        .positionConversionFactor(Constants.ArmSubsystem.Claw.kConversionFactor);
       
       
        clawMotorConfig.closedLoop
        .p(1.0f)
        .i(0.0f)
        .d(0.0)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
  
        // apply configuration
        clawMotor.configure(clawMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      }

    public void closeClaw(){
      System.err.println("closeClaw not implemented");
      //clawMotor.getClosedLoopController().setReference(Constants.ArmSubsystem.closedClawPosition, ControlType.kPosition);
    }

    @Override
    public void periodic() {
      pulleyTuner.updateEncoderValues();
      elbowTuner.updateEncoderValues();
      wristTuner.updateEncoderValues();
      clawTuner.updateEncoderValues();
     
    }
}
