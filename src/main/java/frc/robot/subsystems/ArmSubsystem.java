package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.SendableCANSparkMax;
import frc.robot.extensions.SimableSparkMax;


public class ArmSubsystem extends SubsystemBase {

    private SimableSparkMax wristMotor; 
    private SimableSparkMax elbowMotor;
    private SimableSparkMax pulleyMotor;
    private SimableSparkMax reversedScrewMotor; //TODO: What does this motor do as opposed to the specific hardware used to do it?

    public ArmSubsystem() {
      wristMotor = new SimableSparkMax(Constants.ArmSubsystem.kWristMotorID,MotorType.kBrushless);
      elbowMotor = new SimableSparkMax(Constants.ArmSubsystem.kArmMotorID,MotorType.kBrushless);
      pulleyMotor = new SimableSparkMax(Constants.ArmSubsystem.kPulleyMotorID,MotorType.kBrushless);
      reversedScrewMotor = new SimableSparkMax(Constants.ArmSubsystem.kReversedScrewMotorID,MotorType.kBrushless);

      configureWristMotor();
      configureArmMotor();
      configurePulleyMotor();
      configureReversedScrewMotor();
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
      .zeroOffset(Constants.ArmSubsystem.kWristMotorOffset)
      .positionConversionFactor(Constants.ArmSubsystem.kWristConversionFactor);
     
     
      wristMotorConfig.closedLoop
      .p(1.0f)
      .i(0.0f)
      .d(0.0);

      // apply configuration
      wristMotor.configure(wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    private void configureArmMotor() {
      SparkMaxConfig armMotorConfig = new SparkMaxConfig();
  
        armMotorConfig
          .idleMode(IdleMode.kBrake)
          .inverted(false)
          .openLoopRampRate(1.0)
          .closedLoopRampRate(1.0)
          .smartCurrentLimit(70, 30, 120);
  
        armMotorConfig.absoluteEncoder
        .zeroOffset(Constants.ArmSubsystem.kWristMotorOffset)
        .positionConversionFactor(Constants.ArmSubsystem.kWristConversionFactor);
       
       
        armMotorConfig.closedLoop
        .p(1.0f)
        .i(0.0f)
        .d(0.0);
  
        // apply configuration
        wristMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
      .zeroOffset(Constants.ArmSubsystem.kPulleyMotorOffset)
      .positionConversionFactor(Constants.ArmSubsystem.kPulleyConversionFactor);
     
     
      pulleyMotorConfig.closedLoop
      .p(1.0f)
      .i(0.0f)
      .d(0.0);

      // apply configuration
      wristMotor.configure(pulleyMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    private void configureReversedScrewMotor() {
      SparkMaxConfig reversedScrewMotorConfig = new SparkMaxConfig();
  
        reversedScrewMotorConfig
          .idleMode(IdleMode.kBrake)
          .inverted(false)
          .openLoopRampRate(1.0)
          .closedLoopRampRate(1.0)
          .smartCurrentLimit(70, 30, 120);
  
        reversedScrewMotorConfig.absoluteEncoder
        .zeroOffset(Constants.ArmSubsystem.kReversedScrewMotorOffset)
        .positionConversionFactor(Constants.ArmSubsystem.kReversedScrewConversionFactor);
       
       
        reversedScrewMotorConfig.closedLoop
        .p(1.0f)
        .i(0.0f)
        .d(0.0);
  
        // apply configuration
        wristMotor.configure(reversedScrewMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      }


    public void setPulleyMotorPosition(double pulleyMotorPosition) {
      if (pulleyMotorPosition < Constants.ArmSubsystem.maxPulleyLimit && pulleyMotorPosition > Constants.ArmSubsystem.minPulleyLimit) {
        pulleyMotor.getClosedLoopController().setReference(pulleyMotorPosition, ControlType.kPosition);
      }
    } 

    public void setElbowMotorPosition(double elbowMotorPosition) {
      if (elbowMotorPosition < Constants.ArmSubsystem.maxElbowLimit && elbowMotorPosition > Constants.ArmSubsystem.minElbowLimit) {
        elbowMotor.getClosedLoopController().setReference(elbowMotorPosition, ControlType.kPosition);
      }
    } 

    public void setWristMotorPosition(double wristMotorPosition) {
      if (wristMotorPosition < Constants.ArmSubsystem.maxWristLimit && wristMotorPosition > Constants.ArmSubsystem.minWristLimit) {
        wristMotor.getClosedLoopController().setReference(wristMotorPosition, ControlType.kPosition);
      }
    }
    
    
    public void setHeightGround(){
      double heightPos = Constants.ArmSubsystem.kHeightGround;
      setPulleyMotorPosition(heightPos);
    }

    public void setHeightL2(){
      double heightPos = Constants.ArmSubsystem.kL2Height;
      setPulleyMotorPosition(heightPos);
    }

    public void setHeightL3(){
      double heightPos = Constants.ArmSubsystem.kL3Height;
      setPulleyMotorPosition(heightPos);
    }

    public void setHeightL4(){
      double heightPos = Constants.ArmSubsystem.kL4Height;
      setPulleyMotorPosition(heightPos);
    }

    public void setHeightHumanStation(){
      double heightPos = Constants.ArmSubsystem.kHeightHumanStation;
      setPulleyMotorPosition(heightPos);

    }

    public void setArmGround(){
      double elbowPos = Constants.ArmSubsystem.kElbowPosGround;
      double wristPos = Constants.ArmSubsystem.kWristPosGround;
      setElbowMotorPosition(elbowPos);
      setWristMotorPosition(wristPos);
    }

    public void setArmL2(){
      double elbowPos = Constants.ArmSubsystem.kElbowPosL2;
      double wristPos = Constants.ArmSubsystem.kWristPosL2;
      setElbowMotorPosition(elbowPos);
      setWristMotorPosition(wristPos);
    }

    public void setArmGroundL3(){
      double elbowPos = Constants.ArmSubsystem.kElbowPosL3;
      double wristPos = Constants.ArmSubsystem.kWristPosL3;
      setElbowMotorPosition(elbowPos);
      setWristMotorPosition(wristPos);
    }

    public void setArmGroundL4(){
      double elbowPos = Constants.ArmSubsystem.kElbowPosL4;
      double wristPos = Constants.ArmSubsystem.kWristPosL4;
      setElbowMotorPosition(elbowPos);
      setWristMotorPosition(wristPos);
    }

    public void setArmHumanStation(){
      double elbowPos = Constants.ArmSubsystem.kElbowPosHumanStation;
      double wristPos = Constants.ArmSubsystem.kWristPosHumanStation;
      setElbowMotorPosition(elbowPos);
      setWristMotorPosition(wristPos);
    }

    


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
