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
    private SimableSparkMax clawMotor; // TODO : make this do all the things is needs to do (make gavan do it)
    private static double ARM_HEIGHT; 


    public ArmSubsystem() {
      wristMotor = new SimableSparkMax(Constants.ArmSubsystem.kWristMotorID,MotorType.kBrushless);
      elbowMotor = new SimableSparkMax(Constants.ArmSubsystem.kArmMotorID,MotorType.kBrushless);
      pulleyMotor = new SimableSparkMax(Constants.ArmSubsystem.kPulleyMotorID,MotorType.kBrushless);
      clawMotor = new SimableSparkMax(Constants.ArmSubsystem.kReversedScrewMotorID,MotorType.kBrushless);

      configureWristMotor();
      configureArmMotor();
      configurePulleyMotor();
      configureClawMotor();
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


    private void configureClawMotor() {
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

    public void closeClaw(){
      System.err.println("closeClaw not implemented");
      //clawMotor.getClosedLoopController().setReference(Constants.ArmSubsystem.closedClawPosition, ControlType.kPosition);
    }

    public void openClaw(){
      System.err.println("openClaw not implemented");
      //clawMotor.getClosedLoopController().setReference(Constants.ArmSubsystem.openedClawPosition, ControlType.kPosition);
    }

    public void goToPulleyMotorPosition(double pulleyMotorPosition) {
      if (pulleyMotorPosition < Constants.ArmSubsystem.maxPulleyLimit && pulleyMotorPosition > Constants.ArmSubsystem.minPulleyLimit) {
        pulleyMotor.getClosedLoopController().setReference(pulleyMotorPosition, ControlType.kPosition);
      }
    } 

    public void goToElbowMotorPosition(double elbowMotorPosition) {
      if (elbowMotorPosition < Constants.ArmSubsystem.maxElbowLimit && elbowMotorPosition > Constants.ArmSubsystem.minElbowLimit) {
        elbowMotor.getClosedLoopController().setReference(elbowMotorPosition, ControlType.kPosition);
      }
    } 

    public void goToWristMotorPosition(double wristMotorPosition) {
      if (wristMotorPosition < Constants.ArmSubsystem.maxWristLimit && wristMotorPosition > Constants.ArmSubsystem.minWristLimit) {
        wristMotor.getClosedLoopController().setReference(wristMotorPosition, ControlType.kPosition);
      }
    }
    
    //Sets arm height to the ground
    public void goToHeightGround(){
      double heightPos = Constants.ArmSubsystem.kGroundHeight;
      goToPulleyMotorPosition(heightPos);
    }
    //Sets arm height to Level Two
    public void goToHeightL2(){
      double heightPos = Constants.ArmSubsystem.kL2Height;
      goToPulleyMotorPosition(heightPos);
    }
    //Sets arm height to Level Three
    public void goToHeightL3(){
      double heightPos = Constants.ArmSubsystem.kL3Height;
      goToPulleyMotorPosition(heightPos);
    }
    //Sets arm height to Level Four
    public void goToHeightL4(){
      double heightPos = Constants.ArmSubsystem.kL4Height;
      goToPulleyMotorPosition(heightPos);
    }
    //Sets arm height to the Human Station
    public void goToHeightHumanStation(){
      double heightPos = Constants.ArmSubsystem.kHeightHumanStation;
      goToPulleyMotorPosition(heightPos);

    }

    //Sets arm position to the Ground
    public void goToArmGround(){
      double elbowPos = Constants.ArmSubsystem.kElbowPosGround;
      double wristPos = Constants.ArmSubsystem.kWristPosGround;
      goToElbowMotorPosition(elbowPos);
      goToWristMotorPosition(wristPos);
    }
    //Sets arm position to Level Two
    public void goToArmL2(){
      double elbowPos = Constants.ArmSubsystem.kElbowPosL2;
      double wristPos = Constants.ArmSubsystem.kWristPosL2;
      goToElbowMotorPosition(elbowPos);
      goToWristMotorPosition(wristPos);
    }
    //Sets arm postion to Level Three
    public void goToArmL3(){
      double elbowPos = Constants.ArmSubsystem.kElbowPosL3;
      double wristPos = Constants.ArmSubsystem.kWristPosL3;
      goToElbowMotorPosition(elbowPos);
      goToWristMotorPosition(wristPos);
    }
    //Sets arm position to Level Four
    public void goToArmL4(){
      double elbowPos = Constants.ArmSubsystem.kElbowPosL4;
      double wristPos = Constants.ArmSubsystem.kWristPosL4;
      goToElbowMotorPosition(elbowPos);
      goToWristMotorPosition(wristPos);
    }
    //Sets arm position to the Human Station
    public void goToArmHumanStation(){
      double elbowPos = Constants.ArmSubsystem.kElbowPosHumanStation;
      double wristPos = Constants.ArmSubsystem.kWristPosHumanStation;
      goToElbowMotorPosition(elbowPos);
      goToWristMotorPosition(wristPos);
    }

    public static double getArmHeight(){
      return ARM_HEIGHT;
    }

    public double getElbowMotorPosition(){
      return elbowMotor.getEncoder().getPosition();
    }
    
    public double getWristMotorPosition(){
      return wristMotor.getEncoder().getPosition();
    }
    
    //TODO : Get formula
       public double getCalculatedHeight(){
        return pulleyMotor.getEncoder().getPosition(); 
       }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        ARM_HEIGHT = getCalculatedHeight();
    }
}
