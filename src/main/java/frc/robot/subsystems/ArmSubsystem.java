package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.ArmPosition;
import frc.robot.extensions.SimableSparkMax;


public class ArmSubsystem extends SubsystemBase {

    private SimableSparkMax wristMotor; 
    private SimableSparkMax elbowMotor;
    private SimableSparkMax pulleyMotor;
    private SimableSparkMax clawMotor;
    private static double ARM_HEIGHT; 


    public ArmSubsystem() {
      wristMotor = new SimableSparkMax(Constants.ArmSubsystem.Wrist.kMotorID,MotorType.kBrushless);
      elbowMotor = new SimableSparkMax(Constants.ArmSubsystem.Elbow.kMotorID,MotorType.kBrushless);
      pulleyMotor = new SimableSparkMax(Constants.ArmSubsystem.Pulley.kMotorID,MotorType.kBrushless);
      clawMotor = new SimableSparkMax(Constants.ArmSubsystem.Claw.kMotorID,MotorType.kBrushless);

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
      .zeroOffset(Constants.ArmSubsystem.Wrist.kMotorOffset)
      .positionConversionFactor(Constants.ArmSubsystem.Wrist.kConversionFactor);
     
      wristMotorConfig.closedLoop
      .p(1.0f)
      .i(0.0f)
      .d(0.0);

      // apply configuration
      wristMotor.configure(wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

      //TODO: make sure to add ".feedbackSensor(FeedbackSensor.kAbsoluteEncoder)" to any motor with a absolute encoder
    private void configureArmMotor() {
      SparkMaxConfig elbowMotorConfig = new SparkMaxConfig();
  
        elbowMotorConfig
          .idleMode(IdleMode.kBrake)
          .inverted(false)
          .openLoopRampRate(1.0)
          .closedLoopRampRate(1.0)
          .smartCurrentLimit(70, 30, 120);
  
        elbowMotorConfig.absoluteEncoder
        .zeroOffset(Constants.ArmSubsystem.Wrist.kMotorOffset)
        .positionConversionFactor(Constants.ArmSubsystem.Wrist.kConversionFactor);
       
       
        elbowMotorConfig.closedLoop
        .p(1.0f)
        .i(0.0f)
        .d(0.0);
  
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
        .positionConversionFactor(Constants.ArmSubsystem.Claw.kCnversionFactor);
       
       
        clawMotorConfig.closedLoop
        .p(1.0f)
        .i(0.0f)
        .d(0.0);
  
        // apply configuration
        clawMotor.configure(clawMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
      if (pulleyMotorPosition < Constants.ArmSubsystem.Pulley.kMaxLimit && pulleyMotorPosition > Constants.ArmSubsystem.Pulley.kMinLimit) {
        pulleyMotor.getClosedLoopController().setReference(pulleyMotorPosition, ControlType.kPosition);
      }
    } 

    public void goToElbowMotorPosition(double elbowMotorPosition) {
      if (elbowMotorPosition < Constants.ArmSubsystem.Elbow.kMaxLimit && elbowMotorPosition > Constants.ArmSubsystem.Elbow.kMinLimit) {
        elbowMotor.getClosedLoopController().setReference(elbowMotorPosition, ControlType.kPosition);
      }
    } 

    public void goToWristMotorPosition(double wristMotorPosition) {
      if (wristMotorPosition < Constants.ArmSubsystem.Wrist.kMaxLimit && wristMotorPosition > Constants.ArmSubsystem.Wrist.kMinLimit) {
        wristMotor.getClosedLoopController().setReference(wristMotorPosition, ControlType.kPosition);
      }
    }

    ///*
    /// XXX: Example of method to set arm position in one function call
    ///      Calling them all individually is very verbose
    /// */
    public void goToPosition(ArmPosition position) {
      goToPulleyMotorPosition(position.pulley);
      goToElbowMotorPosition(position.elbow);
      goToWristMotorPosition(position.wrist);
    }
    
    //Sets arm height to the ground
    public void goToHeightGround(){
      double heightPos = Constants.ArmSubsystem.Positions.kGround.pulley;
      goToPulleyMotorPosition(heightPos);
    }
    //Sets arm height to Level Two
    public void goToHeightL2(){
      double heightPos = Constants.ArmSubsystem.Positions.kLevel2.pulley;
      goToPulleyMotorPosition(heightPos);
    }
    //Sets arm height to Level Three
    public void goToHeightL3(){
      double heightPos = Constants.ArmSubsystem.Positions.kLevel3.pulley;
      goToPulleyMotorPosition(heightPos);
    }
    //Sets arm height to Level Four
    public void goToHeightL4(){
      double heightPos = Constants.ArmSubsystem.Positions.kLevel4.pulley;
      goToPulleyMotorPosition(heightPos);
    }
    //Sets arm height to the Human Station
    public void goToHeightHumanStation(){
      double heightPos = Constants.ArmSubsystem.Positions.kHumanStation.pulley;
      goToPulleyMotorPosition(heightPos);

    }

    //Sets arm position to the Ground
    public void goToArmGround(){
      goToElbowMotorPosition(Constants.ArmSubsystem.Positions.kGround.elbow);
      goToWristMotorPosition(Constants.ArmSubsystem.Positions.kGround.wrist);
    }
    //Sets arm position to Level Two
    public void goToArmL2(){
      goToElbowMotorPosition(Constants.ArmSubsystem.Positions.kLevel2.elbow);
      goToWristMotorPosition(Constants.ArmSubsystem.Positions.kLevel2.wrist);
    }
    //Sets arm postion to Level Three
    public void goToArmL3(){
      goToElbowMotorPosition(Constants.ArmSubsystem.Positions.kLevel3.elbow);
      goToWristMotorPosition(Constants.ArmSubsystem.Positions.kLevel3.wrist);
    }
    //Sets arm position to Level Four
    public void goToArmL4(){
      goToElbowMotorPosition(Constants.ArmSubsystem.Positions.kLevel4.elbow);
      goToWristMotorPosition(Constants.ArmSubsystem.Positions.kLevel4.wrist);
    }
    //Sets arm position to the Human Station
    public void goToArmHumanStation(){
      goToElbowMotorPosition(Constants.ArmSubsystem.Positions.kHumanStation.elbow);
      goToWristMotorPosition(Constants.ArmSubsystem.Positions.kHumanStation.wrist);
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
