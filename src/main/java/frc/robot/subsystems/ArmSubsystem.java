package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.ArmPosition;
import frc.robot.extensions.SimableSparkMax;


public class ArmSubsystem extends SubsystemBase {

  // variables for motors with positional control
  private SimableSparkMax pulleyMotor, elbowMotor, wristMotor, clawMotor;
  private SlewRateLimiter pulleyLimiter, elbowLimiter, wristLimiter, clawLimiter;    
  private double wristMotorTarget, elbowMotorTarget, pulleyMotorTarget, clawMotorTarget;

  // there will be a limit switch to calibrate the zero position of the elevator 
  private DigitalInput homeLimitSwitch;
  private static double ARM_HEIGHT; 

  // has the arm been properly initialized?
  private boolean initialized = false;

  public ArmSubsystem() {

    // setup the motors
    pulleyMotor = new SimableSparkMax(Constants.ArmSubsystem.Pulley.kMotorID,MotorType.kBrushless);     
    elbowMotor = new SimableSparkMax(Constants.ArmSubsystem.Elbow.kMotorID,MotorType.kBrushless);       
    wristMotor = new SimableSparkMax(Constants.ArmSubsystem.Wrist.kMotorID,MotorType.kBrushless);
    clawMotor = new SimableSparkMax(Constants.ArmSubsystem.Claw.kMotorID,MotorType.kBrushless);

    pulleyLimiter = new SlewRateLimiter(Constants.ArmSubsystem.Pulley.kSlewRate);
    elbowLimiter = new SlewRateLimiter(Constants.ArmSubsystem.Elbow.kSlewRate);
    wristLimiter = new SlewRateLimiter(Constants.ArmSubsystem.Wrist.kSlewRate);
    clawLimiter = new SlewRateLimiter(Constants.ArmSubsystem.Claw.kSlewRate);

    configureWristMotor();
    configureArmMotor();
    configurePulleyMotor();
    configureClawMotor();    

    // setup limit switch
    homeLimitSwitch = new DigitalInput(Constants.ArmSubsystem.kHomeLimitSwitchID);

  }

  /**
   * XXX: Initializes motor targets and the slew rate limiters.
   * @param findLimits Should the initialization routine search for the lower limit?
   */
  public void initializeArm(boolean findLimits) {
    if(!initialized) {
      pulleyMotorTarget = pulleyMotor.getEncoder().getPosition();
      elbowMotorTarget = elbowMotor.getEncoder().getPosition();
      wristMotorTarget = wristMotor.getEncoder().getPosition();
      clawMotorTarget = clawMotor.getEncoder().getPosition();

      pulleyLimiter.reset(pulleyMotor.getEncoder().getPosition());
      elbowLimiter.reset(elbowMotor.getEncoder().getPosition());
      wristLimiter.reset(wristMotor.getEncoder().getPosition());
      clawLimiter.reset(clawMotor.getEncoder().getPosition());

      //If looking for limit, start the elevator moving down at a safe speed
      if(findLimits) {
        pulleyMotor.getClosedLoopController().setReference(Constants.ArmSubsystem.Pulley.kHomingVelocity, ControlType.kVelocity);        
      } else {
        pulleyMotor.getClosedLoopController().setReference(pulleyMotorTarget, ControlType.kPosition);
      }

      //set the target of other motors to current position
      elbowMotor.getClosedLoopController().setReference(elbowMotorTarget, ControlType.kPosition);
      wristMotor.getClosedLoopController().setReference(wristMotorTarget, ControlType.kPosition);
      clawMotor.getClosedLoopController().setReference(clawMotorTarget, ControlType.kPosition);
    }
  }

  // XXX: Fixed mismatched motor and config names
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
    // TODO: add feedback sensor?

    // apply configuration
    wristMotor.configure(wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // XXX: Fixed mismatched motor and config names
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
      // TODO: add feedback sensor?


    // apply configuration
    elbowMotor.configure(elbowMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // XXX: Fixed mismatched motor and config names
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
    clawMotorTarget = Constants.ArmSubsystem.Claw.kClosedPosition;
  }

  public void openClaw(){
    clawMotorTarget = Constants.ArmSubsystem.Claw.kOpenedPosition;
  }

  public void goToPulleyMotorPosition(double pulleyMotorPosition) {
    pulleyMotorTarget = MathUtil.clamp(pulleyMotorPosition, Constants.ArmSubsystem.Pulley.kMinLimit, Constants.ArmSubsystem.Pulley.kMaxLimit);
  } 

  public void goToElbowMotorPosition(double elbowMotorPosition) {
    elbowMotorTarget = MathUtil.clamp(elbowMotorPosition, Constants.ArmSubsystem.Elbow.kMinLimit, Constants.ArmSubsystem.Elbow.kMaxLimit);
  } 

  public void goToWristMotorPosition(double wristMotorPosition) {
    wristMotorTarget = MathUtil.clamp(wristMotorPosition, Constants.ArmSubsystem.Wrist.kMinLimit, Constants.ArmSubsystem.Wrist.kMaxLimit);
  }

  /**
   * XXX: Set preconfigured positions of the arm motors in a single function call
   * @param position
   */
  public void goToPosition(ArmPosition position) {
    goToPulleyMotorPosition(position.pulley);
    goToElbowMotorPosition(position.elbow);
    goToWristMotorPosition(position.wrist);
  }

  public ArmPosition getArmPosition() {
    return new ArmPosition(getArmHeight(), getElbowMotorPosition(), getWristMotorPosition());
  }
  
  //Sets arm height to the ground
  public void goToHeightGround(){
    goToPulleyMotorPosition(Constants.ArmSubsystem.Positions.kGround.pulley);
  }
  //Sets arm height to Level Two
  public void goToHeightL2(){
    goToPulleyMotorPosition(Constants.ArmSubsystem.Positions.kLevel2.pulley);
  }
  //Sets arm height to Level Three
  public void goToHeightL3(){
    goToPulleyMotorPosition(Constants.ArmSubsystem.Positions.kLevel3.pulley);
  }
  //Sets arm height to Level Four
  public void goToHeightL4(){
    goToPulleyMotorPosition(Constants.ArmSubsystem.Positions.kLevel4.pulley);
  }
  //Sets arm height to the Human Station
  public void goToHeightHumanStation(){
    goToPulleyMotorPosition(Constants.ArmSubsystem.Positions.kHumanStation.pulley);
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

      // XXX: if the limit switch is pressed, home the pulley.
      if(homeLimitSwitch.get() == Constants.ArmSubsystem.Pulley.kLimitSwitchPressedState) {       
        pulleyMotor.stopMotor();
        pulleyMotor.getEncoder().setPosition(0);
        pulleyLimiter.reset(0);        

        // start going home
        pulleyMotorTarget = Constants.ArmSubsystem.Positions.kHome.pulley;
        double homeTarget = pulleyLimiter.calculate(Constants.ArmSubsystem.Positions.kHome.pulley);
        pulleyMotor.getClosedLoopController().setReference(homeTarget, ControlType.kPosition);

        // consider arm initialized
        initialized = true;
      }
      else if(RobotState.isTeleop() || RobotState.isAutonomous()) {
        // These use the updated SendableCanSparkMax with a custom function to perform this task more directly
        pulleyMotor.setReferencePosition(pulleyLimiter, pulleyMotorTarget);
        elbowMotor.setReferencePosition(elbowLimiter, elbowMotorTarget);
        wristMotor.setReferencePosition(wristLimiter, wristMotorTarget);
        clawMotor.setReferencePosition(clawLimiter, clawMotorTarget);
      } 

  }
}
