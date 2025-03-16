package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.GravityAssistedFeedForward;

public class ArmSubsystem extends SubsystemBase {

  private static double ARM_HEIGHT;  
  
  private SparkMax pulleyMotor, elbowMotor, wristMotor, clawMotor;
  private double pulleyMotorTarget, elbowMotorTarget, wristMotorTarget, clawMotorTarget;


  private DigitalInput homeLimitSwitch, clawLimitSwitch;

  private boolean initialized = false;
  private boolean elbowError = true, wristError = true;

  private GravityAssistedFeedForward elbowFF, wristFF;

  // Trapezoidal profiling for elbow
  private TrapezoidProfile elbowProfile;
  private State elbowStateGoal;
  private State elbowStateSetpoint;

  // Trapezoidal profiling for wrist
  private TrapezoidProfile wristProfile;
  private State wristStateGoal;
  private State wristStateSetpoint;


  // Trapezoidal profiling for claw
  private TrapezoidProfile clawProfile;
  private State clawStateGoal;
  private State clawStateSetpoint;

  // Trapezoidal profiling for pulley
  private TrapezoidProfile pulleyProfile;
  private State pulleyStateGoal;
  private State pulleyStateSetpoint;

  
  public ArmSubsystem() {

    pulleyMotor = new SparkMax(Constants.ArmSubsystem.Pulley.kMotorID, MotorType.kBrushless);
    elbowMotor = new SparkMax(Constants.ArmSubsystem.Elbow.kMotorID, MotorType.kBrushless);
    wristMotor = new SparkMax(Constants.ArmSubsystem.Wrist.kMotorID, MotorType.kBrushless);
    clawMotor = new SparkMax(Constants.ArmSubsystem.Claw.kMotorID, MotorType.kBrushless);

    homeLimitSwitch = new DigitalInput(Constants.ArmSubsystem.Pulley.kHomeLimitSwitchID);
    clawLimitSwitch = new DigitalInput(Constants.ArmSubsystem.Claw.kLimitSwitchID);

    elbowFF = new GravityAssistedFeedForward(Constants.ArmSubsystem.Elbow.PIDF.kMINGravityFF,
        Constants.ArmSubsystem.Elbow.PIDF.kGravityFF, Constants.ArmSubsystem.Elbow.kHorizontalAngle);
    
    wristFF =  new GravityAssistedFeedForward(Constants.ArmSubsystem.Wrist.PIDF.kMinGravityFF,
        Constants.ArmSubsystem.Wrist.PIDF.kGravityFF, Constants.ArmSubsystem.Wrist.kHorizontalAngle);

    // trapezoidal profiling for the elbow
    elbowProfile = new TrapezoidProfile(new Constraints(Constants.ArmSubsystem.Elbow.kSlewRate, Constants.ArmSubsystem.Elbow.kAccelerationRate));
    elbowStateSetpoint = new State();
    elbowStateGoal = new State();

    // trapezoidal profiling for the elbow
    wristProfile = new TrapezoidProfile(new Constraints(Constants.ArmSubsystem.Wrist.kSlewRate, Constants.ArmSubsystem.Wrist.kAccelerationRate));
    wristStateSetpoint = new State();
    wristStateGoal = new State();

    // trapezoidal profiling for the elbow
    clawProfile = new TrapezoidProfile(new Constraints(Constants.ArmSubsystem.Claw.kSlewRate, Constants.ArmSubsystem.Claw.kAccelerationRate));
    clawStateSetpoint = new State();
    clawStateGoal = new State();

    // trapezoidal profiling for the pulley
    pulleyProfile = new TrapezoidProfile(new Constraints(Constants.ArmSubsystem.Claw.kSlewRate, Constants.ArmSubsystem.Claw.kAccelerationRate));
    pulleyStateSetpoint = new State();
    pulleyStateGoal = new State();

  }

  public void initializeArm() {

    pulleyMotorTarget = pulleyMotor.getEncoder().getPosition();
    wristMotorTarget = wristMotor.getAbsoluteEncoder().getPosition();
    elbowMotorTarget = elbowMotor.getAbsoluteEncoder().getPosition();
    clawMotorTarget = clawMotor.getEncoder().getPosition();

    elbowStateGoal = new State(elbowMotorTarget, 0);
    elbowStateSetpoint = new State(elbowMotorTarget, 0);

    wristStateGoal = new State(wristMotorTarget, 0);
    wristStateSetpoint = new State(wristMotorTarget, 0);

    clawStateGoal = new State(clawMotorTarget, 0);
    clawStateSetpoint = new State(clawMotorTarget, 0);

    pulleyStateGoal = new State(pulleyMotorTarget, 0);
    pulleyStateSetpoint = new State(pulleyMotorTarget, 0);

    // this is now true as soon as encoders and limiters are initialized
    initialized = true;
  }

  public static double getPulleyHeight() {
    return ARM_HEIGHT;
  }

  public double getElbowMotorPosition() {
    return elbowMotor.getAbsoluteEncoder().getPosition();
  }

  public double getPulleyMotorFF() {
    if (getPulleyHeight() <= Constants.ArmSubsystem.Pulley.kStageTwoPulleyPosition) {
      return Constants.ArmSubsystem.Pulley.PIDF.kStageOneFF;
    } else {
      return Constants.ArmSubsystem.Pulley.PIDF.kStageTwoFF;
    }
  }

  public double getWristMotorPosition() {
    return wristMotor.getAbsoluteEncoder().getPosition();
  }

  public double getPulleyMotorPosition() {
    return pulleyMotor.getEncoder().getPosition();
  }

  public double getAbsoluteWristAngleMax() {
    double elbowDiff = getElbowMotorPosition() - Constants.ArmSubsystem.Elbow.kHorizontalAngle;
    return Constants.ArmSubsystem.Wrist.kMaxLimit + elbowDiff;
  }

  public double getAbsoluteWristAngleMin() {
    double elbowDiff = getElbowMotorPosition() - Constants.ArmSubsystem.Elbow.kHorizontalAngle;
    return Constants.ArmSubsystem.Wrist.kMinLimit + elbowDiff;
  }

  @Override
  public void periodic() {

    // if tuning, do not interfere
    if(Constants.kTuning) {
      return;
    }

    // update static variable accessible to other systems
    ARM_HEIGHT = getPulleyMotorPosition();
    
    // run system only when enabled and initialized in Auto or Teleop
    if (initialized && RobotState.isEnabled() && !RobotState.isTest()) {

      //check if pulley is home
      if (pulleyMotor.get() < 0) {
        if (homeLimitSwitch.get() == Constants.ArmSubsystem.Pulley.kLimitSwitchPressedState) {
          pulleyMotor.set(0);
          pulleyMotor.getEncoder().setPosition(0);
          
          // reset profile at 0
          pulleyStateGoal = new State(Constants.ArmSubsystem.Positions.kHome.pulley, 0);
          pulleyStateSetpoint = new State(0, 0);

        }
      }
  
      // check if claw is home
      if (clawMotor.get() > 0) {
        if (clawLimitSwitch.get() == Constants.ArmSubsystem.Claw.kLimitSwitchPressedState) {
          clawMotor.set(0);
          clawMotor.getEncoder().setPosition(0);

          // reset profile at 0
          clawStateGoal = new State(0,0);
          clawStateSetpoint = new State(0, 0);
        }
      }

      // precalculate safe wrist positions
      double wristSafeTarget = MathUtil.clamp(wristMotorTarget, getAbsoluteWristAngleMin(), getAbsoluteWristAngleMax());

      // XXX: if one or the other of the wrist is not working, then should we use any of it?
      if (wristError == false || elbowError == false) {
        // move the wrist towards its goal
        wristStateGoal = new State(wristSafeTarget, 0); //MUST USE SAFE TARGET
        wristStateSetpoint = wristProfile.calculate(Constants.kRobotLoopTime, wristStateSetpoint, wristStateGoal);
        wristMotor.getClosedLoopController().setReference(wristStateSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, wristFF.calculate(getWristMotorPosition()));

        // move the elbow towards its goal
        elbowStateGoal = new State(elbowMotorTarget, 0);
        elbowStateSetpoint = elbowProfile.calculate(Constants.kRobotLoopTime, elbowStateSetpoint, elbowStateGoal);
        elbowMotor.getClosedLoopController().setReference(elbowStateSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, elbowFF.calculate(getElbowMotorPosition()));
      }

      // move the claw towards its goal
      clawStateGoal = new State(clawMotorTarget, 0);
      clawStateSetpoint = clawProfile.calculate(Constants.kRobotLoopTime, clawStateSetpoint, clawStateGoal);
      clawMotor.getClosedLoopController().setReference(clawStateSetpoint.position, ControlType.kPosition);


      // move the pulley towards its goal
      pulleyStateGoal = new State(pulleyMotorTarget, 0);
      pulleyStateSetpoint = pulleyProfile.calculate(Constants.kRobotLoopTime, pulleyStateSetpoint, pulleyStateGoal);
      pulleyMotor.getClosedLoopController().setReference(pulleyStateSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, getPulleyMotorFF());
      
      
    }
  }
}
