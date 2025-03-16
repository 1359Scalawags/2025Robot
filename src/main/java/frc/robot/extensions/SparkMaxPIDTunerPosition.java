package frc.robot.extensions;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import java.security.InvalidParameterException;
import java.util.function.DoubleSupplier;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
public class SparkMaxPIDTunerPosition extends SparkMaxPIDTunerBase {
    private GenericEntry arbitraryFFEntry;
    private GenericEntry velocityEntry;
    private GenericEntry accelerationEntry;
    private double arbitraryFF0, velocity0, acceleration0;
    private GenericEntry actualPositionEntry;
    private DoubleSupplier positionEncoderSupplier;

    public static double UPDATE_INTERVAL_SECONDS = 0.5;

    public SparkMaxPIDTunerPosition(String name, SparkMax motor, ControlType controlType) {
        super(name, motor);
        this.shuffleboardSetupRoutines.add(this::setupShuffleboard);

        if(controlType != ControlType.kPosition && controlType != ControlType.kMAXMotionPositionControl) {
            throw new InvalidParameterException("Must be a position control type.");
        }
        
        this.motor = motor;
        this.arbitraryFF0 = 0;

        if(this.configAccessor.getFeedbackSensor() == FeedbackSensor.kAbsoluteEncoder) {
            this.positionEncoderSupplier = motor.getAbsoluteEncoder()::getPosition;
        } else {
            this.positionEncoderSupplier = motor.getEncoder()::getPosition;
        }
        if(this.positionEncoderSupplier == null) {
            throw new NullPointerException("Position encoder supplier is not instantiated.");
        }

        this.setControlType(controlType);
        if(this.getControlType() == ControlType.kMAXMotionPositionControl) {
            controlType = ControlType.kMAXMotionPositionControl;
            this.velocity0 = configAccessor.maxMotion.getMaxVelocity();
            this.acceleration0 = configAccessor.maxMotion.getMaxAcceleration();            
        } else {
            controlType = ControlType.kPosition;
        }

    }

    protected double getArbitraryFF() {
        double arb = 0;
        if(arbitraryFFEntry != null) 
             arb = arbitraryFFEntry.getDouble(0);
        return arb;
    }

    private boolean setupShuffleboard() {
        // NOTE: base shuffleboard interface already configured in constructor
        // setup interface in Shuffleboard
        if(this.getControlType() == ControlType.kMAXMotionPositionControl) {
            this.accelerationEntry = super.getValueTuningLayout().add("MAX Acceleration", this.acceleration0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(2, 0)
            .withSize(1,1)
            .getEntry();
            this.velocityEntry = super.getValueTuningLayout().add("MAX Velocity", this.velocity0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(2, 1)
            .withSize(1,1)
            .getEntry();
        } 

        this.arbitraryFFEntry = super.getValueTuningLayout().add("Arbitrary FF", this.arbitraryFF0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(3,0)
            .withSize(1,1)
            .getEntry();  
        this.actualPositionEntry = super.getEncoderFeedbackLayout().add("Encoder Position", this.positionEncoderSupplier.getAsDouble())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0,0)
            .withSize(1,1)
            .getEntry();  

        return true;
    }

    @Override
    protected void updateEncoderValues() {
        super.updateEncoderValues(); // also update encoder values in base class
        if(updateTimer.get() > UPDATE_INTERVAL_SECONDS) {
            if(this.actualPositionEntry != null)
                this.actualPositionEntry.setDouble(positionEncoderSupplier.getAsDouble());
            updateTimer.reset();      
        }            
    }

    @Override
    public void applyTunerValues() {
        super.applyTunerValues(); // also apply values in base class
        SparkMaxConfig newConfig = new SparkMaxConfig();

        if(this.getControlType() == ControlType.kMAXMotionPositionControl && this.velocityEntry != null && this.accelerationEntry != null) {
            newConfig.closedLoop.maxMotion
                .maxVelocity(velocityEntry.getDouble(0.1))
                .maxAcceleration(accelerationEntry.getDouble(0.1));
        }
        motor.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        if(this.getVerbosity() == Verbosity.commands || this.getVerbosity() == Verbosity.all) {
            StringBuilder sb = new StringBuilder();
            if(this.getControlType() == ControlType.kMAXMotionPositionControl) {
                if(this.velocityEntry != null && this.accelerationEntry != null) {
                    sb.append("|  MAX Velocity: " + df10.format(velocityEntry.getDouble(0.1)) + " - ");
                    sb.append(" MAX Acceleration: " + df10.format(accelerationEntry.getDouble(0.1)) + "\n");
                }
            }
            if(this.arbitraryFFEntry != null) {
                sb.append("|  Arbitrary FF: " + arbitraryFFEntry.getDouble(0));
            }
            System.out.println(sb.toString());            
        }

    }

    @Override
    public void resetTunerValues() {
        super.resetTunerValues(); // also reset values in base class
        if(this.arbitraryFFEntry != null) {
            this.arbitraryFFEntry.setDouble(this.arbitraryFF0);            
        }
        if(this.getControlType() == ControlType.kMAXMotionPositionControl && this.velocityEntry != null && this.accelerationEntry != null) {
            velocityEntry.setDouble(this.velocity0);
            accelerationEntry.setDouble(this.acceleration0);            
        }
    }

    public void periodic() {
        double arb = 0;
        if(this.arbitraryFFEntry != null)
            arb = this.arbitraryFFEntry.getDouble(0);
        super.periodic(0, arb);
        this.updateEncoderValues();
    }

}