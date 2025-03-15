package frc.robot.extensions;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import java.security.InvalidParameterException;
import java.util.function.DoubleSupplier;

import javax.naming.OperationNotSupportedException;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Constants;

public class SparkMaxPIDTunerPosition extends SparkMaxPIDTunerBase implements ISparkMaxTuner {
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
        return arbitraryFFEntry.getDouble(0);
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
    public void updateEncoderValues() {
        if(this.isInitialized()) {
            super.updateEncoderValues(); // also update encoder values in base class
            if(updateTimer.get() > UPDATE_INTERVAL_SECONDS) {
                this.actualPositionEntry.setDouble(positionEncoderSupplier.getAsDouble());
                updateTimer.reset();      
            }            
        }

    }

    @Override
    public void applyTunerValues() {
        super.applyTunerValues(); // also apply values in base class
        SparkMaxConfig newConfig = new SparkMaxConfig();

        if(this.getControlType() == ControlType.kMAXMotionPositionControl) {
            newConfig.closedLoop.maxMotion
                .maxVelocity(velocityEntry.getDouble(0.1))
                .maxAcceleration(accelerationEntry.getDouble(0.1));
        }
        motor.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        StringBuilder sb = new StringBuilder();
        if(this.getControlType() == ControlType.kMAXMotionPositionControl) {
            sb.append("MAX Velocity: " + velocityEntry.getDouble(0.1) + " - ");
            sb.append("MAX Acceleration: " + accelerationEntry.getDouble(0.1) + " - ");
        }
        sb.append("Arbitrary FF: " + arbitraryFFEntry.getDouble(0));
        System.out.println(sb.toString());
    }

    @Override
    public void resetTunerValues() {
        super.resetTunerValues(); // also reset values in base class
        this.arbitraryFFEntry.setDouble(this.arbitraryFF0);
        if(this.getControlType() == ControlType.kMAXMotionPositionControl) {
            velocityEntry.setDouble(this.velocity0);
            accelerationEntry.setDouble(this.acceleration0);            
        }
    }

    public void periodic() {
        super.periodic(0, arbitraryFFEntry.getDouble(0));
    }

}