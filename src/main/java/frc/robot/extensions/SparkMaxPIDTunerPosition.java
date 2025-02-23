package frc.robot.extensions;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Tuning.ApplyTunerValues;
import frc.robot.commands.Tuning.ResetTunerValues;
import frc.robot.commands.Tuning.StartTunerMotor;
import frc.robot.commands.Tuning.StopTunerMotor;

public class SparkMaxPIDTunerPosition extends SparkMaxPIDTunerBase {
    private ShuffleboardLayout maxMotionLayout;
    protected GenericEntry ffEntry;
    private GenericEntry velocityEntry;
    private GenericEntry accelerationEntry;
    protected double lastPositionReference;
    private double f0, vel0, acc0;
    private GenericEntry actualPositionEntry;
    DoubleSupplier positionEncoderSupplier;
    private boolean useMaxMotion;
    private boolean useAbsoluteEncoder;

    public static double UPDATE_INTERVAL_SECONDS = 0.5;

    public SparkMaxPIDTunerPosition(String name, SparkMax motor, boolean useAbsoluteEncoder, boolean useMaxMotion) {
        super(name, motor);
        this.motor = motor;
        this.f0 = 0;

        this.useAbsoluteEncoder = useAbsoluteEncoder;   
        if(useAbsoluteEncoder) {
            positionEncoderSupplier = motor.getAbsoluteEncoder()::getPosition;
        } else {
            positionEncoderSupplier = motor.getEncoder()::getPosition;
        }

        this.useMaxMotion = useMaxMotion;
        if(this.useMaxMotion) {
            controlType = ControlType.kMAXMotionPositionControl;
            this.vel0 = configAccessor.maxMotion.getMaxVelocity();
            this.acc0 = configAccessor.maxMotion.getMaxAcceleration();            
        } else {
            controlType = ControlType.kPosition;
        }

        this.lastPositionReference = positionEncoderSupplier.getAsDouble();
        this.tuner.setSetpoint(this.lastPositionReference);
        
        setupShuffleboard(name);
    }

    protected void setupShuffleboard(String name) {
        // setup interface in Shuffleboard
        if(this.useMaxMotion) {
            this.maxMotionLayout = this.tab.getLayout("MAX Motion");
            this.accelerationEntry = this.maxMotionLayout.add("MAX Acceleration", this.acc0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
            this.velocityEntry = this.maxMotionLayout.add("MAX Velocity", this.vel0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
        } 

        this.ffEntry = this.tab.add("Arbitrary FF", this.f0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0,2)
            .withSize(2,1)
            .getEntry();  

        this.actualPositionEntry = this.tab.add("Encoder Position", this.positionEncoderSupplier.getAsDouble())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(5,0)
            .withSize(2,1)
            .getEntry();  

    }

    @Override
    public void updateEncoderValues() {
        super.updateEncoderValues();
        if(updateTimer.get() > UPDATE_INTERVAL_SECONDS) {
            this.actualPositionEntry.setDouble(positionEncoderSupplier.getAsDouble());
            updateTimer.reset();      
        }
    }

    @Override
    public void applyTunerValues() {
        super.applyTunerValues();
        SparkMaxConfig newConfig = new SparkMaxConfig();

        if(this.useMaxMotion) {
            newConfig.closedLoop.maxMotion
                .maxVelocity(velocityEntry.getDouble(0.1))
                .maxAcceleration(accelerationEntry.getDouble(0.1));
        }
        motor.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        StringBuilder sb = new StringBuilder();
        sb.append("MAX Velocity: " + velocityEntry.getDouble(0.1) + " - ");
        sb.append("MAX Acceleration: " + accelerationEntry.getDouble(0.1) + " - ");
        sb.append("F: " + ffEntry.getDouble(0) + " - ");

    }

    @Override
    public void resetTunerValues() {
        super.resetTunerValues();
        tuner.setSetpoint(0);
        this.ffEntry.setDouble(0);
        if(this.useMaxMotion) {
            velocityEntry.setDouble(this.vel0);
            accelerationEntry.setDouble(this.acc0);            
        }
    }

    @Override
    public void startMotor() {
        this.lastPositionReference = this.tuner.getSetpoint();
        motor.getClosedLoopController().setReference(this.lastPositionReference, controlType, ClosedLoopSlot.kSlot0, ffEntry.getDouble(0));
        System.out.println("Setpoint: " + this.lastPositionReference);
    }

}
