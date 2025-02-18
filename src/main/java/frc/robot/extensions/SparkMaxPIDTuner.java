package frc.robot.extensions;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import java.util.Map;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Tuning.ApplyTunerValues;
import frc.robot.commands.Tuning.ResetTunerValues;
import frc.robot.commands.Tuning.StopTunerMotor;

public class SparkMaxPIDTuner {
    private SparkMax motor;
    private ControlType controlType;
    private ClosedLoopConfigAccessor configAccessor;
    private PIDController tuner;    
    private ShuffleboardTab tab;
    private ShuffleboardLayout tuningLayout;
    private ShuffleboardLayout commandLayout;
    private GenericEntry ffEntry;
    private GenericEntry velocityEntry;
    private GenericEntry accelerationEntry;
    private double lastReference;
    private double p0, i0, d0, f0, v0, a0;

    public SparkMaxPIDTuner(String name, SparkMax motor, ControlType controlType) {
        
        this.motor = motor;
        this.controlType = controlType;
        this.configAccessor = motor.configAccessor.closedLoop;
        this.p0 = configAccessor.getP();
        this.i0 = configAccessor.getI();
        this.d0 = configAccessor.getD();
        this.f0 = configAccessor.getFF();

        this.tuner = new PIDController(this.p0, this.i0, this.d0);

        if(controlType == ControlType.kPosition || controlType == ControlType.kMAXMotionPositionControl) {
            this.lastReference = this.motor.getEncoder().getPosition();
        } else {
            this.lastReference = 0;         
        }
        this.tuner.setSetpoint(this.lastReference);
        
        if(controlType == ControlType.kMAXMotionPositionControl) {
            this.v0 = configAccessor.maxMotion.getMaxVelocity();
            this.a0 = configAccessor.maxMotion.getMaxAcceleration();
        }

        setupShuffleboard(name);
    }

    private void setupShuffleboard(String name) {
        // setup interface in Shuffleboard
        this.tab = Shuffleboard.getTab(name + "PID");
        this.tuningLayout = this.tab.getLayout("Value Tuning", BuiltInLayouts.kList);
        this.tuningLayout
            .withPosition(0,0)
            .withSize(2, 4)
            .withProperties(Map.of("Label position", "TOP")); 
        this.tuningLayout.add("PID", tuner).withWidget(BuiltInWidgets.kPIDController);        
        this.ffEntry = this.tuningLayout.add("FF", this.f0).withWidget(BuiltInWidgets.kTextView).getEntry();            
        if(controlType == ControlType.kMAXMotionPositionControl) {
            this.accelerationEntry = this.tab.add("MAX Acceleration", this.a0).withWidget(BuiltInWidgets.kTextView).getEntry();
            this.velocityEntry = this.tab.add("MAX Velocity", this.v0).withWidget(BuiltInWidgets.kTextView).getEntry();
        } 

        this.commandLayout = this.tab.getLayout("Commands", BuiltInLayouts.kList);
        this.commandLayout
            .withPosition(2,0)
            .withSize(2, 4)
            .withProperties(Map.of("Label position", "HIDDEN"));

        //this.commandLayout = this.tab.getLayout("Value Tuning", BuiltInLayouts.kList);
        this.commandLayout.add("Apply", new ApplyTunerValues(this));
        this.commandLayout.add("Reset", new ResetTunerValues(this));
        this.commandLayout.add("STOP!", new StopTunerMotor(this));
    }

    public void applyTunerValues() {
        SparkMaxConfig newConfig = new SparkMaxConfig();
        newConfig.closedLoop
            .p(tuner.getP())
            .i(tuner.getI())
            .d(tuner.getD())
            .velocityFF(ffEntry.getDouble(0));

        if(controlType == ControlType.kMAXMotionPositionControl) {
            newConfig.closedLoop.maxMotion
                .maxVelocity(velocityEntry.getDouble(0.1))
                .maxAcceleration(accelerationEntry.getDouble(0.1));
        }
        motor.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        this.lastReference = tuner.getSetpoint();
        motor.getClosedLoopController().setReference(this.lastReference, controlType);
    }

    public void resetTunerValues() {
        tuner.setP(this.p0);
        tuner.setI(this.i0);
        tuner.setD(this.d0);
        ffEntry.setDouble(this.f0);
        tuner.setSetpoint(0);
        if(controlType == ControlType.kMAXMotionPositionControl) {
            velocityEntry.setDouble(this.v0);
            accelerationEntry.setDouble(this.a0);            
        }
    }

    public void stopMotor() {
        motor.stopMotor();
    }

    public Command getApplyCommand() {
        return new ApplyTunerValues(this);
    }

}
