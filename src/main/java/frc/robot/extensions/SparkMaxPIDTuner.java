package frc.robot.extensions;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;

public class SparkMaxPIDTuner {
    private ShuffleboardTab tab;
    private SparkMax motor;
    private ControlType controlType;
    private ClosedLoopConfigAccessor configAccessor;
    private PIDController tuner;
    private GenericEntry ffEntry;
    private GenericEntry referenceEntry;
    private double lastReference;

    public SparkMaxPIDTuner(String name, SparkMax motor, ControlType controlType) {
        this.motor = motor;
        this.controlType = controlType;
        this.lastReference = 0;
        this.configAccessor = motor.configAccessor.closedLoop;
        this.tuner = new PIDController(configAccessor.getP(), configAccessor.getI(), configAccessor.getD());
        this.tab = Shuffleboard.getTab(name + " Tuner");
        this.tab.add(name, tuner).withPosition(0, 0).withSize(2, 2);
        this.ffEntry = this.tab.add(name + " FF", 0).withWidget(BuiltInWidgets.kTextView).withPosition(2, 0).withSize(2,1).getEntry();
        this.referenceEntry = this.tab.add(name + " Reference", this.lastReference).withWidget(BuiltInWidgets.kTextView).withPosition(2, 1).withSize(2,1).getEntry();
        this.tab.add("Apply " + name + " values", new ApplyValues(this)).withPosition(0, 2).withSize(2, 1);
        this.tab.add("Reset " + name + " values", new ResetValues(this)).withPosition(2, 2).withSize(2, 1);

    }

    public void applyTunerValues() {
        SparkMaxConfig newConfig = new SparkMaxConfig();
        newConfig.closedLoop
            .p(tuner.getP())
            .i(tuner.getI())
            .d(tuner.getD())
            .velocityFF(ffEntry.getDouble(0));
        motor.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        this.lastReference = this.referenceEntry.getDouble(0);
        motor.getClosedLoopController().setReference(this.lastReference, controlType);
    }

    public void resetTunerValues() {
        tuner.setP(configAccessor.getP());
        tuner.setI(configAccessor.getI());
        tuner.setD(configAccessor.getD());
        ffEntry.setDouble(configAccessor.getFF());
        referenceEntry.setDouble(this.lastReference);
    }

    public Command getApplyCommand() {
        return new ApplyValues(this);
    }

    public class ApplyValues extends Command {
        SparkMaxPIDTuner tuner;
    
        public ApplyValues(SparkMaxPIDTuner tuner) {
            this.tuner = tuner;
        }
    
        @Override
        public void initialize() {
            tuner.applyTunerValues();
        }
    
        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public class ResetValues extends Command {
        SparkMaxPIDTuner tuner;
    
        public ResetValues(SparkMaxPIDTuner tuner) {
            this.tuner = tuner;
        }
    
        @Override
        public void initialize() {
            tuner.resetTunerValues();
        }
    
        @Override
        public boolean isFinished() {
            return true;
        }
    }

}
