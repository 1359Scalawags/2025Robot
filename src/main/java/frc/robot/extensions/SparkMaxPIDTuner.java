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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Tuning.ApplyTunerValues;
import frc.robot.commands.Tuning.ResetTunerValues;
import frc.robot.commands.Tuning.StartTunerMotor;
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
    private GenericEntry actualPositionEntry;
    private GenericEntry actualVelocityEntry;
    private Timer updateTimer;

    public static double UPDATE_INTERVAL_SECONDS = 1.0;

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
        updateTimer = new Timer();
        updateTimer.reset();
        updateTimer.start();
    }

    private void setupShuffleboard(String name) {
        // setup interface in Shuffleboard
        this.tab = Shuffleboard.getTab(name + " PID");

        this.tab.add("PID", tuner).withWidget(BuiltInWidgets.kPIDController)
            .withPosition(0, 0)
            .withSize(2,2);        
        this.ffEntry = this.tab.add("FF", this.f0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0,2)
            .withSize(2,1)
            .getEntry();  

        if(controlType == ControlType.kMAXMotionPositionControl) {
            this.accelerationEntry = this.tab.add("MAX Acceleration", this.a0).withWidget(BuiltInWidgets.kTextView).getEntry();
            this.velocityEntry = this.tab.add("MAX Velocity", this.v0).withWidget(BuiltInWidgets.kTextView).getEntry();
        } 

        this.tab.add("Apply", new ApplyTunerValues(this))
            .withPosition(2, 0)
            .withSize(2, 1);
        this.tab.add("Reset", new ResetTunerValues(this))
            .withPosition(2, 1)
            .withSize(2, 1);
        this.tab.add("Start", new StartTunerMotor(this))
            .withPosition(2, 2)
            .withSize(2, 1);
        this.tab.add("STOP!", new StopTunerMotor(this))
            .withPosition(2, 3)
            .withSize(2, 1);

        this.actualPositionEntry = this.tab.add("Encoder Position", this.motor.getEncoder().getPosition())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(5,0)
            .withSize(2,1)
            .getEntry();  

        this.actualVelocityEntry = this.tab.add("Encoder Velocity", this.motor.getEncoder().getVelocity())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(5,1)
            .withSize(2,1)
            .getEntry();  
    }

    public void updateEncoderValues() {
        if(updateTimer.get() > UPDATE_INTERVAL_SECONDS) {
            this.actualPositionEntry.setDouble(motor.getEncoder().getPosition());
            this.actualVelocityEntry.setDouble(motor.getEncoder().getVelocity());      
            updateTimer.reset();      
        }
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

    public void startMotor() {
        motor.getClosedLoopController().setReference(this.lastReference, controlType);
    }

    public void stopMotor() {
        motor.stopMotor();
    }

    public Command getApplyCommand() {
        return new ApplyTunerValues(this);
    }

}
