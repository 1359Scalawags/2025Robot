package frc.robot.extensions;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Tuning.ApplyTunerValues;
import frc.robot.commands.Tuning.ResetTunerValues;
import frc.robot.commands.Tuning.StartTunerMotor;
import frc.robot.commands.Tuning.StopTunerMotor;

public class SparkMaxPIDTuner {
    protected SparkMax motor;
    protected ControlType controlType;
    private ClosedLoopConfigAccessor configAccessor;
    protected PIDController tuner;    
    protected ShuffleboardTab tab;
    private GenericEntry ffEntry;
    private GenericEntry velocityEntry;
    private GenericEntry accelerationEntry;
    protected double lastReference;
    private double p0, i0, d0, f0, vel0, acc0;
    private GenericEntry actualPositionEntry;
    private GenericEntry actualVelocityEntry;
    protected Timer updateTimer;

    public static double UPDATE_INTERVAL_SECONDS = 0.5;

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
            this.lastReference = this.motor.getAbsoluteEncoder().getPosition();
        } else {
            this.lastReference = 0;         
        }
        this.tuner.setSetpoint(this.lastReference);
        
        if(controlType == ControlType.kMAXMotionPositionControl) {
            this.vel0 = configAccessor.maxMotion.getMaxVelocity();
            this.acc0 = configAccessor.maxMotion.getMaxAcceleration();
        }

        setupShuffleboard(name);
        updateTimer = new Timer();
        updateTimer.reset();
        updateTimer.start();
    }

    protected void setupShuffleboard(String name) {
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
            this.accelerationEntry = this.tab.add("MAX Acceleration", this.acc0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(4, 0)
            .withSize(1, 1)
            .getEntry();
            this.velocityEntry = this.tab.add("MAX Velocity", this.vel0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(4, 1)
            .withSize(1, 1)
            .getEntry();
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
            .withPosition(4, 2)
            .withSize(2, 1);

        this.actualPositionEntry = this.tab.add("Encoder Position", this.motor.getAbsoluteEncoder().getPosition())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(5,0)
            .withSize(2,1)
            .getEntry();  

        this.actualVelocityEntry = this.tab.add("Encoder Velocity", this.motor.getAbsoluteEncoder().getVelocity())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(5,1)
            .withSize(2,1)
            .getEntry();  
    }

    public void updateEncoderValues() {
        if(updateTimer.get() > UPDATE_INTERVAL_SECONDS) {
            this.actualPositionEntry.setDouble(motor.getAbsoluteEncoder().getPosition());
            this.actualVelocityEntry.setDouble(motor.getAbsoluteEncoder().getVelocity());      
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
        StringBuilder sb = new StringBuilder();
        sb.append("P: " + configAccessor.getP() + "  ");
        sb.append("I: " + configAccessor.getI() + "  ");
        sb.append("D: " + configAccessor.getD() + "  ");
        System.out.println(sb.toString());

    }

    public void resetTunerValues() {
        tuner.setP(this.p0);
        tuner.setI(this.i0);
        tuner.setD(this.d0);
        ffEntry.setDouble(this.f0);
        tuner.setSetpoint(0);
        if(controlType == ControlType.kMAXMotionPositionControl) {
            velocityEntry.setDouble(this.vel0);
            accelerationEntry.setDouble(this.acc0);            
        }
    }

    public void startMotor() {
        this.lastReference = this.tuner.getSetpoint();
        motor.getClosedLoopController().setReference(this.lastReference, controlType);
        System.out.println("Setpoint: " + this.lastReference);
    }

    public void stopMotor() {
        motor.stopMotor();
    }

}
