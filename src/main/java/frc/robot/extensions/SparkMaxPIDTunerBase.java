package frc.robot.extensions;

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
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Tuning.ApplyTunerValues;
import frc.robot.commands.Tuning.ResetTunerValues;
import frc.robot.commands.Tuning.StartTunerMotor;
import frc.robot.commands.Tuning.StopTunerMotor;

public class SparkMaxPIDTunerBase {
    protected SparkMax motor;
    protected ControlType controlType;
    protected ClosedLoopConfigAccessor configAccessor;
    protected PIDController tuner;    
    protected ShuffleboardTab tab;
    private double p0, i0, d0;
    protected Timer updateTimer;

    public static double UPDATE_INTERVAL_SECONDS = 0.5;

    public SparkMaxPIDTunerBase(String name, SparkMax motor) {
        
        this.motor = motor;
        this.configAccessor = motor.configAccessor.closedLoop;
        this.controlType = ControlType.kPosition;
        this.p0 = configAccessor.getP();
        this.i0 = configAccessor.getI();
        this.d0 = configAccessor.getD();

        this.tuner = new PIDController(this.p0, this.i0, this.d0);

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

    }

    public void updateEncoderValues() {
        
    }

    public void applyTunerValues() {
        SparkMaxConfig newConfig = new SparkMaxConfig();
        newConfig.closedLoop
            .p(tuner.getP())
            .i(tuner.getI())
            .d(tuner.getD());

        motor.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        StringBuilder sb = new StringBuilder();
        sb.append("P: " + configAccessor.getP() + " - ");
        sb.append("I: " + configAccessor.getI() + " - ");
        sb.append("D: " + configAccessor.getD() + " - ");
        System.out.println(sb.toString());

    }

    public void resetTunerValues() {
        tuner.setP(this.p0);
        tuner.setI(this.i0);
        tuner.setD(this.d0);
        tuner.setSetpoint(0);
    }

    public void startMotor() {

    }

    public void stopMotor() {
        motor.stopMotor();
    }

}
