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

public class SparkMaxPIDTunerWithGravityAssist extends SparkMaxPIDTuner {
    private SparkMax motor;
    private ControlType controlType;
    private ClosedLoopConfigAccessor configAccessor;
    private PIDController tuner;    
    private ShuffleboardTab tab;
    private GenericEntry ffEntry;
    private GenericEntry velocityEntry;
    private GenericEntry accelerationEntry;
    private double lastReference;
    private double p0, i0, d0, f0, v0, a0;
    private GenericEntry actualPositionEntry;
    private GenericEntry actualVelocityEntry;
    private Timer updateTimer;

    public static double UPDATE_INTERVAL_SECONDS = 0.5;

    public SparkMaxPIDTunerWithGravityAssist(String name, SparkMax motor, ControlType controlType) {
        super(name,motor,controlType);
        super.setupShuffleboard(name);

        setupShuffleboard(name);
    }

    protected void setupShuffleboard(String name) {
        // setup gravity assist interface in Shuffleboard

    }

    public void updateEncoderValues() {
        super.updateEncoderValues();

    }

    public void applyTunerValues() {
        super.applyTunerValues();
        //TODO: Add gravity assist values
    }

    public void resetTunerValues() {
        super.resetTunerValues();
        //TODO: Add gravity assist values
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
