package frc.robot.extensions;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SparkMaxPIDTuner implements Sendable{
    
    private double p, i, d, iZone, gravityFF, directionalFF, setpoint;
    private boolean enabled;


    public SparkMaxPIDTuner(double kp, double ki, double kd, double kIZone) {
        this(kp, ki, kd, kIZone, 0, 0);
    }
    public SparkMaxPIDTuner(double kp, double ki, double kd, double kIZone, double gravityFF, double directionalFF) {
        this.p = kp;
        this.i = ki;
        this.d = kd;
        this.iZone = kIZone;
        this.gravityFF = gravityFF;
        this.directionalFF = directionalFF;
        this.enabled = false;
        this.setpoint = 0;
    }

    public double getP() {
        return this.p;
    }
    public void setP(double kP) {
        this.p = kP;
    }

    public double getI() {
        return this.i;
    }
    public void setI(double kI) {
        this.i = kI;
    }

    public double getD() {
        return this.d;
    }
    public void setD(double kD) {
        this.d = kD;
    }

    public double getIZone() {
        return this.iZone;
    }
    public void setIZone(double kIZone) {
        this.iZone = kIZone;
    }

    public double getGravityFF() {
        return this.gravityFF;
    }
    public void setGravityFF(double kFF) {
        this.gravityFF = kFF;
    }

    public double getDirectionalFF() {
        return this.directionalFF;
    }
    public void setDirectionalFF(double kFF) {
        this.directionalFF = kFF;
    }

    public double getSetpoint() {
        return this.setpoint;
    }
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public String getName() {
        return "PID Tuner";
    }


    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("MotorController");
        builder.setActuator(true);
        builder.addDoubleProperty("P", this::getP, this::setP);
        builder.addDoubleProperty("I", this::getI, this::setI);
        builder.addDoubleProperty("D", this::getD, this::setD);
        builder.addDoubleProperty("SetPoint", this::getSetpoint, this::setSetpoint);
        builder.addDoubleProperty("Directional FF", this::getDirectionalFF, this::setDirectionalFF);
        builder.addDoubleProperty("Gravity FF", this::getGravityFF, this::setGravityFF);
    }
}
