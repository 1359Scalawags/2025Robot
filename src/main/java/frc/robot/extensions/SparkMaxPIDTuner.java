package frc.robot.extensions;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;

public class SparkMaxPIDTuner extends PIDController {
    
    private double gravityFF, staticFF;
    public SparkMaxPIDTuner(double kp, double ki, double kd, double gravityFF, double staticFF) {
        super(kp, ki, kd);
        this.gravityFF = gravityFF;
        this.staticFF = staticFF;
    }

    public double setGravityFF(double kFF) {
        this.gravityFF = kFF;
    }

    public double setStaticFF(double kFF) {
        this.staticFF = kFF;
    }
}
