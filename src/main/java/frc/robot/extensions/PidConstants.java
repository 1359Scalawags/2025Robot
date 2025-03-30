package frc.robot.extensions;

import frc.robot.Constants;
import frc.robot.Robot;

public class PidConstants {
    private final double p, i, d;
    private final double arbff, gravff;

    public PidConstants(double p, double i, double d, double arbitraryff, double gravityff) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.arbff = arbitraryff;
        this.gravff = gravityff;
    }

    public double p() {
        return Robot.isSimulation() ?  p * (Constants.kSimulationPidScalar * Constants.kRobotLoopTime / 0.001) : p;
    }

    public double i() {
        return Robot.isSimulation() ? i * (Constants.kSimulationPidScalar * Constants.kRobotLoopTime / 0.001) : i;
    }

    public double d() {
        return Robot.isSimulation() ? d * (Constants.kSimulationPidScalar * Constants.kRobotLoopTime / 0.001) : d;
    }

    public double arbFF() {
        return arbff;
    }

    public double gravff() {
        return gravff;
    }


}
