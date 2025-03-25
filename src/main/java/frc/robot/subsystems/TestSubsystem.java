package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TestSubsystem extends SubsystemBase {
    private SparkMax testSparkMax;
    private SparkSim testSparkMaxSim;
    private DCMotor testDCMotorNeo;
    private ElevatorSim testElevator;

    public TestSubsystem() {
        testSparkMax = new SparkMax(Constants.TestSystem.kTestMotor, MotorType.kBrushless);
        testDCMotorNeo = DCMotor.getNEO(1);
        testSparkMaxSim = new SparkSim(testSparkMax, testDCMotorNeo);
        testElevator = new ElevatorSim(testDCMotorNeo, 64, 1, 0.01, 0.1, 2.0, true, .15, 0.0001,0.0);
        configureTestSparkMax();
        testSparkMax.getEncoder().setPosition(testElevator.getPositionMeters());
    }

    public void configureTestSparkMax() {
        SparkMaxConfig config = new SparkMaxConfig();

        config
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .openLoopRampRate(0.5)
            .closedLoopRampRate(0.5)
            .smartCurrentLimit(70, 40, 0);

        config.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);

        config.closedLoop
            .pid(.1,0,0);
            
        config.closedLoop.maxMotion
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
            .maxAcceleration(20)
            .maxVelocity(1);
        
        testSparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // public void setMotorVolts(double voltage) {
    //     double volts = MathUtil.clamp(voltage, -12, 12);
    //     testSparkMax.setVoltage(volts);            
    // }

    public void move(double value) {
        testSparkMax.getClosedLoopController().setReference(value, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, 0.15);
    }

    // public double getMotorVolts() {
    //     return testSparkMax.getAppliedOutput() * 12;
    // }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("TestElevator/Height", testElevator.getPositionMeters());
    }

    @Override
    public void simulationPeriodic() {
        //update the WPILIB elevator simulator
        testElevator.setInput(testSparkMaxSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        testElevator.update(Constants.kSimulationLoopTime);

        //update the SparkMax simulator
        testSparkMaxSim.iterate(testElevator.getVelocityMetersPerSecond(), RoboRioSim.getVInVoltage(), Constants.kSimulationLoopTime);

        //update battery voltage
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(testSparkMaxSim.getMotorCurrent()));
    }
}
