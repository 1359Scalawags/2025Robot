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
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class TestSubsystem extends SubsystemBase {
    private SparkMax elevatorSparkMax;
    private SparkSim elevatorSparkMaxSim;
    private DCMotor elevatorDCMotorNeo;
    private ElevatorSim elevatorSim;
    private Mechanism2d mechanism;
    private MechanismRoot2d mechanismRoot;
    private MechanismLigament2d elevatorMechanism; 
    private double elevatorTargetHeight;


    public TestSubsystem() {
        elevatorSparkMax = new SparkMax(Constants.TestSubsystem.kTestMotor, MotorType.kBrushless);
        elevatorDCMotorNeo = DCMotor.getNEO(1);
        elevatorSparkMaxSim = new SparkSim(elevatorSparkMax, elevatorDCMotorNeo);
        elevatorSim = new ElevatorSim(elevatorDCMotorNeo, 
                                Constants.TestSubsystem.Elevator.kGearRatio, 
                                Constants.TestSubsystem.Elevator.kCarriageMassKg, 
                                Constants.TestSubsystem.Elevator.kSpindleRadiusMeters, 
                                Constants.TestSubsystem.Elevator.kMinHeightMeters, 
                                Constants.TestSubsystem.Elevator.kMaxHeightMeters, 
                                true, 
                                Constants.TestSubsystem.Elevator.kMinHeightMeters, 
                                0.001,0.0);
        configureTestSparkMax();
        

        mechanism = new Mechanism2d(2, 3, new Color8Bit(Color.kBlack));
        mechanismRoot = mechanism.getRoot("Sim Elevator Root", 1, 0);
        elevatorMechanism = mechanismRoot.append(new MechanismLigament2d("Sim Elevator", 1.5, 90));

        elevatorTargetHeight = Constants.TestSubsystem.Elevator.kMinHeightMeters;
        elevatorSim.setState(Constants.TestSubsystem.Elevator.kMinHeightMeters, 0);
        elevatorSparkMax.getClosedLoopController().setReference(elevatorTargetHeight, ControlType.kPosition, ClosedLoopSlot.kSlot0, Constants.TestSubsystem.Elevator.PIDF.kGravityFF);


    }

    public void initializeEncoders() {
        elevatorSparkMaxSim.setPosition(elevatorSim.getPositionMeters());
        System.out.println("Encoder set to: " + elevatorSim.getPositionMeters());
    }

    public void configureTestSparkMax() {
        SparkMaxConfig config = new SparkMaxConfig();

        config
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .openLoopRampRate(0.1)
            .closedLoopRampRate(0.1)
            .smartCurrentLimit(70, 40, 0);

        config.encoder
            .positionConversionFactor(1.0 / (Constants.TestSubsystem.Elevator.kMotorRotationsPerMeter))
            .quadratureMeasurementPeriod(2)
            .quadratureAverageDepth(16);
            //.velocityConversionFactor(1.0 / (Constants.TestSubsystem.Elevator.kMotorRotationsPerMeter * 60.0));

        config.closedLoop
            .p(Robot.isSimulation() ? Constants.TestSubsystem.Elevator.PIDF.kP * 20 : Constants.TestSubsystem.Elevator.PIDF.kP)
            .i(Robot.isSimulation() ? Constants.TestSubsystem.Elevator.PIDF.kI * 20 : Constants.TestSubsystem.Elevator.PIDF.kI)
            .d(Robot.isSimulation() ? Constants.TestSubsystem.Elevator.PIDF.kD * 20 : Constants.TestSubsystem.Elevator.PIDF.kD);
            
        // config.closedLoop.maxMotion
        //     .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
        //     .maxAcceleration(50 * Constants.TestSubsystem.Elevator.kMotorRotationsPerMeter * 60)
        //     .maxVelocity(20 * Constants.TestSubsystem.Elevator.kMotorRotationsPerMeter * 60);
        config.closedLoop.maxMotion
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
            .maxAcceleration(50000)
            .maxVelocity(50000);
        
        elevatorSparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // public void setMotorVolts(double voltage) {
    //     double volts = MathUtil.clamp(voltage, -12, 12);
    //     testSparkMax.setVoltage(volts);            
    // }

    public void move(double value) {
        elevatorTargetHeight = MathUtil.clamp(elevatorTargetHeight + value, Constants.TestSubsystem.Elevator.kMinHeightMeters, Constants.TestSubsystem.Elevator.kMaxHeightMeters);
        elevatorSparkMax.getClosedLoopController().setReference(elevatorTargetHeight, ControlType.kPosition, ClosedLoopSlot.kSlot0, Constants.TestSubsystem.Elevator.PIDF.kGravityFF);
        elevatorSparkMaxSim.getRelativeEncoderSim().setPosition(value);
    }

    // public double getMotorVolts() {
    //     return testSparkMax.getAppliedOutput() * 12;
    // }

    public void updateTelemetry() {
        elevatorMechanism.setLength(elevatorSim.getPositionMeters());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("TestElevator/Height", elevatorSim.getPositionMeters());
        this.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        //update the WPILIB elevator simulator
        elevatorSim.setInput(elevatorSparkMaxSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        elevatorSim.update(Constants.kSimulationLoopTime);

        //update the SparkMax simulator
        double motorRPM = elevatorSim.getVelocityMetersPerSecond() * Constants.TestSubsystem.Elevator.kMotorRotationsPerMeter * 60;
        elevatorSparkMaxSim.iterate(motorRPM, RoboRioSim.getVInVoltage(), Constants.kSimulationLoopTime);

        //update battery voltage
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSparkMaxSim.getMotorCurrent()));
    }
}
