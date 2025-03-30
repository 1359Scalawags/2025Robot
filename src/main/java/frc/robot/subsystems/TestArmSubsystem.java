package frc.robot.subsystems;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TestArmSubsystem extends SubsystemBase {

    private SparkMax armSparkMax;
    private SparkSim armSparkMaxSim;
    private SparkAbsoluteEncoderSim armAbsoluteEncoderSim;
    private DCMotor armDCMotorNeo;
    private SingleJointedArmSim armSim;


    private Mechanism2d mechanism;
    private MechanismRoot2d mechanismRoot;
    private MechanismLigament2d armMechanism;

    private double armTargetAngleDegrees;


    public TestArmSubsystem() {

        // create and configure simulation-ready elevator SparkMax motor
        armSparkMax = new SparkMax(Constants.TestArm.kMotorID, MotorType.kBrushless);
        armDCMotorNeo = DCMotor.getNEO(1);
        armSparkMaxSim = new SparkSim(armSparkMax, armDCMotorNeo);
        armAbsoluteEncoderSim = new SparkAbsoluteEncoderSim(armSparkMax);
        configureArmSparkMax();

        // create simulated arm
        armSim = new SingleJointedArmSim(armDCMotorNeo, 
                                Constants.TestArm.kGearRatio, 
                                Constants.TestArm.kMomentInertiaKgMM, 
                                Constants.TestArm.kArmLengthMeters, 
                                Math.toRadians(Constants.TestArm.kMinAngleDegrees), 
                                Math.toRadians(Constants.TestArm.kMaxAngleDegrees), 
                                true, 
                                Math.toRadians(Constants.TestArm.kMinAngleDegrees), 
                                0.001,0.005);  

        armSim.setState(Math.toRadians(Constants.TestArm.kMinAngleDegrees), 0);   
        armTargetAngleDegrees = Constants.TestArm.kMinAngleDegrees;
        //armAbsoluteEncoderSim.setPosition(Constants.TestArm.kMinAngleDegrees);        
        //armSparkMax.getEncoder().setPosition(armAbsoluteEncoderSim.getPosition());
        armSparkMax.getClosedLoopController().setReference(armTargetAngleDegrees, ControlType.kPosition, ClosedLoopSlot.kSlot0, Constants.TestArm.pid.gravff());

        mechanism = new Mechanism2d(2, 2, new Color8Bit(Color.kBlack));
        mechanismRoot = mechanism.getRoot("Sim Arm Root", 0.5, 1);
        armMechanism = mechanismRoot.append(new MechanismLigament2d("Sim Arm", Constants.TestArm.kArmLengthMeters, Math.toRadians(Constants.TestArm.kMinAngleDegrees)));  
    }

    public MechanismLigament2d getMechanismLigament() {
        return armMechanism;
    }

    public void initializeEncoders() {
        armAbsoluteEncoderSim.setPosition(Math.toDegrees(armSim.getAngleRads()));
        armSparkMaxSim.setPosition(Math.toDegrees(armSim.getAngleRads()));
        System.out.println("Arm: " + Math.toDegrees(armSim.getAngleRads()));
    }

    public void configureArmSparkMax() {
        SparkMaxConfig config = new SparkMaxConfig();

        config
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .openLoopRampRate(0.1)
            .closedLoopRampRate(0.1)
            .smartCurrentLimit(70, 40, 0);

        config.encoder
            .positionConversionFactor(360);

        config.absoluteEncoder
            .positionConversionFactor(360)
            .zeroOffset(0)
            .zeroCentered(true);

        config.closedLoop
            .p(Constants.TestArm.pid.p())
            .i(Constants.TestArm.pid.i())
            .d(Constants.TestArm.pid.d())
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        
        armSparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void move(double armDelta) {
        armTargetAngleDegrees = MathUtil.clamp(armTargetAngleDegrees + armDelta, Constants.TestArm.kMinAngleDegrees, Constants.TestArm.kMaxAngleDegrees);
        armSparkMax.getClosedLoopController().setReference(armTargetAngleDegrees, ControlType.kPosition, ClosedLoopSlot.kSlot0, Constants.TestArm.pid.gravff());
        
    }

    public void updateTelemetry() {
        armMechanism.setAngle(Math.toDegrees(armSim.getAngleRads()));
    }


    @Override
    public void periodic() {
        this.updateTelemetry();
        SmartDashboard.putNumber("TestArm/Angle", Math.toDegrees(armSim.getAngleRads()));
    }

    double minVelocity = 0;
    @Override
    public void simulationPeriodic() {

        //update the WPILIB arm simulator
        armSim.setInput(armSparkMaxSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        armSim.update(Constants.kRobotLoopTime);

        double armMotorRPM = Units.radiansPerSecondToRotationsPerMinute(armSim.getVelocityRadPerSec());      
        SmartDashboard.putNumber("TestArm/armSparkMaxSim/getAppliedOutput", armSparkMaxSim.getAppliedOutput());
        SmartDashboard.putNumber("TestArm/armSim/VelocityRPM", armMotorRPM);        

        minVelocity = Math.min(minVelocity, armMotorRPM);
        SmartDashboard.putNumber("TestArm/armSim/MinVelocity", minVelocity);

        //update the SparkMax simulator
        armSparkMaxSim.iterate(armMotorRPM, RoboRioSim.getVInVoltage(), Constants.kRobotLoopTime);

        //update battery voltage
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armSparkMaxSim.getMotorCurrent()));
    

    }
}
