package frc.robot.extensions;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;

import java.util.Map;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.Tuning.Apply_Values;
import frc.robot.commands.Tuning.Reset_Values;
import frc.robot.commands.Tuning.Start_Motor;
import frc.robot.commands.Tuning.Stop_Motor;

public abstract class SparkMaxPIDTunerBase {
    private String name;
    protected SparkMax motor;
    private ControlType controlType;
    private ShuffleboardLayout valueTunerLayout;
    private ShuffleboardLayout commandButtonLayout;
    private ShuffleboardLayout encoderFeedbackLayout;
    protected ClosedLoopConfigAccessor configAccessor;
    private PIDController tuner;    
    private ShuffleboardTab tab;
    private double p0, i0, d0;
    private double reference; 
    protected Timer updateTimer;
    private boolean shuffleboardCreated;

    public static double UPDATE_INTERVAL_SECONDS = 0.5;

    public SparkMaxPIDTunerBase(String name, SparkMax motor) {
        this.name = name;
        this.shuffleboardCreated = false;
        this.motor = motor;
        this.configAccessor = motor.configAccessor.closedLoop;
        this.controlType = ControlType.kDutyCycle;
        this.p0 = configAccessor.getP();
        this.i0 = configAccessor.getI();
        this.d0 = configAccessor.getD();

        this.tuner = new PIDController(this.p0, this.i0, this.d0);

        updateTimer = new Timer();
        updateTimer.reset();
        updateTimer.start();

        if(!this.isShuffleboardCreated()) {
            setupShuffleboard();            
        }

    }

    protected ControlType getControlType() {
        return this.controlType;
    }

    protected void setControlType(ControlType type) {
        this.controlType = type;
    }

    protected double getReference() {
        return this.reference;
    }

    protected boolean isShuffleboardCreated() {
        return this.shuffleboardCreated;
    }

    protected void setShuffleboardCreated() {
        this.shuffleboardCreated = true;
    }

    protected void setupShuffleboard() {
        // setup interface in Shuffleboard
        this.tab = Shuffleboard.getTab(this.name + " Tuner");
 
        this.commandButtonLayout = this.tab.getLayout("Commands", BuiltInLayouts.kGrid)
            .withPosition(0, 0)
            .withSize(6,1)
            .withProperties(Map.of("Label position", "HIDDEN","Number of columns", 4, "Number of rows", 1, "Show Glyph", true, "Glphy", "PLAY"));

        this.commandButtonLayout.add("Apply", new Apply_Values(this))
            .withPosition(0, 0)
            .withSize(2, 1);
        this.commandButtonLayout.add("Reset", new Reset_Values(this))
            .withPosition(1, 0)
            .withSize(2, 1);
        this.commandButtonLayout.add("Start", new Start_Motor(this))
            .withPosition(2, 0)
            .withSize(2, 1);
        this.commandButtonLayout.add("STOP!", new Stop_Motor(this))
            .withPosition(3, 0)
            .withSize(2, 1);

        this.valueTunerLayout = this.tab.getLayout("Value Tuner", BuiltInLayouts.kGrid)
            .withPosition(2, 1)
            .withSize(2,3)
            .withProperties(Map.of("Label position", "TOP","Number of columns", 3, "Number of rows", 3, "Show Glyph", true, "Glphy", "PENCIL"));


        this.tab.add("PID", tuner)
            .withWidget(BuiltInWidgets.kPIDController)
            .withPosition(0, 1)
            .withSize(1,3)
            .withProperties(Map.of("Label position", "TOP", "Show Glyph", true, "Glphy", "PENCIL"));   

        this.encoderFeedbackLayout = this.tab.getLayout("Feedback", BuiltInLayouts.kList)
            .withPosition(4, 1)
            .withSize(2,3)
            .withProperties(Map.of("Label position", "TOP","Number of columns", 3, "Number of rows", 3, "Show Glyph", true, "Glphy", "HEARTBEAT"));

        this.setShuffleboardCreated();
    }

    public void updateEncoderValues() {
        // base shows no encoder information by default
    }


    public void applyTunerValues() {
        SparkMaxConfig newConfig = new SparkMaxConfig();
        newConfig.closedLoop
            .p(tuner.getP())
            .i(tuner.getI())
            .d(tuner.getD());

        this.reference = tuner.getSetpoint();
        motor.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        StringBuilder sb = new StringBuilder();
        sb.append("P: " + configAccessor.getP() + " - ");
        sb.append("I: " + configAccessor.getI() + " - ");
        sb.append("D: " + configAccessor.getD() + " - ");
        System.out.println(sb.toString());

    }

    public void resetTunerValues() {
        tuner.setPID(this.p0, this.i0, this.d0);
        tuner.setSetpoint(0);
    }

    public void startMotor() {
        
    } 

    public void stopMotor() {
        motor.stopMotor();
    }

    protected ShuffleboardLayout getCommandButtonLayout() {
        return this.commandButtonLayout;
    }

    protected ShuffleboardLayout getValueTuningLayout() {
        return this.valueTunerLayout;
    }

    protected ShuffleboardLayout getEncoderFeedbackLayout() {
        return this.encoderFeedbackLayout;
    }

}
