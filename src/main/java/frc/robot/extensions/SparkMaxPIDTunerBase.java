package frc.robot.extensions;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import java.util.ArrayList;
import java.util.Map;
import java.util.function.BooleanSupplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.commands.Tuning.Apply_Values;
import frc.robot.commands.Tuning.Reset_Values;
import frc.robot.commands.Tuning.Start_Motor;
import frc.robot.commands.Tuning.Stop_Motor;

public abstract class SparkMaxPIDTunerBase implements ISparkMaxTuner {
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
    protected double minReference, maxReference;
    protected Timer updateTimer;
    protected ArrayList<BooleanSupplier> shuffleboardSetupRoutines;
    private boolean isInitialized = false;
    private boolean isRunning = false;
    public GenericEntry isRunningEntry;

    public static double UPDATE_INTERVAL_SECONDS = 0.5;

    public SparkMaxPIDTunerBase(String name, SparkMax motor) {
        this.shuffleboardSetupRoutines = new ArrayList<BooleanSupplier>();
        this.shuffleboardSetupRoutines.add(this::setupShuffleboard);
        this.name = name;
        this.motor = motor;
        this.isRunning = false;
        this.configAccessor = motor.configAccessor.closedLoop;
        this.controlType = ControlType.kDutyCycle;
        this.p0 = configAccessor.getP();
        this.i0 = configAccessor.getI();
        this.d0 = configAccessor.getD();
        this.minReference = Double.MIN_VALUE;
        this.maxReference = Double.MAX_VALUE;
        this.tuner = new PIDController(this.p0, this.i0, this.d0);
        this.tuner.setSetpoint(MathUtil.clamp(this.tuner.getSetpoint(), this.minReference, this.maxReference));

        updateTimer = new Timer();
        updateTimer.reset();
        updateTimer.start();

    }

    public void buildShuffleboard() {
        for(BooleanSupplier function : shuffleboardSetupRoutines) {
            function.getAsBoolean();
        }
        this.isInitialized = true;
    }

    protected boolean isInitialized() {
        return this.isInitialized;
    }

    protected ControlType getControlType() {
        return this.controlType;
    }

    protected void setControlType(ControlType type) {
        this.controlType = type;
    }

    protected double getReference() {
        return MathUtil.clamp(this.reference, this.minReference, this.maxReference);
    }

    public void setSafeReferenceRange(double min, double max) {
        this.minReference = min;
        this.maxReference = max;
    }

    public double getMinReference() {
        return this.minReference;
    }

    public double getMaxReference() {
        return this.maxReference;
    }

    public void setRunningState(boolean motorIsRunning) {
        this.isRunning = motorIsRunning;
        this.isRunningEntry.setBoolean(this.isRunning);            
    }

    public boolean getIsRunning() {
        return this.isRunning;
    }

    private boolean setupShuffleboard() {
        // setup interface in Shuffleboard
        this.tab = Shuffleboard.getTab(this.name + " Tuner");

        //only add commands when in tuning mode
        if(Constants.kTuning) { 
            this.commandButtonLayout = this.tab.getLayout("Commands", BuiltInLayouts.kGrid)
                .withPosition(0, 0)
                .withSize(6,1)
                .withProperties(Map.of("Label position", "HIDDEN","Number of columns", 5, "Number of rows", 1, "Show Glyph", true, "Glphy", "PLAY"));
        
            this.commandButtonLayout.add("Apply", new Apply_Values(this))
                .withPosition(0, 0)
                .withSize(2, 1)
                .withWidget(BuiltInWidgets.kCommand);
            this.commandButtonLayout.add("Reset", new Reset_Values(this))
                .withPosition(1, 0)
                .withSize(2, 1)
                .withWidget(BuiltInWidgets.kCommand);
            this.commandButtonLayout.add("Start", new Start_Motor(this))
                .withPosition(2, 0)
                .withSize(2, 1)
                .withWidget(BuiltInWidgets.kCommand);
            this.commandButtonLayout.add("STOP!", new Stop_Motor(this))
                .withPosition(3, 0)
                .withSize(2, 1)
                .withWidget(BuiltInWidgets.kCommand);
            this.isRunningEntry = this.commandButtonLayout.add("Motor Running", this.isRunning)
                .withPosition(4, 0)
                .withSize(1, 1)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .getEntry();
        }

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

        return true;
    }

    public void updateEncoderValues() {
        // base shows no encoder information by default
        isRunningEntry.setBoolean(this.isRunning);
    }


    public void applyTunerValues() {
        SparkMaxConfig newConfig = new SparkMaxConfig();
        newConfig.closedLoop
            .p(tuner.getP())
            .i(tuner.getI())
            .d(tuner.getD());

        this.reference = tuner.getSetpoint();
        this.reference = MathUtil.clamp(this.reference, this.minReference, this.maxReference);
        tuner.setSetpoint(this.reference);

        motor.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        StringBuilder sb = new StringBuilder();
        sb.append("P: " + configAccessor.getP() + " - ");
        sb.append("I: " + configAccessor.getI() + " - ");
        sb.append("D: " + configAccessor.getD());
        System.out.println(sb.toString());
    }

    public void resetTunerValues() {
        tuner.setPID(this.p0, this.i0, this.d0);
        tuner.setSetpoint(MathUtil.clamp(0, this.minReference, this.maxReference));
    }

    public void startMotor() {
        if(RobotState.isEnabled()) {
            this.setRunningState(true);          
        }
    }

    public void stopMotor() {
        this.setRunningState(false);        
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

    protected void periodic(double gravityFF, double arbitraryFF) {
        if(!this.isInitialized) 
            return;

        if(RobotState.isDisabled()) {
            this.setRunningState(false);   
        }
        if(this.isRunning) {
            motor.getClosedLoopController().setReference(this.reference, this.controlType, ClosedLoopSlot.kSlot0, gravityFF + arbitraryFF);
        }
    }

}