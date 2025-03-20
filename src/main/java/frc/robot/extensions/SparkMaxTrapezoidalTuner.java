package frc.robot.extensions;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import java.security.InvalidParameterException;
import java.text.DecimalFormat;
import java.util.Map;
import java.util.function.DoubleSupplier;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.GenericEntry;
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

public class SparkMaxTrapezoidalTuner implements ISparkMaxTuner {

    //motor and encoders
    private SparkMax motor;
    private ClosedLoopConfigAccessor configAccessor;    
    private ControlType controlType;

    //shuffleboard
    private String name;    
    private ShuffleboardTab tab;    
    private ShuffleboardLayout feedForwardLayout;
    private ShuffleboardLayout commandButtonLayout;
    private ShuffleboardLayout encoderFeedbackLayout;
    private ShuffleboardLayout trapezoidProfileLayout;

    //initial values
    private double p0, i0, d0, gravityFF0, minimumFF0, arbitraryFF0, velocity0, acceleration0;
    private double arbitraryFF, velocity, acceleration;
    
    //PID tuning
    private PIDController tuner;  

    //Trapezoidal profile tuning
    private Constraints motionConstraints;
    private TrapezoidProfile motionProfile;
    private GenericEntry velocityEntry, accelerationEntry;
    private State currentState, goalState;

    //FeedForward tuning
    protected GravityAssistedFeedForward ffController;
    private GenericEntry arbitraryFFEntry, gravityFFEntry, minimumFFEntry;    
    
    //Setpoints
    private double reference; 
    protected double minReference, maxReference;

    //Monitoring
    private GenericEntry actualPositionEntry;
    private GenericEntry actualVelocityEntry;
    private DoubleSupplier positionEncoderSupplier;
    private DoubleSupplier velocityEncoderSupplier;
    
    //Runtime
    public static double UPDATE_INTERVAL_SECONDS = 0.5;    
    protected Timer updateTimer;
    private boolean isRunning = false;
    private GenericEntry isRunningEntry;

    //Debugging
    private Verbosity debugVerbosity;
    private double debugIntervalSeconds;
    protected final static DecimalFormat df5 = new DecimalFormat("#.#####");
    protected final static DecimalFormat df10 = new DecimalFormat("#.##########"); 

    public enum Verbosity {
        none,
        commands,
        all
    }

    public SparkMaxTrapezoidalTuner(String name, SparkMax motor, ControlType controlType, GravityAssistedFeedForward gravityController) {
        this.name = name;
        this.motor = motor;
        this.controlType = controlType;        
        this.isRunning = false;
        this.debugVerbosity = Verbosity.commands;
        this.debugIntervalSeconds = 1.0;
        this.configAccessor = motor.configAccessor.closedLoop;
        this.controlType = controlType;
        this.p0 = configAccessor.getP();
        this.i0 = configAccessor.getI();
        this.d0 = configAccessor.getD();
        this.minReference = Double.MIN_VALUE;
        this.maxReference = Double.MAX_VALUE;
        this.tuner = new PIDController(this.p0, this.i0, this.d0);
        this.tuner.setSetpoint(MathUtil.clamp(this.tuner.getSetpoint(), this.minReference, this.maxReference));

        if(controlType != ControlType.kPosition) {
            throw new UnsupportedOperationException("Only position control is currently implemented.");
        }
        
        // encoder initialization
        if(this.configAccessor.getFeedbackSensor() == FeedbackSensor.kAbsoluteEncoder) {
            this.positionEncoderSupplier = motor.getAbsoluteEncoder()::getPosition;
            this.velocityEncoderSupplier = motor.getAbsoluteEncoder()::getVelocity;
        } else {
            this.positionEncoderSupplier = motor.getEncoder()::getPosition;
            this.velocityEncoderSupplier = motor.getEncoder()::getVelocity;
        }

        if(this.positionEncoderSupplier == null || this.velocityEncoderSupplier == null) {
            throw new NullPointerException("Encoder suppliers are not instantiated.");
        }


        this.velocity0 = 0;
        this.acceleration0 = 0;
        this.motionConstraints = null;
        this.motionProfile = null;

        // feed forward initialization
        this.ffController = gravityController;
        this.arbitraryFF0 = 0;    
        if(this.ffController != null) {
            this.gravityFF0 = this.ffController.getGravityFF();
            this.minimumFF0 = this.ffController.getMinimumFF();              
        }
  
        //build shuffleboard interface
        setupShuffleboard();

        //monitoring updates
        updateTimer = new Timer();
        updateTimer.reset();
        updateTimer.start();

    }

    public void setMotionProfile(double maxVelocity, double maxAcceleration) {
        this.acceleration0 = maxAcceleration;
        this.velocity0 = maxVelocity;
        this.acceleration = this.acceleration0;
        this.velocity = this.velocity0;
        rebuildMotionProfile();
    }

    public void setSafeReferenceRange(double min, double max) {
        this.minReference = min;
        this.maxReference = max;
    }

    public void setVerbosity(Verbosity verbosity, double updateInterval) {
        this.debugVerbosity = verbosity;
        this.debugIntervalSeconds = updateInterval;
    }

    private void setRunningState(boolean motorIsRunning) {
        this.isRunning = motorIsRunning;
        this.isRunningEntry.setBoolean(this.isRunning);            
    }

    private void setupShuffleboard() {
        // setup interface in Shuffleboard
        this.tab = Shuffleboard.getTab(this.name + " Tuner");

        this.setupShuffleboardCommands();
        this.setupShuffleboardTrapezoidal();
        this.setupShuffleboardPID();
        this.setupShuffleboardFeedForward();
        this.setupShuffleboardMonitoring();
    }

    private void setupShuffleboardCommands() {

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
    }

    private void setupShuffleboardPID() {
        this.tab.add("PID", tuner)
            .withWidget(BuiltInWidgets.kPIDController)
            .withPosition(0, 1)
            .withSize(2,3)
            .withProperties(Map.of("Label position", "TOP", "Show Glyph", true, "Glphy", "PENCIL"));   
    }

    private void setupShuffleboardTrapezoidal() {
        this.trapezoidProfileLayout = this.tab.getLayout("Trapezoid Profile", BuiltInLayouts.kGrid)
            .withPosition(2, 1)
            .withSize(1,3)
            .withProperties(Map.of("Label position", "TOP","Number of columns", 1, "Number of rows", 3, "Show Glyph", true, "Glphy", "PENCIL"));

        this.accelerationEntry = this.trapezoidProfileLayout.add("Acceleration", this.acceleration0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 0)
            .withSize(1,1)
            .getEntry();
        this.velocityEntry = this.trapezoidProfileLayout.add("Velocity", this.velocity0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 1)
            .withSize(1,1)
            .getEntry();
    }

    private void setupShuffleboardFeedForward() {
        this.feedForwardLayout = this.tab.getLayout("Feed Forward", BuiltInLayouts.kGrid)
            .withPosition(3, 1)
            .withSize(1,3)
            .withProperties(Map.of("Label position", "TOP","Number of columns", 1, "Number of rows", 3, "Show Glyph", true, "Glphy", "PENCIL"));

        this.arbitraryFFEntry = this.feedForwardLayout.add("Arbitrary FF", this.arbitraryFF0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0,0)
            .withSize(1,1)
            .getEntry();  

        if(this.ffController != null) {
            this.minimumFFEntry = this.feedForwardLayout.add("Min Grav FF", this.ffController.getMinimumFF())
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(0,1)
                .withSize(1,1)
                .getEntry();  
            this.gravityFFEntry = this.feedForwardLayout.add("Gravity FF", this.ffController.getGravityFF())
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(0,2)
                .withSize(1,1)
                .getEntry();              
        } 


    }

    private void setupShuffleboardMonitoring() {
        this.encoderFeedbackLayout = this.tab.getLayout("Encoder Monitoring", BuiltInLayouts.kList)
            .withPosition(5, 1)
            .withSize(2,3)
            .withProperties(Map.of("Label position", "TOP","Number of columns", 1, "Number of rows", 3, "Show Glyph", true, "Glphy", "HEARTBEAT"));

        this.actualPositionEntry = this.encoderFeedbackLayout.add("Position", this.positionEncoderSupplier.getAsDouble())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0,0)
            .withSize(1,1)
            .getEntry();  
        this.actualVelocityEntry = this.encoderFeedbackLayout.add("Velocity", this.positionEncoderSupplier.getAsDouble())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0,1)
            .withSize(1,1)
            .getEntry();  
    }

    protected void updateEncoderValues() {
        if(updateTimer.get() > UPDATE_INTERVAL_SECONDS) {
            if(this.actualPositionEntry != null)
                this.actualPositionEntry.setDouble(positionEncoderSupplier.getAsDouble());
                this.actualVelocityEntry.setDouble(velocityEncoderSupplier.getAsDouble());
            updateTimer.reset();      
        }    
    }

    public void applyTunerValues() {
        // pull values into variables
        this.pullFromGenericEntries();        
        
        // apply PID
        SparkMaxConfig newConfig = new SparkMaxConfig();
            newConfig.closedLoop
                .p(tuner.getP())
                .i(tuner.getI())
                .d(tuner.getD());
        motor.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // update setpoint
        this.reference = tuner.getSetpoint();
        this.reference = MathUtil.clamp(this.reference, this.minReference, this.maxReference);
        tuner.setSetpoint(this.reference);

        if(this.velocity != this.motionConstraints.maxVelocity || this.acceleration != this.motionConstraints.maxAcceleration) {
            rebuildMotionProfile();
        }

        if(this.debugVerbosity == Verbosity.commands || this.debugVerbosity == Verbosity.all) {
            System.out.println(this.getDebugString("Apply"));
        }
    }

    private String getDebugString(String headerName) {
        StringBuilder sb = new StringBuilder();
        sb.append("\n#### " + headerName.toUpperCase() + ": " + this.name + " ####");
        sb.append("|  P: " + df10.format(configAccessor.getP()) + " - ");
        sb.append("|  I: " + df10.format(configAccessor.getI()) + " - ");
        sb.append("|  D: " + df10.format(configAccessor.getD()) + "\n");
        sb.append("|  FF: Arbitrary: " + df10.format(this.arbitraryFF) + " - ");
        if(ffController != null) {
            sb.append("|  Gravity: " + df10.format(this.ffController.getGravityFF()) + " - ");
            sb.append("|  Min Grav: " + df10.format(this.ffController.getMinimumFF()) + "\n");            
        }
        return sb.toString();    
    }

    public void resetTunerValues() {

        this.reference = MathUtil.clamp(0, this.minReference, this.maxReference);
        this.arbitraryFF = this.arbitraryFF0;
        if(ffController != null) {
            this.ffController.setGravityFF(this.gravityFF0);
            this.ffController.setMinimumFF(this.minimumFF0);
        }
        this.acceleration = this.acceleration0;
        this.velocity = this.velocity0;

        tuner.setPID(this.p0, this.i0, this.d0);    
        tuner.setSetpoint(this.reference);    

        this.pushToGenericEntries();     
        
        rebuildMotionProfile();

    }

    private void pushToGenericEntries() {
        // update feedforward
        this.arbitraryFFEntry.setDouble(this.arbitraryFF);
        if(ffController != null) {
            this.gravityFFEntry.setDouble(this.ffController.getGravityFF());
            this.minimumFFEntry.setDouble(this.ffController.getMinimumFF());            
        }

        // update trapezoid profile
        this.accelerationEntry.setDouble(this.acceleration);
        this.velocityEntry.setDouble(this.velocity);
    }

    private void pullFromGenericEntries() {
        // update feedforward
        this.arbitraryFF = arbitraryFFEntry.getDouble(this.arbitraryFF0);
        if(ffController != null) {
            double gff = gravityFFEntry.getDouble(this.gravityFF0);
            double mff = minimumFFEntry.getDouble(this.minimumFF0);
            this.ffController.setGravityFF(gff);
            this.ffController.setMinimumFF(mff);            
        }

        // update trapezoid profile
        this.velocity = velocityEntry.getDouble(this.velocity0);
        this.acceleration = accelerationEntry.getDouble(this.acceleration0);
    }

    private void rebuildMotionProfile() {
        this.motionConstraints = new Constraints(this.velocity, this.acceleration);
        this.motionProfile = new TrapezoidProfile(this.motionConstraints);
        this.currentState = new State(this.positionEncoderSupplier.getAsDouble(), this.velocityEncoderSupplier.getAsDouble());
        if(this.controlType == ControlType.kPosition) {
            this.goalState = new State(this.reference, 0);
        } else {
            throw new UnsupportedOperationException("Only position control is currently implemented.");
        }
    }

    public void startMotor() {
        if(motionProfile == null) {
            throw new InvalidParameterException("The motion profile must be initialized.");
        }
        System.out.println("\n#### START: " + this.name + " #### \n");
        rebuildMotionProfile();
        if(RobotState.isEnabled()) {
            this.setRunningState(true);          
        }
    }

    public void stopMotor() {
        System.out.println("\n#### STOP: " + this.name + " #### \n");
        this.setRunningState(false);        
        motor.stopMotor();
    }


    int count = 0;
    public void periodic() {
        
        // disable if robot is disabled
        if(RobotState.isDisabled() && this.isRunning) {
            this.setRunningState(false);   
            isRunningEntry.setBoolean(false);
        }

        // run the motor if enabled
        if(this.isRunning) {
            // combine feedforward values
            double feedForward = arbitraryFF;
            if(ffController != null) {
                feedForward += ffController.calculate(this.positionEncoderSupplier.getAsDouble());
            }

            if(this.controlType == ControlType.kPosition) {
                goalState = new State(this.reference, 0);    
                currentState = motionProfile.calculate(Constants.kRobotLoopTime, currentState, goalState);
                motor.getClosedLoopController().setReference(currentState.position, this.controlType, ClosedLoopSlot.kSlot0, feedForward);
            } 
            else {
                throw new InvalidParameterException("Only position control is supported.");
            }
        }            

        // show debug info if verbosity is high
        if(this.debugVerbosity == Verbosity.all) {        
            count++;        
            if(count * Constants.kRobotLoopTime > this.debugIntervalSeconds) {
                System.out.println(getDebugString("Periodic"));      
                count = 0;                      
            }
        } 

        this.updateEncoderValues();
    }
}