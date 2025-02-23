package frc.robot.extensions;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

public class SparkMaxPIDTunerArmPosition extends SparkMaxPIDTunerPosition {

    protected GravityAssistedFeedForward ffController;
    private GenericEntry gravityFFEntry;
    private GenericEntry minimumFFEntry;
    private double gravFF0, minFF0;

    public SparkMaxPIDTunerArmPosition(String name, SparkMax motor, ControlType controlType, GravityAssistedFeedForward gravityFFController) {
        super(name,motor,controlType);

        this.ffController = gravityFFController;
        this.gravFF0 = ffController.getGravityFF();
        this.minFF0 = ffController.getMinimumFF();

        //setup items specific to this inherited class
        if(!super.isShuffleboardCreated()) {
            setupShuffleboard();            
        }
    }

    protected void setupShuffleboard() {
        super.setupShuffleboard();
        // setup gravity assist interface in Shuffleboard
        this.minimumFFEntry = super.getValueTuningLayout().add("Minimum FF", this.ffController.getMinimumFF())
            .withWidget(BuiltInWidgets.kTextView)
            //.withPosition(5,2)
            //.withSize(2,1)
            .getEntry();  
        this.gravityFFEntry = super.getValueTuningLayout().add("Gravity FF", this.ffController.getMinimumFF())
            .withWidget(BuiltInWidgets.kTextView)
            //.withPosition(5,2)
            //.withSize(2,1)
            .getEntry();  
        super.setShuffleboardCreated();
    }

    @Override
    public void applyTunerValues() {
        super.applyTunerValues();
        if(super.updateTimer.get() > SparkMaxPIDTunerBase.UPDATE_INTERVAL_SECONDS) {           
            ffController.setMinimumFF(this.minimumFFEntry.getDouble(0));
            ffController.setGravityFF(this.gravityFFEntry.getDouble(0));
        }
    }

    @Override
    public void resetTunerValues() {
        super.resetTunerValues();
        ffController.setGravityFF(this.gravFF0);
        ffController.setMinimumFF(this.minFF0);
    }

    @Override
    public void startMotor() {
        this.lastPositionReference = this.tuner.getSetpoint();
        double gravityFF = ffController.calculate(this.motor.getAbsoluteEncoder().getPosition());
        motor.getClosedLoopController().setReference(this.lastPositionReference, controlType, ClosedLoopSlot.kSlot0, gravityFF + super.arbitraryFFEntry.getDouble(0));

    }

}
