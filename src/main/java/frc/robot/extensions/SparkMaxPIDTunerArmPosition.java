package frc.robot.extensions;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

public class SparkMaxPIDTunerArmPosition extends SparkMaxPIDTunerPosition implements ISparkMaxTuner {

    protected GravityAssistedFeedForward ffController;
    private GenericEntry gravityFFEntry;
    private GenericEntry minimumFFEntry;
    private double gravFF0, minFF0;

    public SparkMaxPIDTunerArmPosition(String name, SparkMax motor, ControlType controlType, GravityAssistedFeedForward gravityFFController) {
        super(name,motor,controlType);
        this.shuffleboardSetupRoutines.add(this::setupShuffleboard);
        this.ffController = gravityFFController;
        this.gravFF0 = ffController.getGravityFF();
        this.minFF0 = ffController.getMinimumFF();   
    }

    private boolean setupShuffleboard() {
        // setup gravity assist interface in Shuffleboard
        this.minimumFFEntry = super.getValueTuningLayout().add("Minimum FF", this.ffController.getMinimumFF())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(1,0)
            .withSize(1,1)
            .getEntry();  
        this.gravityFFEntry = super.getValueTuningLayout().add("Gravity FF", this.ffController.getGravityFF())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(1,1)
            .withSize(1,1)
            .getEntry();  

        return true;
    }

    @Override
    public void applyTunerValues() {
        super.applyTunerValues();         
        ffController.setMinimumFF(this.minimumFFEntry.getDouble(0));
        ffController.setGravityFF(this.gravityFFEntry.getDouble(0));

        StringBuilder sb = new StringBuilder();
        sb.append("Gravity FF: " + gravityFFEntry.getDouble(0) + " - ");
        sb.append("Minimum FF: " + minimumFFEntry.getDouble(0));
        System.out.println(sb.toString());
    }

    @Override
    public void resetTunerValues() {
        super.resetTunerValues();
        ffController.setGravityFF(this.gravFF0);
        ffController.setMinimumFF(this.minFF0);
        this.gravityFFEntry.setDouble(this.gravFF0);
        this.minimumFFEntry.setDouble(this.minFF0);
    }

    public void periodic() {
        double gravityFF = ffController.calculate(this.motor.getAbsoluteEncoder().getPosition());
        super.periodic(gravityFF, this.getArbitraryFF());
    }

}