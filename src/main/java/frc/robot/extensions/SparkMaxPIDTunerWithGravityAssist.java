package frc.robot.extensions;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Tuning.ApplyTunerValues;
import frc.robot.commands.Tuning.ResetTunerValues;
import frc.robot.commands.Tuning.StartTunerMotor;
import frc.robot.commands.Tuning.StopTunerMotor;

public class SparkMaxPIDTunerWithGravityAssist extends SparkMaxPIDTunerPosition {

    protected GravityAssistedFeedForward ffController;
    private ShuffleboardLayout gravityAssistLayout;
    private GenericEntry gravityFFEntry;
    private GenericEntry minimumFFEntry;
    private double gravFF0, minFF0;

    public SparkMaxPIDTunerWithGravityAssist(String name, SparkMax motor, GravityAssistedFeedForward gravityFFController, ControlType controlType) {
        super(name,motor,controlType);
        super.setupShuffleboard(name);
        this.ffController = gravityFFController;
        this.gravFF0 = ffController.getGravityFF();
        this.minFF0 = ffController.getMinimumFF();
        setupShuffleboard(name);
    }

    protected void setupShuffleboard(String name) {
        // setup gravity assist interface in Shuffleboard
        this.minimumFFEntry = super.tab.add("Minimum FF", this.ffController.getMinimumFF())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(5,2)
            .withSize(2,1)
            .getEntry();  
            this.gravityFFEntry = super.tab.add("Gravity FF", this.ffController.getMinimumFF())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(5,2)
            .withSize(2,1)
            .getEntry();  
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
        motor.getClosedLoopController().setReference(this.lastPositionReference, controlType, ClosedLoopSlot.kSlot0, gravityFF + super.ffEntry.getDouble(0));

    }

}
