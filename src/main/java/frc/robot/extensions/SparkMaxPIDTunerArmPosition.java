// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extensions;

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

        if(this.getVerbosity() == Verbosity.commands || this.getVerbosity() == Verbosity.all) {
            StringBuilder sb = new StringBuilder();
            if(this.gravityFFEntry != null && this.minimumFFEntry != null){
                sb.append("|  Gravity FF: " + df10.format(gravityFFEntry.getDouble(0)) + " - ");
                sb.append("|  Minimum FF: " + df10.format(minimumFFEntry.getDouble(0)) );
            }
            System.out.println(sb.toString());            
        }
    }

    @Override
    public void resetTunerValues() {
        super.resetTunerValues();
        ffController.setGravityFF(this.gravFF0);
        ffController.setMinimumFF(this.minFF0);
        if(this.gravityFFEntry != null && this.minimumFFEntry != null){
            this.gravityFFEntry.setDouble(this.gravFF0);
            this.minimumFFEntry.setDouble(this.minFF0);            
        }

    }

    public void periodic() {
        double gravityFF = ffController.calculate(this.motor.getAbsoluteEncoder().getPosition());
        super.periodic(gravityFF, this.getArbitraryFF());
    }

}