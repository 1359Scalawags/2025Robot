// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extensions;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

public class ArmPosition {

    public final double pulley;
    public final double elbow;

    public ArmPosition(double pulley, double elbow){
        this.pulley = pulley;
        this.elbow = elbow;
    }
        //TODO: tune these tolerances?
    public boolean isNear(ArmPosition position){
        if(MathUtil.isNear(this.pulley, position.pulley, Constants.ArmSubsystem.Pulley.kTolerance) && 
            MathUtil.isNear(this.elbow, position.elbow, Constants.ArmSubsystem.Elbow.kTolerance)){
            return true;
        } else {
            return false;
        }
    }


}