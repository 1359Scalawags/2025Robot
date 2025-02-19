package frc.robot.extensions;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

///*
/// Represents an arm position based on the positions of its different motors
/// 
///  */
public class ArmPosition {

    public final double pulley;  
    public final double elbow;  
    public final double wrist;    

    public ArmPosition(double pulley, double elbow, double wrist) {
        this.pulley = pulley;
        this.elbow = elbow;
        this.wrist = wrist;
    }

    public boolean isNear(ArmPosition position) {
        if(MathUtil.isNear(this.pulley, position.pulley, Constants.ArmSubsystem.Pulley.kPositionTolerance) &&
           MathUtil.isNear(this.elbow, position.elbow, Constants.ArmSubsystem.Elbow.kPositionTolerance) &&
           MathUtil.isNear(this.wrist, position.wrist, Constants.ArmSubsystem.Wrist.kPositionTolerance))
           return true;

        return false;
    }
}
