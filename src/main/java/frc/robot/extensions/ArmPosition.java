package frc.robot.extensions;

import edu.wpi.first.math.MathUtil;

public class ArmPosition {

    public final double pulley;
    public final double elbow;
    public final double wrist;

    public ArmPosition(double pulley, double elbow, double wrist){
        this.pulley = pulley;
        this.elbow = elbow;
        this.wrist = wrist;
    }
        //TODO: tune these tolerances?
    public boolean isNear(ArmPosition position){
        if(MathUtil.isNear(this.pulley, position.pulley, 5) && 
            MathUtil.isNear(this.pulley, position.pulley, 5)&&
            MathUtil.isNear(this.pulley, position.pulley, 5)){
            return true;
        } else {
            return false;
        }

    }


}