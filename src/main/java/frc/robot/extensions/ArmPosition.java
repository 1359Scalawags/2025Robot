package frc.robot.extensions;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

public class ArmPosition {

    public final double pulley;
    public final double elbow;
    public final double wrist;

    public ArmPosition(double pulley, double elbow, double wrist){
        this.pulley = pulley;
        this.elbow = elbow;
        this.wrist = wrist;
    }

    public boolean isNear(ArmPosition position){
        if(MathUtil.isNear(this.pulley, position.pulley, 0) && 
            MathUtil.isNear(this.pulley, position.pulley, 0) &&
            MathUtil.isNear(this.pulley, position.pulley, 0)){
            return true;
        } else {
            return false;
        }

    }


}
