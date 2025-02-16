package frc.robot.extensions;

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
}
