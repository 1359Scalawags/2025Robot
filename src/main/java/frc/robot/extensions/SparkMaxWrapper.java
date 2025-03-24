package frc.robot.extensions;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;

import com.revrobotics.spark.SparkSim;


//XXX: I don't think this is necessary
public class SparkMaxWrapper extends SparkMax 
{
    private SparkSim simulatedSparkMax;
    public SparkMaxWrapper(int deviceId, MotorType motorType, DCMotor dcMotor) {
        super(deviceId, motorType);
        simulatedSparkMax = new SparkSim(this, dcMotor);

    }
    
}
