package frc.robot.extensions;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Robot;

public class SimableSparkMax extends SendableCANSparkMax {
    
        private SimDevice simSparkMax;
        private SimDouble simSpeed;

        public SimableSparkMax (int motorID, MotorType type, String name){
            super(motorID, type);

            if(!Robot.isReal()){
                simSparkMax = SimDevice.create(name);

                if(simSparkMax != null){
                    simSpeed = simSparkMax.createDouble("Speed", Direction.kOutput, 0.0);
                } else {
                    System.out.print("STOOOPPP!!!!!");
                }
            }
        }
        public SimableSparkMax (int motorID, MotorType type){
            this(motorID,type,Integer.toString(motorID));
        }

        @Override
        public void set (double speed){
            if (simSparkMax != null){
                simSpeed.set(speed);
            } else {
                super.set(speed);
            }
            
        }

        @Override
        public double get(){
            if (simSparkMax != null){
                return simSpeed.get();
            } else {
                return super.get();
            }
        }

        @Override 
        public void setVoltage(double outputVolts){
             if (simSparkMax != null){
                set(outputVolts / RobotController.getBatteryVoltage());
            } else {
                super.setVoltage(outputVolts);
            }
        }
}
