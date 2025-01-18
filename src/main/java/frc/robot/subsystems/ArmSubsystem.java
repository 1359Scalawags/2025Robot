package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.frc.CANSparkMax;

public class ArmSubsystem extends SubsystemBase {

    private CANSparkMax wristMotor; 
    private CANSparkMax armMotor;




    public ArmSubsystem() {
      wristMotor = new CANSparkMax()
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
