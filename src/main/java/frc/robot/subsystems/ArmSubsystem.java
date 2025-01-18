package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.frc.CANSparkMax;

public class ArmSubsystem extends SubsystemBase {

    private SendableCANSparkMax wristMotor; 
    private SendableCANSparkMax armMotor;
    private SendableCANSparkMax pulleyMotor;
    private SendableCANSparkMax reversedScrewMotor;




    public ArmSubsystem() {
      wristMotor = new SendableCANSparkMax(Constants.ArmSubsystem.kWristMotorID,MotorType.kBrushless);
      armMotor = new SendableCANSparkMax(Constants.ArmSubsystem.kArmMotorID,MotorType.kBrushless);
      pulleyMotor = new SendableCANSparkMax(Constants.ArmSubsystem.kPulleyMotorID,MotorType.kBrushless);
      reversedScrewMotor = new SendableCANSparkMax(Constants.ArmSubsystem.kPulleyMotorID,MotorType.kBrushless);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
