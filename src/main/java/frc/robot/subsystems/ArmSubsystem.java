package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.SendableCANSparkMax;


public class ArmSubsystem extends SubsystemBase {

    private SendableCANSparkMax wristMotor; 
    private SendableCANSparkMax armMotor;
    private SendableCANSparkMax pulleyMotor;
    private SendableCANSparkMax reversedScrewMotor; //TODO: What does this motor do as opposed to the specific hardware used to do it?




    public ArmSubsystem() {
      wristMotor = new SendableCANSparkMax(Constants.ArmSubsystem.kWristMotorID,MotorType.kBrushless);
      armMotor = new SendableCANSparkMax(Constants.ArmSubsystem.kArmMotorID,MotorType.kBrushless);
      pulleyMotor = new SendableCANSparkMax(Constants.ArmSubsystem.kPulleyMotorID,MotorType.kBrushless);
      reversedScrewMotor = new SendableCANSparkMax(Constants.ArmSubsystem.kReversedScrewMotorID,MotorType.kBrushless);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
