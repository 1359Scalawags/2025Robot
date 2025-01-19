package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.Sendable;
import frc.robot.extensions.SendableCANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase{
  private SendableCANSparkMax babyLockingMotor;
  private SendableCANSparkMax positionMotor;

    public ClimberSubsystem() {
      babyLockingMotor = new SendableCANSparkMax(Constants.ClimberSubsystem.kBabyLockingMotorID, MotorType.kBrushless);
      positionMotor = new SendableCANSparkMax(Constants.ClimberSubsystem.kPositionMotorID, MotorType.kBrushless);
    }
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
      }
}
