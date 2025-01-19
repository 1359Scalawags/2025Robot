package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.Sendable;
import frc.robot.extensions.SendableCANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase{
  private SendableCANSparkMax babyLockingMotor;  // TODO: why are we locking babies? Is that legal?
  private SendableCANSparkMax positionMotor;

  private AbsoluteEncoder babyLockingEncoder;


    public ClimberSubsystem() {
      babyLockingMotor = new SendableCANSparkMax(Constants.ClimberSubsystem.kBabyLockingMotorID, MotorType.kBrushless);
      positionMotor = new SendableCANSparkMax(Constants.ClimberSubsystem.kPositionMotorID, MotorType.kBrushless);

      babyLockingEncoder = babyLockingMotor.getAbsoluteEncoder();
      AbsoluteEncoderConfig babyLockingConfig = new AbsoluteEncoderConfig();
      babyLockingConfig.zeroOffset(Constants.ClimberSubsystem.kPositionEncoderOffset).positionConversionFactor(Constants.ClimberSubsystem.kPositionConversionFactor);
      babyLockingConfig.apply(babyLockingConfig);

    }
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
      }
}
