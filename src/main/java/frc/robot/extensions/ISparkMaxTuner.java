package frc.robot.extensions;

public interface ISparkMaxTuner {
    void applyTunerValues();
    void stopMotor();
    void startMotor();
    void resetTunerValues();
    void periodic();
}
