// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extensions;

import com.revrobotics.spark.SparkAbsoluteEncoder;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/*
 * This is a wrapper class solely for use with the shuffleboard / smart dashboard.
 * Do no use for 
 */
public class SendableSparkAbsoluteEncoder implements Sendable {

    SparkAbsoluteEncoder absoluteEncoder;

    public SendableSparkAbsoluteEncoder(SparkAbsoluteEncoder absoluteEncoder) {
        this.absoluteEncoder = absoluteEncoder;
    }

    public double getPosition() {
        return absoluteEncoder.getPosition();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.setActuator(false);
        builder.addDoubleProperty("Absolute Position", this::getPosition, null);
    }
}
