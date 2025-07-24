// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.LimelightHelpers;
import frc.robot.extensions.LimelightHelpers.PoseEstimate;
import swervelib.SwerveDrive;

public class VisionSubsystem extends SubsystemBase {
        // Basic targeting data
    double tx = LimelightHelpers.getTX(Constants.Vision.klimelightOne);  // Horizontal offset from crosshair to target in degrees
    double ty = LimelightHelpers.getTY(Constants.Vision.klimelightOne);  // Vertical offset from crosshair to target in degrees
    double ta = LimelightHelpers.getTA(Constants.Vision.klimelightOne);  // Target area (0% to 100% of image)
    boolean hasTarget = LimelightHelpers.getTV(Constants.Vision.klimelightOne); // Do you have a valid target?

    String climberCamer = Constants.Vision.klimelightOne; //used for driver vision, on the climber side
    String armCamera = Constants.Vision.klimelightTwo; //used for apriltags, and some driver vision, on the arm side

    public VisionSubsystem() {

    }
        //NOTE: the reason for all the if statments is to make sure the camera is only reading pose 
             //when there is a tag present and it is accurate (I think), im not  

        //this function grabs a pose estimate from the camera.
    public PoseEstimate getEstimatedGlobalPose(String armcamera) {
        if (LimelightHelpers.getTV(armCamera)) { //if the LLis reading a target
            hasTarget = true; //LL has a target
            PoseEstimate estimatedCameraPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(armCamera);

            if(Constants.kDebug) {
                SmartDashboard.putBoolean("limelightTV", LimelightHelpers.getTV(armCamera)); //shows there is a target reading 
                SmartDashboard.putNumber("limelightX", estimatedCameraPose.pose.getX()); //distance left-right from target
                SmartDashboard.putNumber("limelightY", estimatedCameraPose.pose.getY()); //distance front-back from target
            }
            return estimatedCameraPose; // returns a pose estimate
        }
        if(Constants.kDebug) {
            SmartDashboard.putBoolean("limelightTV", LimelightHelpers.getTV(armCamera));
            SmartDashboard.putNumber("limelightX", new PoseEstimate().pose.getX());
            SmartDashboard.putNumber("limelightY", new PoseEstimate().pose.getY());            
        }

        return new PoseEstimate(); // IDK abt ths (should return a pose estimate, not sure if its needed)
    }

        //this function updates the pose estimate, using the readings from the camera.
    public void updatePoseEstimator(SwerveDrive swerve) {
        PoseEstimate poseEstimate = getEstimatedGlobalPose(armCamera);
        if (poseEstimate != null) {
            swerve.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds); 
                //adds the vision mesurments to robots pose, .pose is the limelights pose2d, .timestampseconds is (whats is it?). These are both gotten from the LL helpsers.
        }
    }

        //Uses MT1 (2LL's)
    public void updatePosesEstimator(SwerveDrive swerve) {
        double maxTA = 0.4; //this it so you only take the LL mesurment when its a certain portion of the image that will give a accurate reading.
        String camera = null;
        String[] limelights = { "limelight-left", "limelight-right" }; //TODO: how does this work? and where should we replace them with our limelights?
        for (String limelight : limelights) {
            if (LimelightHelpers.getTV(limelight) && LimelightHelpers.getTA(limelight) > maxTA) {
                maxTA = LimelightHelpers.getTA(limelight);
                camera = limelight;
            }
        }
        if (camera != null) {
            PoseEstimate poseEst = getEstimatedGlobalPose(camera);
            swerve.addVisionMeasurement(poseEst.pose, poseEst.timestampSeconds);
            if(Constants.kDebug) {
                SmartDashboard.putBoolean("limelightTV", true);                
            }

            hasTarget = true;
        } else {
            hasTarget = false;
            if(Constants.kDebug) {
                SmartDashboard.putBoolean("limelightTV", false);                
            }

        }
    }

        //Uses MT2 (1LL)
    public void updatePosesEstimatorMT2(SwerveDrive swerve) {
        double maxta = 0;
        String camera = null;
        PoseEstimate mt2 = new PoseEstimate();
        String[] limelights = { "limelight-left", "limelight-right" }; // TODO: how does this work? and where should
                                                                       // we replace them with our limelights?
        for (String limelight : limelights) {
            LimelightHelpers.SetRobotOrientation(limelight, swerve.getPose().getRotation().getDegrees(), 0, 0, 0, 0,
                    0);
            LimelightHelpers.PoseEstimate megaTag2Pose = LimelightHelpers
                    .getBotPoseEstimate_wpiBlue_MegaTag2(limelight);

            if (megaTag2Pose.tagCount > 0) {
                // we have a tag!
                // if the TA is larger than the other camera
                if (LimelightHelpers.getTA(limelight) > maxta) {
                    maxta = LimelightHelpers.getTA(limelight);
                    mt2 = megaTag2Pose;
                    camera = limelight;
                }
            }
        }
        if (camera != null) {
            swerve.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
            if(Constants.kDebug) {
                SmartDashboard.putBoolean("limelightTV", true);                
            }

        } else {
            if(Constants.kDebug) {
                SmartDashboard.putBoolean("limelightTV", false);                
            }
        }
    }

    public PoseEstimate[] getEstimatedGlobalPose(String[] limelights) {
        PoseEstimate[] poseEsts = new PoseEstimate[limelights.length];
        int num = 0;
        for (String limelight : limelights) {
            if (LimelightHelpers.getTV(limelight)) {
                PoseEstimate poseEst = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);
                poseEsts[num] = poseEst;
            } else {
                poseEsts[num] = null;
            }
            num++;
        }
        return poseEsts;
        // IDK abt ths, (I think the reason for their uncertainty is because they don't know wethere you should return a vision mesurment if they arnt seeing a tag)
    }

  // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  public double limelight_aim_proportional() {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= Constants.SwerveSubsystem.MAX_SPEED;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

 public double limelight_range_proportional() {    
    double kP = .1;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    targetingForwardSpeed *= Constants.SwerveSubsystem.MAX_SPEED;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }


    public void periodic() {

    }
}
