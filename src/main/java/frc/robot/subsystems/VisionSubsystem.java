package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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

    public PoseEstimate getEstimatedGlobalPose(String armcamera) {
        if (LimelightHelpers.getTV(armCamera)) {
            hasTarget = true;
            PoseEstimate estimatedCameraPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(armCamera);

            SmartDashboard.putBoolean("limelightTV", LimelightHelpers.getTV(armCamera));
            SmartDashboard.putNumber("limelightX", estimatedCameraPose.pose.getX());
            SmartDashboard.putNumber("limelightY", estimatedCameraPose.pose.getY());
            return estimatedCameraPose;
        }
        SmartDashboard.putBoolean("limelightTV", LimelightHelpers.getTV(armCamera));
        SmartDashboard.putNumber("limelightX", new PoseEstimate().pose.getX());
        SmartDashboard.putNumber("limelightY", new PoseEstimate().pose.getY());
        return new PoseEstimate(); // IDK abt ths
    }

    public void updatePoseEstimator(SwerveDrive swerve) {
        PoseEstimate poseEst = getEstimatedGlobalPose("limelight-left");
        if (poseEst != null) {
            swerve.addVisionMeasurement(poseEst.pose, poseEst.timestampSeconds);
        }
    }

    // public void updatePosesEstimatorMT2(SwerveDrive swerve) {

    //     double maxta = 0;
    //     String camera = null;
    //     PoseEstimate mt2PoseEstimate = new PoseEstimate();
    //     String[] limelights = { "limelight-left", "limelight-right" }; // , "limelight-rear"
    //     for (String limelight : limelights) {
    //         LimelightHelpers.SetRobotOrientation(limelight, swerve.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    //         LimelightHelpers.PoseEstimate megaTag2Pose = LimelightHelpers
    //                 .getBotPoseEstimate_wpiBlue_MegaTag2(limelight);

    //         if (megaTag2Pose.tagCount > 0) {
    //             // we have a tag!
    //             // if the TA is larger than the other camera
    //             if (LimelightHelpers.getTA(limelight) > maxta) {
    //                 maxta = LimelightHelpers.getTA(limelight);
    //                 mt2PoseEstimate = megaTag2Pose;
    //                 camera = limelight;
    //             }

    //         }

    //     }
    //     if (camera != null) {
    //         swerve.addVisionMeasurement(mt2PoseEstimate.pose, mt2PoseEstimate.timestampSeconds);
    //         SmartDashboard.putBoolean("limelightTV", true);
    //     } else {
    //         SmartDashboard.putBoolean("limelightTV", false);
    //     }
    // }

    public void periodic() {

    }
}
