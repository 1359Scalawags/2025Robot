package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
        // Basic targeting data
    double tx = LimelightHelpers.getTX(Constants.Vision.klimelightOne);  // Horizontal offset from crosshair to target in degrees
    double ty = LimelightHelpers.getTY(Constants.Vision.klimelightOne);  // Vertical offset from crosshair to target in degrees
    double ta = LimelightHelpers.getTA(Constants.Vision.klimelightOne);  // Target area (0% to 100% of image)
    boolean hasTarget = LimelightHelpers.getTV(Constants.Vision.klimelightOne); // Do you have a valid target?

    double txnc = LimelightHelpers.getTXNC(Constants.Vision.klimelightOne);  // Horizontal offset from principal pixel/point to target in degrees
    double tync = LimelightHelpers.getTYNC(Constants.Vision.klimelightOne);  // Vertical  offset from principal pixel/point to target in degrees
    // HttpCamera Limelight2 = new HttpCamera("Climber Camera", "http://10.13.59.15:5800", HttpCameraKind.kMJPGStreamer);
    // HttpCamera Limelight3G = new HttpCamera("Arm Camera", "", HttpCameraKind.kMJPGStreamer);


    public VisionSubsystem() {
        // CameraServer.addCamera
        // CameraServer.putVideo(Limelight2);
        // CameraServer.getInstance().addCamera(Limelight2);
        // Shuffleboard.getTab("Camera").add(Limelight2);
        // CameraServer.startAutomaticCapture(Limelight2);
        // Shuffleboard.getTab("Camera").add(Limelight2);
        // try {
        //     Shuffleboard.getTab("Camera").add(Limelight2);
        // } catch (Exception ex) {
        //     System.out.println("Camera failed to initialize.");
        // }

    }

    public void periodic() {

        // This method will be called once per scheduler run


            //TODO: Figure out megatag how to use this megatag code.
        //     LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        // if (limelightMeasurement.tagCount >= 2) {  // Only trust measurement if we see multiple tags
        //     m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
        //     m_poseEstimator.addVisionMeasurement(
        //         limelightMeasurement.pose,
        //         limelightMeasurement.timestampSeconds
    //);

//         // First, tell Limelight your robot's current orientation
//     double robotYaw = m_gyro.getYaw();  
//     LimelightHelpers.SetRobotOrientation("", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

//         // Get the pose estimate
//     LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("");

//         // Add it to your pose estimator
//     m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
//     m_poseEstimator.addVisionMeasurement(
//         limelightMeasurement.pose,
//         limelightMeasurement.timestampSeconds
// );

    }
}
