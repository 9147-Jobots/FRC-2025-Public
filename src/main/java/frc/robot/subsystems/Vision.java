// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.constants.VisionConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private final PhotonCamera cameraFront; //creating a new photon camera
  private final PhotonCamera cameraBack;
  private final Transform3d cameraFrontToRobot; //creating a new transform3d
  private final Transform3d cameraBackToRobot; //creating a new transform3d
  public static PhotonPoseEstimator frontPoseEstimator; //creating a new photon pose estimator
  public static PhotonPoseEstimator backPoseEstimator; //creating a new photon pose estimator
  public static PhotonPipelineResult lastResultFront; //creating a new photon pipeline result
  public static PhotonPipelineResult lastResultBack; //creating a new photon pipeline result
  public static PhotonPipelineResult result;
  public static PhotonPipelineResult lastResult;
  private static EstimatedRobotPose frontPose; //creating a new estimated robot pose
  private static EstimatedRobotPose backPose; //creating a new estimated robot pose
  private static EstimatedRobotPose pose;
  private AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(); //creating a new apriltag field layout
  
  public Vision() {
    // This method will be called once per scheduler run

    //creating a new photon camera
    cameraFront = new PhotonCamera("Front");
    cameraBack = new PhotonCamera("Back");
    //updating the camera
    lastResultFront = cameraFront.getLatestResult();
    lastResultBack = cameraBack.getLatestResult();

    //creating a new transform3d
    cameraFrontToRobot = new Transform3d(
      new Translation3d(
          VisionConstants.cameraFront.x, 
          VisionConstants.cameraFront.y, 
          VisionConstants.cameraFront.z), 
      new Rotation3d(
          VisionConstants.cameraFront.pitch, 
          VisionConstants.cameraFront.roll, 
          VisionConstants.cameraFront.yaw
      ));
    
    cameraBackToRobot = new Transform3d(
      new Translation3d(
          VisionConstants.cameraBack.x, 
          VisionConstants.cameraBack.y, 
          VisionConstants.cameraBack.z), 
      new Rotation3d(
          VisionConstants.cameraBack.pitch, 
          VisionConstants.cameraBack.roll, 
          VisionConstants.cameraBack.yaw
      ));

      //creating a new photon pose estimator
      frontPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, cameraFrontToRobot);
      frontPose = frontPoseEstimator.update(lastResultFront).orElse(null);

      backPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, cameraBackToRobot);
      backPose = backPoseEstimator.update(lastResult).orElse(null);
  }

  public static EstimatedRobotPose GetFieldPose() {
    if (getBackPoseAmbiguity() != -1 && getFrontPoseAmbiguity() != -1) {
      if (getBackPoseAmbiguity() < getFrontPoseAmbiguity()) {
        return backPose;
      } else {
        return frontPose;
      }
    }

    if (getBackPoseAmbiguity() != -1) {
      return backPose;
    } 
    if (getFrontPoseAmbiguity() != -1) {
      return frontPose;
    }

    return null;
  }

  public static double getFrontPoseAmbiguity() {
    if (lastResultFront == null) {
      return -1;
    }
    if (lastResultFront.getTargets().isEmpty()) {
      return -1;
    }
    return lastResultFront.getBestTarget().getPoseAmbiguity();
  }

  public static double getBackPoseAmbiguity() {
    if (lastResultBack == null) {
      return -1;
    }
    if (lastResultBack.getTargets().isEmpty()) {
      return -1;
    }
    return lastResultBack.getBestTarget().getPoseAmbiguity();
  }

  public static void updatePosition() {
    backPose = backPoseEstimator.update(lastResultBack).orElse(null);
    frontPose = frontPoseEstimator.update(lastResultFront).orElse(null);
  }
  
  @Override
  public void periodic() {
    // Instantiate Photon Camera with cam name
    lastResultFront = cameraFront.getLatestResult();
    lastResultBack = cameraBack.getLatestResult();
    updatePosition();

    if (result.hasTargets()) {
      //var target = result.getBestTarget();
      //var camToTarget = target.getBestCameraToTarget();
      // SmartDashboard.putBoolean("Has targets", true);
      // SmartDashboard.putNumber("x Distance to tag", camToTarget.getX());
      // SmartDashboard.putNumber("y Distance to tag", camToTarget.getY());
      // SmartDashboard.putNumber("Angle to tag", camToTarget.getRotation().getAngle());
    } else {
      SmartDashboard.putBoolean("Has targets", false);
    }

    //update pose to Smartdashboard if result isn't null
    pose = GetFieldPose();
    if (pose != null) {
      SmartDashboard.putNumber("Pose : X", pose.estimatedPose.getTranslation().getX());
      SmartDashboard.putNumber("Pose : Y", pose.estimatedPose.getTranslation().getY());
      SmartDashboard.putNumber("Pose : Z", pose.estimatedPose.getTranslation().getZ());
      SmartDashboard.putNumber("Pose : Rotation", pose.estimatedPose.getRotation().getAngle());
    }
  }
}
