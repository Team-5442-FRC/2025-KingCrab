// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Telemetry;


public class Vision extends SubsystemBase {
  
  ArrayList<Object> cameras = new ArrayList<Object>();

  
    public final static PhotonCamera ThriftyCamera = new PhotonCamera("Thrifty_Camera");
    public final static PhotonCamera MicrosoftCamera = new PhotonCamera("Microsoft_Camera");

    PhotonPoseEstimator microsoftPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0)));
    PhotonPoseEstimator thriftyPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0)));

    Telemetry logger = RobotContainer.logger;

  public Vision() {
    cameras.add(MicrosoftCamera);
    cameras.add(ThriftyCamera);
  }
  

  public Pose2d getFieldPose() {
    double fX = 0;
    double fY = 0;
    double fR = 0;
    double tot = 0;

    for (Object camera: cameras) {
      if (camera instanceof CalculatedLimelight) {
        if(((CalculatedLimelight)camera).hasTarget()) {
          fX += ((CalculatedLimelight)camera).getFieldPose().getX() * ((CalculatedLimelight)camera).getTrust();
          fY += ((CalculatedLimelight)camera).getFieldPose().getY() * ((CalculatedLimelight)camera).getTrust();
          fR += ((CalculatedLimelight)camera).getFieldPose().getRotation().getRadians() * ((CalculatedLimelight)camera).getTrust();
          tot += ((CalculatedLimelight)camera).getTrust();
        }
      }
      if (camera instanceof PhotonCamera) {
        var result = ((PhotonCamera)camera).getLatestResult();
        if(result.hasTargets()) {
          Pose3d robotPose3d = PhotonUtils.estimateFieldToRobotAprilTag(
            result.getBestTarget().bestCameraToTarget,
            AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(0).get(), //TODO Choose which target its looking at
            new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0))
          );
          fX += robotPose3d.getX();
          fY += robotPose3d.getY();
          fR += robotPose3d.getRotation().getZ();
          tot += 1; //TODO Calculate Trust
        }
      }
    }
    fX /= tot;
    fY /= tot;
    fR /= tot;
    return new Pose2d(fX,fY, new Rotation2d(fR));
  }
  
  @Override
  public void periodic() {
    var VisionResult = Vision.MicrosoftCamera.getLatestResult();
    SmartDashboard.putBoolean("Has Targets", VisionResult.hasTargets());

    // VisionResult.getMultiTagResult()
    Optional<EstimatedRobotPose> pose = microsoftPoseEstimator.update(VisionResult);
    SmartDashboard.putString("Pose: ", pose.toString());

    Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(VisionResult.getTargets().get(0).getBestCameraToTarget(), AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(0).get(), new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0)));
  }
}
