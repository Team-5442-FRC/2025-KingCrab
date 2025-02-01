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
import frc.robot.Constants.visionConstants;


public class Vision extends SubsystemBase {
  
  ArrayList<CalculatedCamera> cameras = new ArrayList<CalculatedCamera>(); // the binder binder of binding binders like melvin sneedly invented at his 5th grade invention convention in the hit movie the first epic captain underpants movie

  
    public final static CalculatedPhotonVision MicrosoftCamera = new CalculatedPhotonVision("Microsoft_Camera",visionConstants.MicrosoftCameraOffset);
    public final static CalculatedPhotonVision ThriftyCamera = new CalculatedPhotonVision("Thrifty_Camera",visionConstants.ThriftyCameraOffset);
    public final static CalculatedPhotonVision GenoCamera = new CalculatedPhotonVision("Geno_Camera", visionConstants.GenoCameraOffset); // geno camera
    public final static CalculatedLimelight LimelightMain = new CalculatedLimelight("limelight-main");

    PhotonPoseEstimator microsoftPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0)));
    PhotonPoseEstimator thriftyPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0)));

    Telemetry logger = RobotContainer.logger;

  public Vision() {
    cameras.add(MicrosoftCamera);
    cameras.add(ThriftyCamera);
    cameras.add(GenoCamera);
  }
  

  public Pose2d getFieldPose() {
    double fX = 0;
    double fY = 0;
    double fR = 0;
    double tot = 0;

    for (CalculatedCamera camera: cameras) {
      camera.updateResult();
      if (camera.hasTarget()) {
        fX += camera.getFieldPose().getX() * camera.getTrust();
        fY += camera.getFieldPose().getY() * camera.getTrust();
        fR += camera.getFieldPose().getRotation().getRadians() * camera.getTrust();
        tot += camera.getTrust();
      }
    }
    fX /= tot;
    fY /= tot;
    fR /= tot;
    return new Pose2d(fX,fY, new Rotation2d(fR));
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Microsoft_Camera HasTarget", MicrosoftCamera.hasTarget());
    SmartDashboard.putBoolean("Thrifty_Camera HasTarget", ThriftyCamera.hasTarget());
    SmartDashboard.putBoolean("Geno_Camera HasTarget", GenoCamera.hasTarget());

    SmartDashboard.putNumber("camera position x" , getFieldPose().getX());
    SmartDashboard.putNumber("camera position y" , getFieldPose().getY());
    SmartDashboard.putNumber("camera position r" , getFieldPose().getRotation().getDegrees());

    SmartDashboard.putNumber("Microsoft_Camera Trust", MicrosoftCamera.getTrust());
    SmartDashboard.putNumber("Thrifty_Camera Trust", ThriftyCamera.getTrust());
    SmartDashboard.putNumber("Geno_Camera Trust", GenoCamera.getTrust());
    SmartDashboard.putNumber("limelight-main Trust", LimelightMain.getTrust());
  }
}
