// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.ArrayList;

import org.ejml.data.Matrix;
import org.ejml.equation.MatrixConstructor;
import org.ejml.simple.SimpleMatrix;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.Telemetry;
import frc.robot.Constants.visionConstants;


public class Vision implements Runnable {
  
  ArrayList<CalculatedCamera> cameras = new ArrayList<CalculatedCamera>(); // the binder binder of binding binders like melvin sneedly invented at his 5th grade invention convention in the hit movie the first epic captain underpants movie

  
    public static CalculatedPhotonVision FrontRightM1Cam = new CalculatedPhotonVision("Front Right (M1)", visionConstants.FrontRightM1CamOffset);
    public static CalculatedPhotonVision FrontLeftM2Cam = new CalculatedPhotonVision("Front Left (M2)", visionConstants.FrontLeftM2CamOffset);
    // public final static CalculatedLimelight LimelightMain = new CalculatedLimelight("limelight-main");

    PhotonPoseEstimator microsoftPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0)));
    PhotonPoseEstimator thriftyPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0)));

    Telemetry logger = RobotContainer.logger;

  public Vision() {
    cameras.add(FrontRightM1Cam);
    // cameras.add(FrontLeftM2Cam);
  }

  public boolean hasTarget() {
    for (CalculatedCamera camera : cameras) {
      if (camera.hasTarget()) return true;
    }
    return false;
  }
  

  public Pose2d getFieldPose() {
    double fX = 0;
    double fY = 0;
    double fR = 0;
    double tot = 0;

    for (CalculatedCamera camera: cameras) {
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

  public Pose2d getTagRelativePose(Pose2d robotPose, int aprilTag) {
    Pose3d aprilPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(aprilTag).get();

    double x0 = aprilPose.getX() - robotPose.getX();
    double y0 = aprilPose.getY() - robotPose.getY();
    
    double distance = Math.sqrt((x0*x0)+(y0*y0));
    
    double angle0 = Math.atan2(y0,x0);
    double angle = angle0 - aprilPose.getRotation().getZ() + Math.PI;
    if (angle > Math.PI) angle -= (2*Math.PI);
    double x = distance * Math.cos(angle);
    double y = distance * Math.sin(angle);

    double botAngle = robotPose.getRotation().getRadians() - aprilPose.getRotation().getZ() + Math.PI;
    if (botAngle > Math.PI) botAngle -= (2*Math.PI);

    return new Pose2d(x,y,new Rotation2d(botAngle));
  }

  public Pose2d getTagRelativePose(int aprilTag) {
    return getTagRelativePose(getFieldPose(), aprilTag);
  }

  public double calculateError() {
    double error = 0;
    for (CalculatedCamera camera: cameras) {
      error += Math.abs(camera.getFieldPose().getX()-getFieldPose().getX());
      error += Math.abs(camera.getFieldPose().getY()-getFieldPose().getY());
      error += Math.abs(camera.getFieldPose().getRotation().getDegrees()-getFieldPose().getRotation().getDegrees());
    }
    return error;
  }
  
  @Override
  public void run() {
    // Update camera readings to be in sync with the robot
    for (CalculatedCamera camera: cameras) {
      camera.updateResult();
    }

    // Add camera readings to odometry if they exist
    if (hasTarget()) RobotContainer.drivetrain.addVisionMeasurement(
      getFieldPose(),
      Utils.fpgaToCurrentTime(Timer.getFPGATimestamp()) - 0.05
    );

    SmartDashboard.putNumber("FR Camera X", FrontRightM1Cam.getTargetPose().getX());
    SmartDashboard.putNumber("FR Camera Y", FrontRightM1Cam.getTargetPose().getY());
    SmartDashboard.putNumber("FR Camera R", FrontRightM1Cam.getTargetPose().getRotation().getDegrees());
    SmartDashboard.putNumber("FR Camera Trust", FrontRightM1Cam.getTrust());
    
    SmartDashboard.putNumber("FL Camera X", FrontLeftM2Cam.getTargetPose().getX());
    SmartDashboard.putNumber("FL Camera Y", FrontLeftM2Cam.getTargetPose().getY());
    SmartDashboard.putNumber("FL Camera R", FrontLeftM2Cam.getTargetPose().getRotation().getDegrees());
    SmartDashboard.putNumber("FL Camera Trust", FrontLeftM2Cam.getTrust());
    
    SmartDashboard.putNumber("Camera Field X" , getFieldPose().getX());
    SmartDashboard.putNumber("Camera Field Y" , getFieldPose().getY());
    SmartDashboard.putNumber("Camera Field R" , getFieldPose().getRotation().getDegrees());
    SmartDashboard.putNumber("Camera Error", calculateError());
    SmartDashboard.putBoolean("Has Target", hasTarget());

    SmartDashboard.putNumber("Odometry Field X", RobotContainer.drivetrain.getState().Pose.getX());
    SmartDashboard.putNumber("Odometry Field Y", RobotContainer.drivetrain.getState().Pose.getY());
    SmartDashboard.putNumber("Odometry Field R", RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees());

    SmartDashboard.putNumber("Tag 7 X", getTagRelativePose(RobotContainer.drivetrain.getState().Pose, 7).getX());
    SmartDashboard.putNumber("Tag 7 Y", getTagRelativePose(RobotContainer.drivetrain.getState().Pose, 7).getY());
    SmartDashboard.putNumber("Tag 7 R", getTagRelativePose(RobotContainer.drivetrain.getState().Pose, 7).getRotation().getDegrees());
  }
}