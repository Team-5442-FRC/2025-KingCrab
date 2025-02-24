package frc.robot.subsystems.Vision;

import javax.naming.spi.DirStateFactory.Result;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.visionConstants;

/** It's like the instructions for making an apple pie. */
public class CalculatedPhotonVision extends CalculatedCamera {

  String camera;
  PhotonCamera cam;
  Transform3d camOffset;
  PhotonPipelineResult result;


  public CalculatedPhotonVision(String camera, Transform3d camOffset) {
    super(camera);
    this.camera = camera;   
    this.camOffset = camOffset;
    cam = new PhotonCamera(camera);
    updateResult();
  }

  @Override
  public void updateResult() {
    result = getPhotonCamera().getLatestResult();
  }

  public PhotonCamera getPhotonCamera() {
    return cam;
  }

  
  public PhotonPipelineResult getResult() {
    return result;
  }
    
  @Override
  public boolean hasTarget() {
    return !(getResult().getBestTarget() == null);
  }

  @Override
  public long getTargetID() {
    if (hasTarget()) return getResult().getBestTarget().fiducialId;
    return 0;
  }


  @Override
  public Pose2d getFieldPose() {
    if (hasTarget()) {
      return PhotonUtils.estimateFieldToRobotAprilTag(
        getResult().getBestTarget().getBestCameraToTarget(),
        AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose((int)getTargetID()).get(),
        camOffset
      ).toPose2d();
    }
    return new Pose2d();
  }

  @Override
  public double getTrust() { //TODO might be able to take area the tag takes up and use that instead
    Pose2d targetRelative = getTargetPose();
    if (hasTarget()) return (1 / Math.sqrt((targetRelative.getX() * targetRelative.getX()) + (targetRelative.getY() * targetRelative.getY())) + (visionConstants.AngleDistrust * Math.cos(targetRelative.getRotation().getRadians())));
    return 0; 
  }


  @Override
  public Pose2d getTargetPose() {
    if (hasTarget()) {
      // X and Y
      double x = getResult().getBestTarget().getBestCameraToTarget().getX();
      double y = getResult().getBestTarget().getBestCameraToTarget().getY();
      
      // Rotation
      double rotation = -getResult().getBestTarget().getBestCameraToTarget().getRotation().getZ(); // Radians
      if (rotation >= 0) rotation -= Math.PI; // Radians
      else if (rotation < 0) rotation += Math.PI; // Radians

      // X and Y transform(ers robots in disguise)
      double x1 = (x*Math.cos(camOffset.getRotation().getZ())) - (y*Math.sin(camOffset.getRotation().getZ()));
      double y1 = (y*Math.cos(camOffset.getRotation().getZ())) + (x*Math.sin(camOffset.getRotation().getZ()));

      // Return
      return new Pose2d(
        x1 + camOffset.getX(),
        y1 + camOffset.getY(),
        new Rotation2d(rotation - camOffset.getRotation().getZ())); // Rotation is yaw, radians
    }
    return new Pose2d(0, 0, new Rotation2d(0));
  }
}
