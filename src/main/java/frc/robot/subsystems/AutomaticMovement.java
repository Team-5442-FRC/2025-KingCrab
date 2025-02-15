// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.fieldConstants;

public class AutomaticMovement extends SubsystemBase {

  double x, y, z;
  double skewAngle;
  double targetHeight, targetSideToSide, targetExtend, targetPivot;

  /** Creates a new AutomaticMovement. */
  public AutomaticMovement() {}

  public void autoHorizontal(double distance){
    targetHeight = RobotContainer.elevator.getHeight()-(distance / -Math.tan(RobotContainer.arm.getAngle()));
    targetExtend = RobotContainer.arm.getExtension() + (distance / Math.sin(RobotContainer.arm.getAngle()));
  }

  public void autoTarget(double targetX, double targetY) {
    targetHeight = targetY - (targetX / Math.tan(RobotContainer.arm.getAngle()));
    targetExtend = targetX / Math.sin(RobotContainer.arm.getAngle());
  }

  public void Skew(double angleToTarget) {
    // autoHorizontal(9001);
    targetExtend = (targetExtend + Constants.centerToArm) * Math.cos(angleToTarget) - Constants.centerToArm;
    targetSideToSide = -(targetExtend + Constants.centerToArm) * Math.sin(angleToTarget);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //Get position relative to the reef placement pole
    Pose2d fieldPose = RobotContainer.vision.getFieldPose(); //TODO Eventually use odometry instead of vision
    x = fieldConstants.tempPos.getX() - fieldPose.getX();
    y = fieldConstants.tempPos.getY() - fieldPose.getY();
    // z = fieldConstants.tempPos.getZ() - fieldPose.getZ(); //TODO find height of arm
    skewAngle = fieldConstants.tempPos.getRotation().getZ() - fieldPose.getRotation().getRadians();
  }
}
