// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.fieldConstants;

public class PositionManager extends SubsystemBase {

  Pose2d robotPose;
  boolean isReefRight; // true = right, false = left
  int reefLevel, reefSide;
  double targetPivot, targetExtend, targetHeight, targetSideToSide;

  /** Creates a new PositionManager. */
  public PositionManager() {}

  public int reefSideToAprilTag(int reefSide) {
    if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
      if (reefSide == 1) return 7;
      if (reefSide == 2) return 8;
      if (reefSide == 3) return 9;
      if (reefSide == 4) return 10;
      if (reefSide == 5) return 11;
      if (reefSide == 6) return 6;
    }
    else {
      if (reefSide == 1) return 18;
      if (reefSide == 2) return 17;
      if (reefSide == 3) return 22;
      if (reefSide == 4) return 21;
      if (reefSide == 5) return 20;
      if (reefSide == 6) return 19;
    }
    return 1;
  }

  public void setReefTarget(boolean isReefRight, int reefLevel, int reefSide) {
    this.isReefRight = isReefRight;
    this.reefLevel = reefLevel;
    this.reefSide = reefSide;
  }

  public void updatePositions(double pivot, double extend, double height, double sideToSide) {
    RobotContainer.arm.setTargetAngle(pivot);
    RobotContainer.arm.setTargetExtend(extend);
    RobotContainer.elevator.setUpAndDownPos(height);
    RobotContainer.elevator.setSide2SidePos(sideToSide);
  }

  public double calculateArmPivot(int reefLevel) {
    if (reefLevel == 1) return Math.toRadians(fieldConstants.L1Angle);
    else if (reefLevel == 2) return Math.toRadians(fieldConstants.L2Angle);
    else if (reefLevel == 3) return Math.toRadians(fieldConstants.L3Angle);
    else if (reefLevel == 4) return Math.toRadians(fieldConstants.L4Angle);
    return Math.toRadians(fieldConstants.ErrorAngle);
  }

  public double calculateArmExtend(int reefLevel, double x) {
    return x / Math.sin(calculateArmPivot(reefLevel));
  }

  public double calculateHeight(int reefLevel, double x, double y) {
    return y - (x - Math.tan(calculateArmPivot(reefLevel)));
  }

  public double calculateSideToSide(double y, boolean isReefRight) {
    if (isReefRight) return -y + fieldConstants.ReefSideToSide;
    else return -y - fieldConstants.ReefSideToSide;
  }

  @Override
  public void periodic() {
    if (RobotContainer.isAutomaticPositioningMode) {
      robotPose = RobotContainer.vision.getTagRelativePose(reefSideToAprilTag(reefSide));

      targetPivot = calculateArmPivot(reefLevel);
      targetExtend = calculateArmExtend(reefLevel, robotPose.getX());
      targetHeight = calculateHeight(reefLevel, robotPose.getX(), robotPose.getY());
      targetSideToSide = calculateSideToSide(robotPose.getY(), isReefRight);

      updatePositions(targetPivot, targetExtend, targetHeight, targetSideToSide);
    }
  }
}
