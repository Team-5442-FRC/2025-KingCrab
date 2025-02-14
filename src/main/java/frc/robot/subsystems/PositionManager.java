// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.posManagerConstants;

public class PositionManager extends SubsystemBase {

  Pose2d robotPose;
  boolean isReefRight; // true = right, false = left
  int reefLevel;
  double targetPivot, targetExtend, targetHeight, targetSideToSide;

  /** Creates a new PositionManager. */
  public PositionManager() {}

  public void setReefTarget(boolean isReefRight, int reefLevel) {
    this.isReefRight = isReefRight;
    this.reefLevel = reefLevel;
  }

  public void updatePositions(double pivot, double extend, double height, double sideToSide) {
    RobotContainer.arm.setTargetAngle(pivot);
    RobotContainer.arm.setTargetExtend(extend);
    RobotContainer.elevator.setUpAndDownPos(height);
    RobotContainer.elevator.setTargetPos(sideToSide);
  }

  public double calculateArmPivot(int reefLevel) {
    if (reefLevel == 1) return Math.toRadians(posManagerConstants.L1Angle);
    else if (reefLevel == 2) return Math.toRadians(posManagerConstants.L2Angle);
    else if (reefLevel == 3) return Math.toRadians(posManagerConstants.L3Angle);
    else if (reefLevel == 4) return Math.toRadians(posManagerConstants.L4Angle);
    return Math.toRadians(posManagerConstants.ErrorAngle);
  }

  public double calculateArmExtend(int reefLevel, double x) {
    return x / Math.sin(calculateArmPivot(reefLevel));
  }

  public double calculateHeight(int reefLevel, double x, double y) {
    return y - (x - Math.tan(calculateArmPivot(reefLevel)));
  }

  public double calculateSideToSide(double y, boolean isReefRight) {
    if (isReefRight) {
      return -y + posManagerConstants.ReefSideToSide;
    }
    else {
      return -y - posManagerConstants.ReefSideToSide;
    }
  }

  @Override
  public void periodic() {
    if (RobotContainer.isAutomaticPositioningMode) {
      robotPose = RobotContainer.vision.getFieldPose();

      targetPivot = calculateArmPivot(reefLevel);
      targetExtend = calculateArmExtend(reefLevel, robotPose.getX());
      targetHeight = calculateHeight(reefLevel, robotPose.getX(), robotPose.getY());
      targetSideToSide = calculateSideToSide(robotPose.getY(), isReefRight);
    }
  }
}
