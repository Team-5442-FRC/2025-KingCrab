// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.armConstants;
import frc.robot.Constants.fieldConstants;

public class PositionManager extends SubsystemBase {

  Pose2d robotPose, fieldPose;
  boolean isReefRight; // true = right, false = left
  int reefLevel, reefSide;
  double targetPivot, /*targetExtend,*/ targetHeight, targetSideToSide;
  public double xSpeed, ySpeed, rSpeed;
  double yOffset;

  /** Creates a new PositionManager. */
  public PositionManager() {}

  public double mToIn(double measure) { //Converts meters to inches
    return measure * 39.3701;
  }

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

  public double reefLevelToHeight(int reefLevel) {
    if (reefLevel == 1) return fieldConstants.L1Height;
    if (reefLevel == 2) return fieldConstants.L2Height;
    if (reefLevel == 3) return fieldConstants.L3Height;
    if (reefLevel == 4) return fieldConstants.L4Height;
    if (reefLevel == 5) return fieldConstants.CoralStationHeight; // 5 is Coral Station
    return fieldConstants.L1Height;
  }

  public void setReefTarget(boolean isReefRight, int reefLevel, int reefSide) {
    this.isReefRight = isReefRight;
    this.reefLevel = reefLevel;
    this.reefSide = reefSide;
  }

  public void updatePositions(double pivot, /*double extend,*/ double height, double sideToSide, double wristAngle) {
    RobotContainer.arm.setTargetAngle(pivot);
    // RobotContainer.arm.setTargetExtend(extend);
    RobotContainer.elevator.setUpAndDownPos(height);
    RobotContainer.elevator.setSide2SidePos(sideToSide);
    RobotContainer.manipulator.setWristAngle(wristAngle);
  }

  public double calculateArmPivot(int reefLevel) {
    if (reefLevel == 1) return Math.toRadians(fieldConstants.L1Angle);
    else if (reefLevel == 2) return Math.toRadians(fieldConstants.L2Angle);
    else if (reefLevel == 3) return Math.toRadians(fieldConstants.L3Angle);
    else if (reefLevel == 4) return Math.toRadians(fieldConstants.L4Angle);
    else if (reefLevel == 5) return Math.toRadians(fieldConstants.CoralStationAngle); // 5 means Coral Station
    return Math.toRadians(fieldConstants.ErrorAngle);
  }

  public double calculateWristAngle(int reefLevel) {
    if (reefLevel >= 2 && reefLevel <= 4) return 0;
    else if (reefLevel == 1 || reefLevel == 5) return Math.toRadians(90);
    return 0;
  }

  // public double calculateArmPivot(double x) {
  //   if (mToIn(x) >= armConstants.PivotToCoral - 0.1) x = armConstants.PivotToCoral - 0.1;
  //   if (mToIn(x) < 0) x = 0;
  //   double angle = ((Math.PI/2) - Math.asin(mToIn(x) / armConstants.PivotToCoral)) + (Math.PI/2);
  //   if (Double.isNaN(angle)) return Math.PI/2;
  //   return angle;
  // }

  // public double calculateArmExtend(int reefLevel, double x) { // UNUSED
  //   return mToIn(x) / Math.sin(calculateArmPivot(reefLevel));
  // }

  public double calculateHeight(double angle, double z) {
    // return z - (mToIn(x) / Math.tan(calculateArmPivot(reefLevel) - (Math.PI/2)));
    // return z - (r * Math.sin(calculateArmPivot(reefLevel) - (Math.PI/2))) + 5;
    return z - (armConstants.PivotToCoral * -Math.cos(angle)) + armConstants.VerticalCoralOffset;
  }

  public double calculateSideToSide(double y, boolean isReefRight) {
    if (isReefRight) return fieldConstants.ReefSideToSide + mToIn(y);
    else return -fieldConstants.ReefSideToSide + mToIn(y);
  }

  @Override
  public void periodic() {
    if (RobotContainer.isAutomaticPositioningMode && RobotContainer.vision.hasTarget()) {
      // // robotPose = RobotContainer.vision.getTagRelativePose(reefSideToAprilTag(reefSide));
      // robotPose = RobotContainer.vision.FrontRightM1Cam.getTargetPose();

      // targetPivot = calculateArmPivot(robotPose.getX() - armConstants.CenterToPivot + fieldConstants.TagToL2and3XOffset);
      // // targetExtend = calculateArmExtend(reefLevel, robotPose.getX());
      // targetHeight = calculateHeight(targetPivot, reefLevelToHeight(reefLevel));
      // targetSideToSide = calculateSideToSide(robotPose.getY(), isReefRight);
      // targetSideToSide = 0; // TODO - DELETEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE

      // updatePositions(targetPivot, /*targetExtend,*/ targetHeight, targetSideToSide);

    }

    if (RobotContainer.isAutomaticDriveMode && RobotContainer.vision.hasTarget()) {
      fieldPose = RobotContainer.vision.getFieldPose();
      Pose3d aprilPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(reefSideToAprilTag(reefSide)).get();
      // Pose3d aprilPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(7).get();

      
      if (isReefRight) yOffset = fieldConstants.DriveRightY;
      else yOffset = fieldConstants.DriveLeftY;

      xSpeed = -RobotContainer.Deadzone(((aprilPose.getX() - fieldPose.getX()) - (fieldConstants.DriveL2andL3X * Math.cos(aprilPose.getRotation().getZ()) - yOffset * Math.sin(aprilPose.getRotation().getZ()))) * fieldConstants.DrivekP, fieldConstants.DriveMinAutoSpeed);
      ySpeed = -RobotContainer.Deadzone(((aprilPose.getY() - fieldPose.getY()) - (yOffset * Math.cos(aprilPose.getRotation().getZ()) + fieldConstants.DriveL2andL3X * Math.sin(aprilPose.getRotation().getZ()))) * fieldConstants.DrivekP, fieldConstants.DriveMinAutoSpeed);
      rSpeed = RobotContainer.Deadzone((aprilPose.getRotation().getZ() - fieldPose.getRotation().getRadians()) * fieldConstants.DrivekP * 5, fieldConstants.DriveMinAutoSpeed);
    }
    else {
      xSpeed = 0;
      ySpeed = 0;
      rSpeed = 0;
    }

    SmartDashboard.putNumber("Target Pivot", targetPivot);
    SmartDashboard.putNumber("Target Height", targetHeight);
    SmartDashboard.putNumber("Target SideToSide", targetSideToSide);

    SmartDashboard.putNumber("Level", ButtonBox.lookup(ButtonBox.readBox())[0]);
    SmartDashboard.putNumber("Branch", ButtonBox.lookup(ButtonBox.readBox())[1]);
    SmartDashboard.putNumber("Side", reefSide);
    SmartDashboard.putNumber("Tag Target", reefSideToAprilTag(reefSide));
    SmartDashboard.putNumber("Button Box Reading", ButtonBox.readBox());
    SmartDashboard.putNumber("REEF TO HEIGHT", reefLevelToHeight(reefLevel));

    SmartDashboard.putNumber("Auto Drive X", xSpeed);
    SmartDashboard.putNumber("Auto Drive Y", ySpeed);
    SmartDashboard.putNumber("Auto Drive R", rSpeed);
  }
}