// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.armConstants;
import frc.robot.Constants.elevatorConstants;
import frc.robot.Constants.fieldConstants;

public class PositionManager extends SubsystemBase {

  Pose2d robotPose, fieldPose;
  boolean isReefRight; // true = right, false = left
  boolean isAlgae;
  int reefLevel, reefSide;
  double targetPivot, /*targetExtend,*/ targetHeight, targetSideToSide;
  public double xSpeed, ySpeed, rSpeed;
  public boolean driveLeftCoralStation = false;
  public boolean driveRightCoralStation = false;
  double xOffset, yOffset, rOffset;

  Command autoDriveToTag = new Command() {};

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
    if (isAlgae) {
      if (reefLevel == 2) return fieldConstants.AlgaeL2Height;
      if (reefLevel == 3) return fieldConstants.AlgaeL3Height;
      if (reefLevel == 4) return fieldConstants.BargeShootHeight;
    }
    if (reefLevel == 1) return fieldConstants.L1Height;
    if (reefLevel == 2) return fieldConstants.L2Height;
    if (reefLevel == 3) return fieldConstants.L3Height;
    if (reefLevel == 4) return fieldConstants.L4Height;
    if (reefLevel == 5) return fieldConstants.CoralStationHeight; // 5 is Coral Station
    if (reefLevel == 6) return fieldConstants.FloorPickupHeight; // 6 is Floor Pickup
    return fieldConstants.L2Height;
  }

  public void setReefTarget(boolean isReefRight, int reefLevel, int reefSide, boolean isAlgae) {
    this.isReefRight = isReefRight;
    this.reefLevel = reefLevel;
    this.reefSide = reefSide;
    this.isAlgae = isAlgae;
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
    if (reefLevel == 2 && !isAlgae) return Math.toRadians(fieldConstants.L2Angle);
    if (reefLevel == 3 && !isAlgae) return Math.toRadians(fieldConstants.L3Angle);
    if (isAlgae && (reefLevel == 2 || reefLevel == 3)) return Math.toRadians(fieldConstants.AlgaeAngle);
    if (reefLevel == 4 && !isAlgae) return Math.toRadians(fieldConstants.L4Angle);
    if (isAlgae && reefLevel == 4) return Math.toRadians(fieldConstants.BargeShootAngle);
    if (reefLevel == 5) return Math.toRadians(fieldConstants.CoralStationAngle); // 5 means Coral Station
    if (reefLevel == 6) return Math.toRadians(fieldConstants.FloorPickupAngle); // 6 means Floor Pickup
    
    return Math.toRadians(fieldConstants.ErrorAngle);
  }

  public double calculateWristAngle(int reefLevel) {
    if (isAlgae && (reefLevel >= 2 && reefLevel <= 4)) return Math.toRadians(90);
    if (!isAlgae && (reefLevel >= 2 && reefLevel <= 4)) return 0;
    if (reefLevel == 1 || reefLevel == 5 || reefLevel == 6) return Math.toRadians(90);
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
    if (isAlgae) return 0;
    // if (isReefRight) return fieldConstants.ReefSideToSide + mToIn(y);
    if (isReefRight) return elevatorConstants.ArmRightLimit;
    // else return -fieldConstants.ReefSideToSide + mToIn(y);
    else return elevatorConstants.ArmLeftLimit;
  }

  public Command generatePathToTag(int aprilTag) {
    Pose2d tagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(aprilTag).get().toPose2d();
    double x = tagPose.getX() + (fieldConstants.PathfindOffset * Math.cos(tagPose.getRotation().getRadians()));
    double y = tagPose.getY() + (fieldConstants.PathfindOffset * Math.sin(tagPose.getRotation().getRadians()));
    Pose2d targetPose = new Pose2d(x, y, tagPose.getRotation().plus(new Rotation2d(Math.PI)));

    return AutoBuilder.pathfindToPose(
      targetPose,
      new PathConstraints(0.5, 1, 180, 180),
      0
    );
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

      // updatePositions(targetPivot, /*targetExtend,*/ targetHeight, targetSideToSide);

    }

    if (!driveLeftCoralStation && !driveRightCoralStation) {
      robotPose = RobotContainer.vision.getTagRelativePose(RobotContainer.drivetrain.getState().Pose, reefSideToAprilTag(reefSide));

      if (isAlgae) {
        xOffset = robotPose.getX() - fieldConstants.DriveAlgaeX;
        yOffset = 0;
      }
      else {
        xOffset = robotPose.getX() - fieldConstants.DriveL2andL3X;
        if (isReefRight) yOffset = robotPose.getY() + fieldConstants.DriveRightY;
        else yOffset = robotPose.getY() + fieldConstants.DriveLeftY;
      }
      rOffset = -robotPose.getRotation().getRadians();
    }
    else if (driveLeftCoralStation) {
      if (DriverStation.getAlliance().get().equals(Alliance.Red)) robotPose = RobotContainer.vision.getTagRelativePose(RobotContainer.drivetrain.getState().Pose, 1);
      else RobotContainer.vision.getTagRelativePose(RobotContainer.drivetrain.getState().Pose, 13);

      xOffset = robotPose.getX() - fieldConstants.DriveCoralX;
      yOffset = 0;
      rOffset = -robotPose.getRotation().getRadians();
    }
    else if (driveRightCoralStation) {
      if (DriverStation.getAlliance().get().equals(Alliance.Red)) robotPose = RobotContainer.vision.getTagRelativePose(RobotContainer.drivetrain.getState().Pose, 2);
      else RobotContainer.vision.getTagRelativePose(RobotContainer.drivetrain.getState().Pose, 12);

      xOffset = robotPose.getX() - fieldConstants.DriveCoralX;
      yOffset = 0;
      rOffset = -robotPose.getRotation().getRadians();
    }

    double xSpeedTemp = RobotContainer.Deadzone(((xOffset * Math.cos(rOffset)) - (yOffset * Math.sin(rOffset))) * fieldConstants.DrivekP, fieldConstants.DriveMinAutoSpeed);
    double ySpeedTemp = RobotContainer.Deadzone(((yOffset * Math.cos(rOffset)) + (xOffset * Math.sin(rOffset))) * fieldConstants.DrivekP, fieldConstants.DriveMinAutoSpeed);
    xSpeed = RobotContainer.Cosine(xSpeedTemp, ySpeedTemp, 0.5);
    ySpeed = RobotContainer.Sine(xSpeedTemp, ySpeedTemp, 0.5);
    double rSpeedTemp = RobotContainer.Deadzone(rOffset * fieldConstants.RotatekP, fieldConstants.DriveMinAutoSpeed);
    if (rSpeedTemp >= 0) rSpeed = Math.pow(rSpeedTemp, 0.5);
    else rSpeed = -Math.pow(-rSpeedTemp, 0.5);

    
    if (RobotContainer.xbox1.getYButtonPressed() && !autoDriveToTag.isScheduled()) {
      autoDriveToTag = generatePathToTag(reefSideToAprilTag(reefSide));
      autoDriveToTag.schedule();
    }
    else if (!RobotContainer.xbox1.getYButton()) {
      autoDriveToTag.cancel();
    }
    SmartDashboard.putBoolean("Auto Drive Command Scheduled?", autoDriveToTag.isScheduled());


    SmartDashboard.putNumber("Target Pivot", targetPivot);
    SmartDashboard.putNumber("Target Height", targetHeight);
    SmartDashboard.putNumber("Target SideToSide", targetSideToSide);

    SmartDashboard.putNumber("Level", ButtonBox.lookup(ButtonBox.readBox())[0]);
    SmartDashboard.putNumber("Branch", ButtonBox.lookup(ButtonBox.readBox())[1]);
    SmartDashboard.putNumber("Side", reefSide);
    SmartDashboard.putNumber("Tag Target", reefSideToAprilTag(reefSide));
    SmartDashboard.putNumber("Button Box Reading", ButtonBox.readBox());

    SmartDashboard.putNumber("Auto Drive X", xSpeed);
    SmartDashboard.putNumber("Auto Drive Y", ySpeed);
    SmartDashboard.putNumber("Auto Drive R", rSpeed);
  }
}