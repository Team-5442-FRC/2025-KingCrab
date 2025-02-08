// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutomaticMovement extends SubsystemBase {
  /** Creates a new AutomaticMovement. */
  public AutomaticMovement() {}

  public void autoHorizontal(double distance){
    RobotContainer.elevator.setUpAndDownPos(RobotContainer.elevator.getHeight()-(distance / -Math.tan(RobotContainer.arm.getAngle())));
    RobotContainer.arm.setTargetExtend(RobotContainer.arm.getExtension() + (distance / Math.sin(RobotContainer.arm.getAngle())));
  }

  public void autoTarget(double targetX, double targetY) {
    RobotContainer.elevator.setUpAndDownPos(targetY - (targetX / Math.tan(RobotContainer.arm.getAngle())));
    RobotContainer.arm.setTargetExtend(targetX / Math.sin(RobotContainer.arm.getAngle()));
  }

  public void Skew(double angleToTarget) {
    RobotContainer.arm.setTargetExtend((RobotContainer.arm.getExtension() + Constants.centerToArm) * Math.cos(angleToTarget) - Constants.centerToArm);
    RobotContainer.elevator.setSide2SidePos(-(RobotContainer.arm.getExtension() + Constants.centerToArm) * Math.sin(angleToTarget));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
