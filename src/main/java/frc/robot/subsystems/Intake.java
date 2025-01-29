// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.intakeConstants;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {

  double leftSpeed;
  double rightSpeed;
  double pivotSpeed;
  double targetAngle;
  double currentAngle;
  PIDController pivotPID = new PIDController(intakeConstants.IntakePIDkp, intakeConstants.IntakePIDki, intakeConstants.IntakePIDkd);

  /** Creates a new Intake. */
  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    RobotContainer.leftMotor.set(leftSpeed);
    RobotContainer.rightMotor.set(rightSpeed);

    pivotSpeed = pivotPID.calculate(targetAngle - currentAngle);

    RobotContainer.pivotMotor.set(pivotSpeed);
  }

  public void setIntakeSpeed(double leftSpeed, double rightSpeed){
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;
  }

  public void setPivotAngle(double targetAngle){
    if (targetAngle > intakeConstants.MaxAngle) targetAngle = intakeConstants.MaxAngle;
    if (targetAngle < intakeConstants.MinAngle) targetAngle = intakeConstants.MinAngle;

    this.targetAngle = targetAngle;
  }

}
