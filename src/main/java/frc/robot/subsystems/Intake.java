// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.intakeConstants;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {

  boolean isRelativeOffsetSet = false;
  double relativeOffset = 0;

  double leftSpeed;
  double rightSpeed;
  double pivotSpeed;
  double targetAngle = intakeConstants.PivotAngleAlgae; // TODO Update position to inside frame!
  PIDController pivotPID = new PIDController(intakeConstants.IntakePIDkp, intakeConstants.IntakePIDki, intakeConstants.IntakePIDkd);

  /** Creates a new Intake. */
  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (!isRelativeOffsetSet) {
      relativeOffset = (RobotContainer.intakePivotMotor.getEncoder().getPosition() * intakeConstants.PivotGearRatio) -(RobotContainer.intakeEncoder.get() - intakeConstants.PivotEncoderOffset) - 0.25;
      isRelativeOffsetSet = true;
    }
    
    RobotContainer.leftMotor.set(leftSpeed);
    RobotContainer.rightMotor.set(rightSpeed);

    pivotSpeed = pivotPID.calculate(getPivotAngle() - targetAngle); // Calculates the pivot speed by subtracting the target angle from the current angle
    if (pivotSpeed > 0 && getPivotAngle() > intakeConstants.PivotMaxAngle) pivotSpeed = 0; // If beyond upper limit, cancel movement
    if (pivotSpeed < 0 && getPivotAngle() < intakeConstants.PivotMinAngle) pivotSpeed = 0; // If beyond lower limit, cancel movement

    RobotContainer.intakePivotMotor.set(pivotSpeed); // Sets the motor to go the speed calculated

    SmartDashboard.putNumber("Intake Pivot Encoder", RobotContainer.intakeEncoder.get() - intakeConstants.PivotEncoderOffset);
    SmartDashboard.putNumber("Intake Pivot Radians", getPivotAngle());
    SmartDashboard.putNumber("Intake Pivot Degrees", Math.toDegrees(getPivotAngle()));
    SmartDashboard.putNumber("Left Speed", leftSpeed);
    SmartDashboard.putNumber("Right Speed", rightSpeed);
    SmartDashboard.putNumber("Intake Pivot Speed", pivotSpeed);
    SmartDashboard.putNumber("Intake Motor Encoder", RobotContainer.intakePivotMotor.getEncoder().getPosition() * intakeConstants.PivotGearRatio);
  }

  public void setIntakeSpeed(double leftSpeed, double rightSpeed){
    this.leftSpeed = leftSpeed; // Sets the class variable to the local variable.
    this.rightSpeed = rightSpeed;
  }

  public void setPivotAngle(double targetAngle){
    if (targetAngle > intakeConstants.MaxAngle) targetAngle = intakeConstants.MaxAngle; // If the target angle is greater than the maximum angle, the target angle is set to the max angle
    if (targetAngle < intakeConstants.MinAngle) targetAngle = intakeConstants.MinAngle; // If the target angle is less than the minimum angle, the target angle is set to the min angle

    this.targetAngle = targetAngle; // Sets the class variable to the local variable
  }

  /** Returns the pivot angle of the intake in radians. */
  public double getPivotAngle() {
    // return (RobotContainer.intakeEncoder.get() - intakeConstants.PivotEncoderOffset + 0.25) * (2 * Math.PI);
    return ((RobotContainer.intakePivotMotor.getEncoder().getPosition() * intakeConstants.PivotGearRatio) - relativeOffset) * (2 * Math.PI);
  }
}
