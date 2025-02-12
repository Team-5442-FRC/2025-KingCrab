// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.intakeConstants;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {

  double leftSpeed; // Initizlizes the variables
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
    // currentAngle = RobotContainer.pivotMotor.getAlternateEncoder().getPosition() / intakeConstants.PivotGearRatio; // Sets the current angle to the position of the motor divided by the gear ratio
    
    RobotContainer.leftMotor.set(leftSpeed);
    RobotContainer.rightMotor.set(rightSpeed);

    // pivotSpeed = pivotPID.calculate(targetAngle - currentAngle); // Calculates the pivot speed by subtracting the target angle from the current angle

    // RobotContainer.pivotMotor.set(pivotSpeed); // Sets the motor to go the speed calculated
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

}
