// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.driveConstants;
import frc.robot.Constants.manipulatorConstants;
import frc.robot.RobotContainer;

public class Manipulator extends SubsystemBase {
  boolean horizontalButton = false;//Button to set manipulator angle to horizontal
  boolean verticalButton = false;//Button to set manipulator angle to vertical
  double manipulatorSpeed;
  double wristSpeed;
  double targetWristAngle;

  boolean wristManualMode = true;

  PIDController wristPID = new PIDController(manipulatorConstants.WristPIDkp, manipulatorConstants.WristPIDki,manipulatorConstants.WristPIDkd);
  SlewRateLimiter wristLimiter = new SlewRateLimiter(8);
  
  /** Creates a new Manipulator. */
  public Manipulator() {}
  
  public double getRotation() {
    return (RobotContainer.wristMotor.getEncoder().getPosition() / 49) * 2 * Math.PI; // Radians
  }

  public void setWristAngle(double angle) {
    if (angle > manipulatorConstants.WristRightLimit) angle = manipulatorConstants.WristRightLimit;
    if (angle < manipulatorConstants.WristLeftLimit) angle = manipulatorConstants.WristLeftLimit;
    targetWristAngle = angle;
  }

  public void setManipulatorSpeed(double speed) {
    manipulatorSpeed = speed;
  }

  public void setWristSpeed(double speed) {
    if (Math.abs(RobotContainer.Deadzone(speed, driveConstants.Controller2Deadzone))>0) wristManualMode = true;
    else if (wristManualMode && Math.abs(RobotContainer.Deadzone(speed, driveConstants.Controller2Deadzone)) == 0) {
      targetWristAngle = getRotation();
      wristManualMode = false;
    }

    if(wristManualMode) {
      wristSpeed = RobotContainer.Deadzone(speed, driveConstants.Controller2Deadzone) * manipulatorConstants.WristSpeedFactor;
      //if it's past the right limit and still moving right, stop moving and move back left
      if ((wristSpeed>0) && (getRotation()>manipulatorConstants.WristRightLimit)) {
        wristSpeed = 0;
        targetWristAngle = manipulatorConstants.WristRightLimit;
      }
      //if it's past the left limit and still moving left, stop moving and move back right
      if ((wristSpeed<0) && (getRotation()<manipulatorConstants.WristLeftLimit)) {
        wristSpeed = 0;
        targetWristAngle = manipulatorConstants.WristLeftLimit;
      }
    }

    if (!wristManualMode) {
      wristSpeed = -wristLimiter.calculate(wristPID.calculate(targetWristAngle - getRotation()));
      if (wristSpeed > 0 && getRotation() > manipulatorConstants.WristRightLimit) wristSpeed = 0;
      if (wristSpeed < 0 && getRotation() < manipulatorConstants.WristLeftLimit) wristSpeed = 0;
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // wristSpeed = wristPID.calculate(targetWristAngle - getRotation());
    
    RobotContainer.wristMotor.set(wristSpeed);
    RobotContainer.manipulatorIntakeMotor.set(-manipulatorSpeed);

    SmartDashboard.putNumber("Wrist Encoder", RobotContainer.wristMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Wrist Degrees", Math.toDegrees(getRotation()));
    SmartDashboard.putNumber("Wrist Speed", wristSpeed);
    SmartDashboard.putBoolean("Proximity Sensor", RobotContainer.manipulatorProxSensor.get());
    
    SmartDashboard.putNumber("Intake State", RobotContainer.manipulatorCommand.state);
  }
}
