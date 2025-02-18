// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.armConstants;
import frc.robot.Constants.manipulatorConstants;

public class Manipulator extends SubsystemBase {
  boolean horizontalButton = false;//Button to set manipulator angle to horizontal
  boolean verticalButton = false;//Button to set manipulator angle to vertical
  double manipulatorSpeed;
  double wristSpeed;
  double targetWristAngle;
  PIDController wristPID = new PIDController(manipulatorConstants.WristPIDkd, manipulatorConstants.WristPIDki,manipulatorConstants.WristPIDkp );
  
  /** Creates a new Manipulator. */
  public Manipulator() {}
  
  public double getRotation() {
    return 0; //TODO Find how to get the rotation angle
  }

  public void rotateWrist() {
    if (horizontalButton) { //Horizontal
      targetWristAngle = 0 - manipulatorConstants.WristAngleOffset;
    }
    else if (verticalButton) { //Vertical
      targetWristAngle = 90 - manipulatorConstants.WristAngleOffset;
    }
    
    //Refer to the PID to set the speed
    wristSpeed = wristPID.calculate(targetWristAngle - getRotation());
  }

  public void setManipulatorSpeed(double speed) {
    manipulatorSpeed = speed;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    RobotContainer.manipulatorIntakeMotor.set(manipulatorSpeed);
    RobotContainer.wristMotor.set(wristSpeed * manipulatorConstants.WristSpeedFactor);
  }
}
