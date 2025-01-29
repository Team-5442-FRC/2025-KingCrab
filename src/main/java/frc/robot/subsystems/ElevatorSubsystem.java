// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.elevatorConstants;
import frc.robot.subsystems.DriveModes;

public class ElevatorSubsystem extends SubsystemBase {
  
boolean side2SideManualMode;
boolean upAndDownManualMode;
double side2SideTargtePos;
double upAndDownTargetPos;
double side2SideCurrentPos;
double upAndDownCurrentPos;
double side2SideSpeed;
double upAndDownSpeed;
PIDController upAndDownPID = new PIDController(.00001, 0, 0);
PIDController side2sidePID = new PIDController(.00001, 0, 0);

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {}

  @Override
  public void periodic() {
    if (Math.abs(RobotContainer.xbox1.getLeftY())>0) {side2SideManualMode = true;}//TODO Add deadzone thing
    else if (side2SideManualMode && Math.abs(RobotContainer.xbox1.getLeftY())>0) {side2SideTargtePos = side2SideCurrentPos; side2SideManualMode = false;} //TODO Add deadzone thing



    if(side2SideManualMode) {
      if ((side2SideSpeed>0) && (side2SideCurrentPos>=elevatorConstants.armRightLimit)) {
        side2SideSpeed = 0;
        side2SideTargtePos = elevatorConstants.armRightLimit;
      }
      if ((side2SideSpeed<0) && (side2SideCurrentPos<=elevatorConstants.armLeftLimit)) {
        side2SideSpeed = 0;
        side2SideTargtePos = elevatorConstants.armLeftLimit;
      }
    }


    if (Math.abs(RobotContainer.xbox1.getLeftX()) > 0) {upAndDownManualMode = true;} //TODO Add deadzone thing
    else if (upAndDownManualMode && Math.abs(RobotContainer.xbox1.getLeftX()) > 0) {  //TODO Add deadzone thing
      upAndDownTargetPos = upAndDownCurrentPos;
       upAndDownManualMode = false; 
      }


    if(upAndDownManualMode) {
      if ((upAndDownSpeed>0) && (upAndDownCurrentPos>=elevatorConstants.armTopLimit)) {
        upAndDownSpeed = 0;
        upAndDownTargetPos = elevatorConstants.armTopLimit;
      }
      if ((upAndDownSpeed<0) && (upAndDownCurrentPos<=elevatorConstants.armBottomLimit)) {
        upAndDownSpeed = 0;
        upAndDownTargetPos = elevatorConstants.armBottomLimit;
      }
    }


    if (!upAndDownManualMode) {
      upAndDownPID.calculate(upAndDownTargetPos - upAndDownCurrentPos);
    }
    if (!side2SideManualMode) {
      side2sidePID.calculate(side2SideTargtePos - side2SideCurrentPos);
    }
    
  }
  public void setTargetPos(double side2SideTargtePos) {
    if (side2SideTargtePos > elevatorConstants.armTopLimit) {
      side2SideTargtePos = elevatorConstants.armTopLimit;
    }
    if (upAndDownTargetPos < elevatorConstants.armBottomLimit) {
      upAndDownTargetPos = elevatorConstants.armBottomLimit;
    }
  }

  public double getHeight() {
    return 0; //TODO Find formula to calculate
  }

  public double setSideToSide() {
    return 0; //TODO Find formula to calculate
  }
}
