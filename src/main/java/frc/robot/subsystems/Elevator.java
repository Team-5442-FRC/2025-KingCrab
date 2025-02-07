// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.elevatorConstants;
import frc.robot.subsystems.DriveModes;

public class Elevator extends SubsystemBase {
  
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
  public Elevator() {}

  @Override
  public void periodic() {
    RobotContainer.upAndDownMotor.set(upAndDownSpeed);
    RobotContainer.side2SideMotor.set(side2SideSpeed);
  }

  public void setTargetPos(double side2SideTargtePos) {
    //if it wants to move past the limit, this won't let it
    if (side2SideTargtePos > elevatorConstants.armTopLimit) {
      side2SideTargtePos = elevatorConstants.armTopLimit;
    }
    //if it wants to move past the limit, this won't let it
    if (upAndDownTargetPos < elevatorConstants.armBottomLimit) {
      upAndDownTargetPos = elevatorConstants.armBottomLimit;
    }
  }

  public void moveSide2Side(double speed) {
    //Calculates user input to determine if user is controlling it
    if (Math.abs(RobotContainer.Deadzone(speed))>0) side2SideManualMode = true;
    else if (side2SideManualMode && Math.abs(RobotContainer.Deadzone(speed))>0) {
      side2SideTargtePos = side2SideCurrentPos; 
      side2SideManualMode = false;
    }

    //Limits the movement of side to side motion from user movement to inside the limits
    if(side2SideManualMode) {
      side2SideSpeed = RobotContainer.Deadzone(speed);
      //if it's past the right limit and still moving right, stop moving and move back left
      if ((side2SideSpeed>0) && (side2SideCurrentPos>=elevatorConstants.armRightLimit)) {
        side2SideSpeed = 0;
        side2SideTargtePos = elevatorConstants.armRightLimit;
      }
      //if it's past the left limit and still moving left, stop moving and move back right
      if ((side2SideSpeed<0) && (side2SideCurrentPos<=elevatorConstants.armLeftLimit)) {
        side2SideSpeed = 0;
        side2SideTargtePos = elevatorConstants.armLeftLimit;
      }
    }

    //PID to determine speed when in computer controlled mode
    if (!side2SideManualMode) side2SideSpeed = side2sidePID.calculate(side2SideTargtePos - side2SideCurrentPos); 
}

  public void moveUpAndDown (double speed) {
    //Calculate if user is in control
    if (Math.abs(RobotContainer.Deadzone(speed)) > 0) upAndDownManualMode = true;
    else if (upAndDownManualMode && Math.abs(RobotContainer.Deadzone(speed)) > 0) {
      upAndDownTargetPos = upAndDownCurrentPos;
      upAndDownManualMode = false; 
    }

    //Determine speed when in manual mode  
    if(upAndDownManualMode) {
      upAndDownSpeed = RobotContainer.Deadzone(speed);
      //if it's past the top limit and still moving up, stop moving and move back down
      if ((upAndDownSpeed>0) && (upAndDownCurrentPos>=elevatorConstants.armTopLimit)) {
        upAndDownSpeed = 0;
        upAndDownTargetPos = elevatorConstants.armTopLimit;
      }
      //if it's below the bottom limit and still moving down, stop moving and move back up
      if ((upAndDownSpeed<0) && (upAndDownCurrentPos<=elevatorConstants.armBottomLimit)) {
        upAndDownSpeed = 0;
        upAndDownTargetPos = elevatorConstants.armBottomLimit;
      }
    }
    
    //PID to determine speed when in computer controlled mode
    if (!upAndDownManualMode) upAndDownSpeed = upAndDownPID.calculate(upAndDownTargetPos - upAndDownCurrentPos);
  }

  public double getHeight() {
    return RobotContainer.upAndDownMotor.getAbsoluteEncoder().getPosition(); //TODO Find formula to calculate
  }

  public double getSideToSide() {
    return RobotContainer.side2SideMotor.getAbsoluteEncoder().getPosition(); //TODO Find formula to calculate
  }
  
}

