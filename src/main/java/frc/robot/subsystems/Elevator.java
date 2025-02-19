// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.elevatorConstants;
import frc.robot.subsystems.DriveModes;

public class Elevator extends SubsystemBase {
  
boolean side2SideManualMode = true;// This should set the target pos to the current pos since it is true
boolean upAndDownManualMode = true;// "
double side2SideTargtePos = 0;
double upAndDownTargetPos = elevatorConstants.PivotToFloorOffset;
double side2SideCurrentPos = 0;
double upAndDownCurrentPos = elevatorConstants.PivotToFloorOffset;
double side2SideSpeed = 0;
double upAndDownSpeed = 0;
PIDController upAndDownPID = new PIDController(0.2, 0, 0); // D was 0.02
PIDController side2sidePID = new PIDController(.00001, 0, 0);

// Counters for encoder rotations
double lastHeight = 0;
double height = 0;
int intHeight = 1;
double combinedHeight = 0;

  /** Creates a new ElevatorSubsystem. */
  public Elevator() {}

  @Override
  public void periodic() {

    height = RobotContainer.elevatorEncoder.get();
    if (lastHeight > 0.75 && height < 0.25) intHeight += 1;
    if (lastHeight < 0.25 && height > 0.75) intHeight -= 1;
    combinedHeight = intHeight + height - elevatorConstants.UpAndDownOffset;
    lastHeight = height;

    upAndDownCurrentPos = getHeight();
    side2SideCurrentPos = getSideToSide();
    
    RobotContainer.upAndDownMotor.set(upAndDownSpeed);
    RobotContainer.side2SideMotor.set(side2SideSpeed);
    
    SmartDashboard.putNumber("Elevator Encoder", RobotContainer.elevatorEncoder.get());
    SmartDashboard.putNumber("Elevator Combined Height", combinedHeight);
    SmartDashboard.putNumber("Elevator Speed", upAndDownSpeed);
    SmartDashboard.putNumber("Elevator Height", getHeight());

    SmartDashboard.putNumber("Elevator Side To Side Encoder", RobotContainer.side2SideMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Elevator Side To Side Value", getSideToSide());
    SmartDashboard.putNumber("Elevator Side To Side Speed", side2SideSpeed);
  }

  public void setSide2SidePos(double side2SideTargtePos) {
    if (side2SideTargtePos > elevatorConstants.ArmRightLimit) {
      side2SideTargtePos = elevatorConstants.ArmRightLimit;
    }
    if (side2SideTargtePos < elevatorConstants.ArmLeftLimit) {
      side2SideTargtePos = elevatorConstants.ArmLeftLimit;
    }
    this.side2SideTargtePos = side2SideTargtePos;
  }
    
  public void setUpAndDownPos(double upAndDownTargetPos) {
    if (upAndDownTargetPos > elevatorConstants.ArmTopLimit) {
      upAndDownTargetPos = elevatorConstants.ArmTopLimit;
    }
    //if it wants to move past the limit, this won't let it
    if (upAndDownTargetPos < elevatorConstants.ArmBottomLimit) {
      upAndDownTargetPos = elevatorConstants.ArmBottomLimit;
    }
    this.upAndDownTargetPos = upAndDownTargetPos;
  }

  public void moveSide2Side(double speed) {

    //Calculates user input to determine if user is controlling it
    if (Math.abs(RobotContainer.Deadzone(speed))>0) side2SideManualMode = true;
    else if (side2SideManualMode && Math.abs(RobotContainer.Deadzone(speed)) == 0) {
      side2SideTargtePos = side2SideCurrentPos; 
      side2SideManualMode = false;
    }

    //Limits the movement of side to side motion from user movement to inside the limits
    if(side2SideManualMode) {
      side2SideSpeed = RobotContainer.Deadzone(speed) * elevatorConstants.Side2SideSpeedFactor;
      //if it's past the right limit and still moving right, stop moving and move back left
      if ((side2SideSpeed>0) && (side2SideCurrentPos>elevatorConstants.ArmRightLimit)) {
        side2SideSpeed = 0;
        side2SideTargtePos = elevatorConstants.ArmRightLimit;
      }
      //if it's past the left limit and still moving left, stop moving and move back right
      if ((side2SideSpeed<0) && (side2SideCurrentPos<elevatorConstants.ArmLeftLimit)) {
        side2SideSpeed = 0;
        side2SideTargtePos = elevatorConstants.ArmLeftLimit;
      }
    }

    //PID to determine speed when in computer controlled mode
    if (!side2SideManualMode) {
      side2SideSpeed = side2sidePID.calculate(side2SideTargtePos - side2SideCurrentPos); 
      if (side2SideSpeed > 0 && side2SideCurrentPos > elevatorConstants.ArmRightLimit) side2SideSpeed = 0;
      if (side2SideSpeed < 0 && side2SideCurrentPos < elevatorConstants.ArmLeftLimit) side2SideSpeed = 0;
    } 
  }

  public void moveUpAndDown (double speed) {
    //Calculate if user is in control
    if (Math.abs(RobotContainer.Deadzone(speed)) > 0) upAndDownManualMode = true;
    else if (upAndDownManualMode && Math.abs(RobotContainer.Deadzone(speed)) == 0) {
      upAndDownTargetPos = upAndDownCurrentPos;
      upAndDownManualMode = false; 
    }

    //Determine speed when in manual mode  
    if(upAndDownManualMode) {
      upAndDownSpeed = RobotContainer.Deadzone(speed) * elevatorConstants.UpAndDownSpeedFactor;
      //if it's past the top limit and still moving up, stop moving and move back down
      if ((upAndDownSpeed>0) && (upAndDownCurrentPos>=elevatorConstants.ArmTopLimit)) {
        upAndDownSpeed = 0;
        upAndDownTargetPos = elevatorConstants.ArmTopLimit;
      }

      //if it's below the bottom limit and still moving down, stop moving and move back up
      if ((upAndDownSpeed<0) && (upAndDownCurrentPos<=elevatorConstants.ArmBottomLimit)) {
        upAndDownSpeed = 0;
        upAndDownTargetPos = elevatorConstants.ArmBottomLimit;
      }
    }
    
    //PID to determine speed when in computer controlled mode
    if (!upAndDownManualMode) {
      upAndDownSpeed = upAndDownPID.calculate(upAndDownCurrentPos - upAndDownTargetPos);
      if (upAndDownSpeed > 0 && upAndDownCurrentPos > elevatorConstants.ArmTopLimit) upAndDownSpeed = 0;
      if (upAndDownSpeed < 0 && upAndDownCurrentPos < elevatorConstants.ArmBottomLimit) upAndDownSpeed = 0;
    }
  }

  /**Height of the pivot point in inches from floor*/
  public double getHeight() {
    return (combinedHeight * elevatorConstants.InchesPerRotation) + elevatorConstants.PivotToFloorOffset;
  }

  public double getSideToSide() {
    return ((RobotContainer.side2SideMotor.getEncoder().getPosition() / 49) * (30/26d)) / (5/3d) + elevatorConstants.Side2SideOffset; // TODO figure out why it needs the 5/3 constant (incorrect gearbox ratio?)
  }
  
}

