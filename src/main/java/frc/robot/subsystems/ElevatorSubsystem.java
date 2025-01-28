// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.elevatorConstants;
import frc.robot.subsystems.DriveModes;

public class ElevatorSubsystem extends SubsystemBase {
  
boolean manualMode;
double targtePos;
double currentPos;
double speed;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {}

  @Override
  public void periodic() {
    if (1>0) {manualMode = true;}//TODO Add deadzone thing
    else if (manualMode && 1>0) {targtePos = currentPos; manualMode = false;} //TODO Add deadzone thing

  }
  public void user () {
    if(manualMode) {
      if ((speed>0) && (currentPos>=elevatorConstants.armRightLimit)) {
        speed = 0;
        targtePos = elevatorConstants.armRightLimit;
      }
      if ((speed<0) && (currentPos>=elevatorConstants.armLeftLimit)) {
        speed = 0;
        targtePos = elevatorConstants.armLeftLimit;
      }
    }
  }
}
