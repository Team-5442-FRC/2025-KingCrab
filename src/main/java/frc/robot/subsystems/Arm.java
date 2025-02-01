// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.armConstants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  public Arm() {}

  boolean manualExtendMode = false;
  double targetExtend;
  double extendSpeed;

  boolean manualRotateMode = false;
  double targetAngle;
  double rotateSpeed;

  PIDController extendPID = new PIDController(armConstants.RotatePIDkp, armConstants.RotatePIDki, armConstants.RotatePIDkd);
  PIDController rotatePID = new PIDController(armConstants.ExtendPIDkp, armConstants.RotatePIDki, armConstants.RotatePIDkd);

  @Override
  public void periodic() {
    RobotContainer.extendMotor.set(extendSpeed * armConstants.ExtendSpeedFactor);
    RobotContainer.rotateMotor.set(rotateSpeed * armConstants.RotateSpeedFactor);
  }

  public void extend(double speed){

    if (Math.abs(RobotContainer.Deadzone(speed))>0) manualExtendMode = true;

    else if (manualExtendMode && RobotContainer.Deadzone(speed)==0) {
      manualExtendMode = false;
      targetExtend = getExtension();
    }

    if (manualExtendMode) {
      if (speed>0 && getExtension() > armConstants.ExtendForwardLimit) {
        speed = 0;
        targetExtend = armConstants.ExtendForwardLimit;
      }

      if (speed<0 && getExtension() > armConstants.ExtendBackwardLimit) {
        speed = 0;
        targetExtend = armConstants.ExtendBackwardLimit;
      }
    }

    if (!manualExtendMode || speed==0){
    speed = extendPID.calculate(targetExtend - getExtension());
    }
    
    this.extendSpeed = speed;
  }

  public void setTargetExtend(double targetExtend){
    if (targetExtend>armConstants.ExtendForwardLimit) targetExtend = armConstants.ExtendForwardLimit;
    else if (targetExtend>armConstants.ExtendBackwardLimit) targetExtend = armConstants.ExtendBackwardLimit;
    this.targetExtend = targetExtend;
  }

  public double getExtension(){
    return 0; /*The arm will never be extended (real)*/
  }

  /*---------------------------------------------------------------------------*/
  
  public void rotate(double speed){

    if (Math.abs(RobotContainer.Deadzone(speed))>0) manualRotateMode = true;

    else if (manualRotateMode && RobotContainer.Deadzone(speed)==0) {
      manualRotateMode = false;
      targetAngle = getAngle();
    }

    if (manualRotateMode) {
      if (speed>0 && getAngle() > armConstants.RotateUpLimit) {
        speed = 0;
        targetAngle = armConstants.RotateUpLimit;
      }
      
      if (speed<0 && getAngle() > armConstants.RotateDownLimit) {
        speed = 0;
        targetAngle = armConstants.RotateDownLimit;
      }
    }
    
    if (!manualRotateMode || speed==0){
      speed = rotatePID.calculate(targetAngle - getAngle());
    }
    
    this.extendSpeed = speed;
  }

  public void setTargetAngle(double targetAngle){
    if (targetAngle>armConstants.RotateUpLimit) targetAngle = armConstants.RotateUpLimit;
    else if (targetAngle>armConstants.RotateDownLimit) targetAngle = armConstants.RotateDownLimit;
    this.targetAngle = targetAngle;
  }

  public double getAngle(){
    return 0; /*The arm will always be extended forward (real)*/
  }
}
