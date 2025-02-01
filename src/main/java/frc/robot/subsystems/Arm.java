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

  PIDController extendPID = new PIDController(armConstants.rotatePIDkp, armConstants.rotatePIDki, armConstants.rotatePIDkd);
  PIDController rotatePID = new PIDController(armConstants.extendPIDkp, armConstants.rotatePIDki, armConstants.rotatePIDkd);

  @Override
  public void periodic() {
    RobotContainer.extendMotor.set(extendSpeed);
    RobotContainer.rotateMotor.set(rotateSpeed);
  }

  public void extend(double speed){

    if (Math.abs(RobotContainer.Deadzone(speed))>0) manualExtendMode = true;

    else if (manualExtendMode && RobotContainer.Deadzone(speed)==0) {
      manualExtendMode = false;
      targetExtend = getExtension();
    }

    if (manualExtendMode) {
      if (speed>0 && getExtension() > armConstants.extendForwardLimit) {
        speed = 0;
        targetExtend = armConstants.extendForwardLimit;
      }

      if (speed<0 && getExtension() > armConstants.extendBackwardLimit) {
        speed = 0;
        targetExtend = armConstants.extendBackwardLimit;
      }
    }

    if (!manualExtendMode || speed==0){
      speed = extendPID.calculate(targetExtend - getExtension());
    }
    
    this.extendSpeed = speed;
  }

  public void setTargetExtend(double targetExtend){
    if (targetExtend>armConstants.extendForwardLimit) targetExtend = armConstants.extendForwardLimit;
    else if (targetExtend>armConstants.extendBackwardLimit) targetExtend = armConstants.extendBackwardLimit;
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
      if (speed>0 && getAngle() > armConstants.rotateUpLimit) {
        speed = 0;
        targetAngle = armConstants.rotateUpLimit;
      }
      
      if (speed<0 && getAngle() > armConstants.rotateDownLimit) {
        speed = 0;
        targetAngle = armConstants.rotateDownLimit;
      }
    }
    
    if (!manualRotateMode || speed==0){
      speed = rotatePID.calculate(targetAngle - getAngle());
    }
    
    this.extendSpeed = speed;
  }

  public void setTargetAngle(double targetAngle){
    if (targetAngle>armConstants.rotateUpLimit) targetAngle = armConstants.rotateUpLimit;
    else if (targetAngle>armConstants.rotateDownLimit) targetAngle = armConstants.rotateDownLimit;
    this.targetAngle = targetAngle;
  }

  public double getAngle(){
    return 0; /*Again, the arm will always be pointed straight forward*/
  }
}
