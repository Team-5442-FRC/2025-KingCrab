// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//All of the imports needed
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.armConstants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  public Arm() {}

  //3 variables for the extend functions
  boolean manualExtendMode = false; //If the user is using the control stick
  double targetExtend; //The target that the arm should extend to (when not manual)
  double extendSpeed; //The speed that the arm should go (when manual)

  //Versions of the above variables, just with the rotate functions
  boolean manualRotateMode = false;
  double targetAngle;
  double rotateSpeed;

  //Initiallizing the PIDs
  PIDController extendPID = new PIDController(armConstants.RotatePIDkp, armConstants.RotatePIDki, armConstants.RotatePIDkd);
  PIDController rotatePID = new PIDController(armConstants.ExtendPIDkp, armConstants.RotatePIDki, armConstants.RotatePIDkd);

  @Override
  public void periodic() {
    //At all times, set the motor to the speed given
    RobotContainer.extendMotor.set(extendSpeed * armConstants.ExtendSpeedFactor);
    RobotContainer.rotateMotor.set(rotateSpeed * armConstants.RotateSpeedFactor);
  }

  public void extend(double speed){

    //If the speed is enough above the deadzone, turn on manual mode.
    if (Math.abs(RobotContainer.Deadzone(speed))>0) manualExtendMode = true;

    //If the user has stopped using the stick and manual mode is on, turn it off and set the target to be where it is right now.
    else if (manualExtendMode && RobotContainer.Deadzone(speed)==0) {
      manualExtendMode = false;
      targetExtend = getExtension();
    }

    if (manualExtendMode) {
      //If the speed is positive, and is trying to move past the limit, set it to 0 and set the target to the limit
      if (speed>0 && getExtension() > armConstants.ExtendForwardLimit) {
        speed = 0;
        targetExtend = armConstants.ExtendForwardLimit;
      }

      //If the speed is negative, and is trying to move past the limit, set it to 0 and set the target to the limit
      if (speed<0 && getExtension() > armConstants.ExtendBackwardLimit) {
        speed = 0;
        targetExtend = armConstants.ExtendBackwardLimit;
      }
    }

    //If it isn't in manual mode or if something caused the speed to be zero, refer to the PID to set the speed
    if (!manualExtendMode || speed==0){
    speed = extendPID.calculate(targetExtend - getExtension());
    }
    
    //Set the global speed to the speed in this function 
    this.extendSpeed = speed;
  }

  public void setTargetExtend(double targetExtend){
    //Checking if the target is within the limits
    if (targetExtend > armConstants.ExtendForwardLimit) targetExtend = armConstants.ExtendForwardLimit;
    else if (targetExtend > armConstants.ExtendBackwardLimit) targetExtend = armConstants.ExtendBackwardLimit;
    this.targetExtend = targetExtend;
  }

  //TODO, need to do math to find it at some point
  public double getExtension(){
    return 0; /*The arm will never be extended (real)*/
  }

  /*---------------------------------------------------------------------------*/
  
  public void rotate(double speed){

    //If the speed is enough above the deadzone, turn on manual mode.
    if (Math.abs(RobotContainer.Deadzone(speed))>0) manualRotateMode = true;

    //If the user has stopped using the stick and manual mode is on, turn it off and set the target to be where it is right now.
    else if (manualRotateMode && RobotContainer.Deadzone(speed)==0) {
      manualRotateMode = false;
      targetAngle = getAngle();
    }

    if (manualRotateMode) {
      //If the speed is positive, and is trying to move past the limit, set it to 0 and set the target to the limit
      if (speed>0 && getAngle() > armConstants.RotateUpLimit) {
        speed = 0;
        targetAngle = armConstants.RotateUpLimit;
      }
      
      //If the speed is negative, and is trying to move past the limit, set it to 0 and set the target to the limit
      if (speed<0 && getAngle() > armConstants.RotateDownLimit) {
        speed = 0;
        targetAngle = armConstants.RotateDownLimit;
      }
    }
    
    //If it isn't in manual mode or if something caused the speed to be zero, refer to the PID to set the speed
    if (!manualRotateMode || speed==0){
      speed = rotatePID.calculate(targetAngle - getAngle());
    }
    
    //Set the global speed to the speed in this function 
    this.extendSpeed = speed;
  }

  public void setTargetAngle(double targetAngle){
    //Checking if the target is within the limits
    if (targetAngle>armConstants.RotateUpLimit) targetAngle = armConstants.RotateUpLimit;
    else if (targetAngle>armConstants.RotateDownLimit) targetAngle = armConstants.RotateDownLimit;
    this.targetAngle = targetAngle;
  }

  //TODO, need to do math to find it at some point
  public double getAngle(){
    return 0; /*The arm will always be extended forward (real)*/
  }
}
