// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


//All of the imports needed
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.armConstants;
import frc.robot.Constants.driveConstants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  public Arm() {}

  //3 variables for the extend functions
  // boolean manualExtendMode = true; //If the user is using the control stick
  // double targetExtend; //The target that the arm should extend to (when not manual)
  // double extendSpeed; //The speed that the arm should go (when manual)

  //Versions of the above variables, just with the rotate functions
  boolean manualRotateMode = true;
  double targetAngle = Math.toRadians(170); // Should be overwritten by manual mode on startup
  double rotateSpeed;

  //Initiallizing the PIDs
  // PIDController extendPID = new PIDController(armConstants.ExtendPIDkp, armConstants.ExtendPIDki, armConstants.ExtendPIDkd);
  PIDController rotatePID = new PIDController(armConstants.RotatePIDkp, armConstants.RotatePIDki, armConstants.RotatePIDkd);
  SlewRateLimiter rotateLimiter = new SlewRateLimiter(8);

  @Override
  public void periodic() {
    //At all times, set the motor to the speed given
    // RobotContainer.extendMotor.set(-extendSpeed); // Inverted motor
    RobotContainer.rotateMotor.set(-rotateSpeed); // Inverted motor
    
    SmartDashboard.putNumber("Pivot Raw Encoder", RobotContainer.pivotEncoder.get());
    SmartDashboard.putNumber("Pivot Degrees", Math.toDegrees(getAngle()));
    SmartDashboard.putNumber("Pivot Speed", rotateSpeed);
  }

  // public void extend(double speed){

  //   //If the speed is enough above the deadzone, turn on manual mode.
  //   if (Math.abs(RobotContainer.Deadzone(speed))>0) manualExtendMode = true;

  //   //If the user has stopped using the stick and manual mode is on, turn it off and set the target to be where it is right now.
  //   else if (manualExtendMode && RobotContainer.Deadzone(speed)==0) {
  //     manualExtendMode = false;
  //     targetExtend = getExtension();
  //   }

  //   if (manualExtendMode) {
  //     //If the speed is positive, and is trying to move past the limit, set it to 0 and set the target to the limit
  //     if (speed>0 && getExtension() > armConstants.ExtendForwardLimit) {
  //       speed = 0;
  //       targetExtend = armConstants.ExtendForwardLimit;
  //     }

  //     //If the speed is negative, and is trying to move past the limit, set it to 0 and set the target to the limit
  //     if (speed<0 && getExtension() < armConstants.ExtendBackwardLimit) {
  //       speed = 0;
  //       targetExtend = armConstants.ExtendBackwardLimit;
  //     }

  //     speed *= armConstants.ExtendSpeedFactor;
  //   }

  //   //If it isn't in manual mode or if something caused the speed to be zero, refer to the PID to set the speed
  //   if (!manualExtendMode || speed==0){
  //     speed = extendPID.calculate(getExtension() - targetExtend);
  //     if (speed>0 && getExtension() > armConstants.ExtendForwardLimit) speed = 0;
  //     if (speed<0 && getExtension() < armConstants.ExtendBackwardLimit) speed = 0;
  //   }
    
  //   //Set the global speed to the speed in this function 
  //   this.extendSpeed = speed;
  // }

  // public void setTargetExtend(double targetExtend){
  //   //Checking if the target is within the limits
  //   if (targetExtend > armConstants.ExtendForwardLimit) targetExtend = armConstants.ExtendForwardLimit;
  //   else if (targetExtend < armConstants.ExtendBackwardLimit) targetExtend = armConstants.ExtendBackwardLimit;
  //   this.targetExtend = targetExtend;
  // }

  // /** Returns the extention of the arm in inches. */
  // public double getExtension() {
  //   return -(RobotContainer.extendMotor.getEncoder().getPosition() * 0.38 * 18) / 5 + armConstants.PivotToMinExtend + armConstants.MinExtendToCoral;
  // }

  /*---------------------------------------------------------------------------*/
  
  public void rotate(double speed) {

    //If the speed is enough above the deadzone, turn on manual mode.
    if (Math.abs(RobotContainer.Deadzone(speed, driveConstants.Controller2Deadzone))>0) manualRotateMode = true;

    //If the user has stopped using the stick and manual mode is on, turn it off and set the target to be where it is right now.
    else if (manualRotateMode && RobotContainer.Deadzone(speed, driveConstants.Controller2Deadzone)==0) {
      manualRotateMode = false;
      targetAngle = getAngle();
    }

    if (manualRotateMode) {
      //If the speed is positive, and is trying to move past the limit, set it to 0 and set the target to the limit
      if (speed>0 && getAngle() > armConstants.RotateUpLimit) {
        speed = 0;
        targetAngle = armConstants.RotateUpLimit;
      }
      if (speed<0 && getAngle() < armConstants.RotateDownLimit) {
        speed = 0;
        targetAngle = armConstants.RotateDownLimit;
      }
      
      //   double h = RobotContainer.elevator.getHeight();
      //   double r = getExtension();
      //   //If the speed is negative, and is trying to move past the limit, set it to 0 and set the target to the limit
      //   if (speed < 0 && (getAngle() < Math.acos(h/r) || getAngle() < armConstants.RotateDownLimit)) {
      //   speed = 0;
      //   targetAngle = Math.max(Math.acos(h/r),armConstants.RotateDownLimit);
      // }

      speed *= armConstants.RotateSpeedFactor;
    }
    
    //If it isn't in manual mode or if something caused the speed to be zero, refer to the PID to set the speed
    if (!manualRotateMode || speed == 0){
      speed = rotateLimiter.calculate(rotatePID.calculate(getAngle() - targetAngle));
      if (speed>0 && getAngle() > armConstants.RotateUpLimit) speed = 0;
      if (speed<0 && getAngle() < armConstants.RotateDownLimit) speed = 0;

      if (getAngle() > Math.PI || getAngle() < 0) speed = 0;
    }
    
    this.rotateSpeed = speed;
  }

  public void setTargetAngle(double targetAngle){
    //Checking if the target is within the limits
    double h = RobotContainer.elevator.getHeight();
    // double r = getExtension();
    double r = armConstants.PivotToCoral;
    if (targetAngle > armConstants.RotateUpLimit) targetAngle = armConstants.RotateUpLimit;
    // else if (targetAngle < Math.acos(h/r) || targetAngle < armConstants.RotateDownLimit) targetAngle = Math.max(Math.acos(h/r),armConstants.RotateDownLimit);
    else if (targetAngle < armConstants.RotateDownLimit) targetAngle = armConstants.RotateDownLimit;
    this.targetAngle = targetAngle;
  }

  // public double calculateLowerAngleLimits(double height){
  //   double lowerArmLimit = 0;
  //   return lowerArmLimit;
  // }
  
  public double getAngle(){
    return (-(RobotContainer.pivotEncoder.get() - armConstants.RotateEncoderOffset) + 0.25) * 2 * Math.PI; /*The arm will always be extended forward (real)*/
  }
}
