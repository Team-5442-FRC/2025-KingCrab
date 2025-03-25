// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.climberConstants;

public class Climber extends SubsystemBase {
  // makes a speed varible set to zero
  double speed = 0;
  double position = 0;
  double targetAngle = 0;
  double flipperTargetAngle = 0;
  double flipperSpeed = 0;
  public boolean flipperPID = false;

  /** Creates a new Climber. */
  public Climber() {}

  @Override
  public void periodic() {
    if (flipperPID) {
      flipperSpeed = (getFlipperAngle() - flipperTargetAngle) * climberConstants.flipperMotorPIDkp;
      RobotContainer.flipperMotor.set(flipperSpeed);
    }
    else RobotContainer.flipperMotor.set(0);
    RobotContainer.winchMotor.set(-speed);
    RobotContainer.climberServo.setAngle(targetAngle);

    SmartDashboard.putNumber("Climber Encoder", getEncoderValue());
    SmartDashboard.putNumber("Climber Speed", speed);

    SmartDashboard.putNumber("Flipper Angle", Math.toDegrees(getFlipperAngle()));
    SmartDashboard.putNumber("Flipper Target Angle", Math.toDegrees(flipperTargetAngle));
    SmartDashboard.putNumber("Flipper Speed", flipperSpeed);
  }

  // creates new speed and sets it the the prevousily metioned speed
  public void setClimbSpeed(double speed) {
    // if (speed > 0 && getEncoderValue() > climberConstants.MaxPosition) speed = 0;
    // else if (speed < 0 && getEncoderValue() < climberConstants.MinPosition) speed = 0;
    this.speed = speed;
  }

  public double getFlipperAngle() {
    return -(RobotContainer.flipperMotor.getEncoder().getPosition()/25) * (Math.PI * 2);
  }

  public void setFlipperTargetAngle(double flipperAngle) {
    this.flipperTargetAngle = flipperAngle;
  }

  public void setServoAngle(double angle) {
    this.targetAngle = angle;
  }

  public double getEncoderValue() {
    return RobotContainer.winchMotor.getEncoder().getPosition();
  }
}
