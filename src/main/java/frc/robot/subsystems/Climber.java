// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.climberConstants;

public class Climber extends SubsystemBase {
  // makes a speed varible set to zero
  double speed = 0;
  double position = 0;

  /** Creates a new Climber. */
  public Climber() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // position = RobotContainer.climberMotor.getAbsoluteEncoder().getPosition();

    // sets speed to newley created speed
    RobotContainer.climberMotor.set(speed);
  }
  // creates new speed and sets it the the prevousily metioned speed
  public void setClimbSpeed(double speed) {
    // if (position > climberConstants.MaxPosition && speed > 0) speed = 0;
    // if (position < climberConstants.MinPosition && speed < 0) speed = 0;
    this.speed = speed;
  }
}
