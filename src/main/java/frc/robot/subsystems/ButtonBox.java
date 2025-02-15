// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ButtonBox extends SubsystemBase {
  /** Creates a new ButtonBox. */
  public ButtonBox() {}

  public static int readBox() {
    int totalValue = 0;
    if (RobotContainer.arduino.getRawButton(0)) totalValue += 1;
    if (RobotContainer.arduino.getRawButton(1)) totalValue += 2;
    if (RobotContainer.arduino.getRawButton(2)) totalValue += 4;
    if (RobotContainer.arduino.getRawButton(3)) totalValue += 8;
    if (RobotContainer.arduino.getRawButton(4)) totalValue += 16;
    if (RobotContainer.arduino.getRawButton(5)) totalValue += 32;
    
    return totalValue;
  }

  public static int[] lookup(int value) {
    int level = (value % 4) + 1;
    int row = (value + 1) / 4;
    return new int[] {level,row};
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
