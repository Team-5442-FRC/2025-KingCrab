// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.RobotContainer;

public class ButtonBox implements Runnable {
  /** Creates a new ButtonBox. */
  public ButtonBox() {}

  static int value;

  public void run() {
    int totalValue = 0;
  
    if (RobotContainer.arduino.getRawButton(1)) totalValue += 1;
    if (RobotContainer.arduino.getRawButton(2)) totalValue += 2;
    if (RobotContainer.arduino.getRawButton(3)) totalValue += 4;
    if (RobotContainer.arduino.getRawButton(4)) totalValue += 8;
    if (RobotContainer.arduino.getRawButton(5)) totalValue += 16;
    if (RobotContainer.arduino.getRawButton(6)) totalValue += 32;
  
    if (RobotContainer.arduino.getRawButton(7)) totalValue = -totalValue;

    value = totalValue;
  }

  public static int readBox() { 
    return value;
  }

  public static int[] lookup(int value) {
    int level = (Math.abs(value) % 4) + 1;
    int branch = (Math.abs(value)) / 4;
    return new int[] {level,branch};
  }
  public static boolean isAlgae(int value) {
    return value < 0;
  }
  public static boolean shootBarge(int value) {
    return value == 100;
  }
}
