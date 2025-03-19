// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.manipulatorConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManipulatorCommand extends Command {

  public int state = 0;
  double speed = 0;

  public boolean aPressed, bPressed, xPressed, yPressed;
  public boolean a, b, x, y;
  public boolean aReleased, bReleased, xReleased, yReleased;

  /** Creates a new ManipulatorCommand. */
  public ManipulatorCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.manipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.manipulator.setWristSpeed(RobotContainer.xbox2.getRightX());

    if (RobotContainer.xbox2.getPOV() == 270) RobotContainer.manipulator.setWristAngle(Math.toRadians(-90));
    if (RobotContainer.xbox2.getPOV() == 0) RobotContainer.manipulator.setWristAngle(Math.toRadians(0));
    if (RobotContainer.xbox2.getPOV() == 90) RobotContainer.manipulator.setWristAngle(Math.toRadians(90));

    // Check for Button States
    if (!a && RobotContainer.xbox2.getAButton()) { aPressed = true; aReleased = false; }
    else if (a && !RobotContainer.xbox2.getAButton()) { aReleased = true; aPressed = false; }
    else { aPressed = false; aReleased = false; }
    a = RobotContainer.xbox2.getAButton();

    if (!b && RobotContainer.xbox2.getBButton()) { bPressed = true; bReleased = false; }
    else if (b && !RobotContainer.xbox2.getBButton()) { bReleased = true; bPressed = false; }
    else { bPressed = false; bReleased = false; }
    b = RobotContainer.xbox2.getBButton();

    if (!x && RobotContainer.xbox2.getXButton()) { xPressed = true; xReleased = false; }
    else if (x && !RobotContainer.xbox2.getXButton()) { xReleased = true; xPressed = false; }
    else { xPressed = false; xReleased = false; }
    x = RobotContainer.xbox2.getXButton();

    if (!y && RobotContainer.xbox2.getYButton()) { yPressed = true; yReleased = false; }
    else if (y && !RobotContainer.xbox2.getYButton()) { yReleased = true; yPressed = false; }
    else { yPressed = false; yReleased = false; }
    y = RobotContainer.xbox2.getYButton();
    
    ///// At Rest \\\\\
    if (state == 0) {
      speed = 0;

      if (aPressed) state = 1; // Go to Coral Intake
      if (bPressed) state = 3; // Go to Algae Intake
      if (xPressed) state = -1; // Go to Intake Reverse
    }
    
    ///// Coral Intake \\\\\
    if (state == 1) {
      speed = manipulatorConstants.CoralIntakeSpeed;

      if (RobotContainer.manipulatorProxSensor.get()) {
        speed = 0;
        state = 2; // Go to Coral Placement
      }
      if (xPressed) state = 0; // Return to At Rest
    }

    ///// Coral Placement \\\\\
    if (state == 2) {
      if (aPressed) speed = manipulatorConstants.CoralPlaceSpeed;
      else if (aReleased) speed = 0;
      else if (xPressed) speed = manipulatorConstants.IntakeReverseSpeed;
      else if (xReleased) speed = 0;

      if (!RobotContainer.manipulatorProxSensor.get()) state = 0; // Return to At Rest
    }

    ///// Algae Intake \\\\\
    if (state == 3) {
      speed = manipulatorConstants.AlgaeIntakeSpeed;

      if (RobotContainer.manipulatorProxSensor.get()) state = 4; // Go to Algae Hold
      else if (bReleased) state = 4; // Go to Algae Hold
      else if (yPressed) state = 0; // Return to At Rest
    }

    ///// Algae Hold \\\\\
    if (state == 4) {
      speed = manipulatorConstants.AlgaeHoldSpeed;

      if (yPressed) state = 5; // Return to At Rest
    }

    ///// Algae Shoot \\\\\
    if (state == 5) {
      speed = manipulatorConstants.AlgaeShootSpeed;

      if (yReleased) state = 0;
    }

    ///// Reverse Intake \\\\\
    if (state == -1) {
      speed = manipulatorConstants.IntakeReverseSpeed;

      if (xReleased) state = 0; // Return to At Rest
    }

    RobotContainer.manipulator.setManipulatorSpeed(speed); // Set speed based on state

    // if (RobotContainer.xbox2.getAButtonPressed()) RobotContainer.manipulator.setManipulatorSpeed(manipulatorConstants.manipulatorIntakeSpeed);
    // else if (RobotContainer.xbox2.getAButtonReleased()) RobotContainer.manipulator.setManipulatorSpeed(0);
    // else if (RobotContainer.xbox2.getXButton()) RobotContainer.manipulator.setManipulatorSpeed(manipulatorConstants.manipulatorOutakeSpeed);
    // else if (RobotContainer.xbox2.getXButtonReleased()) RobotContainer.manipulator.setManipulatorSpeed(0);
    // else if (RobotContainer.manipulatorProxSensor.get()) RobotContainer.manipulator.setManipulatorSpeed(0);

    // if (RobotContainer.xbox2.getBButton() && RobotContainer.manipulatorProxSensor.get()) RobotContainer.manipulator.setManipulatorSpeed(manipulatorConstants.manipulatorIntakeSpeed);
    // else if (RobotContainer.xbox2.getYButton()) RobotContainer.manipulator.setManipulatorSpeed(manipulatorConstants.manipulatorOutakeSpeed);
    // else if (!RobotContainer.manipulatorProxSensor.get()) RobotContainer.manipulator.setManipulatorSpeed(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
