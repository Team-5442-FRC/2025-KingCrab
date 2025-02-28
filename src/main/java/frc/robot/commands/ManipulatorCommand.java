// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.manipulatorConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManipulatorCommand extends Command {
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

    if (RobotContainer.xbox2.getAButton()) RobotContainer.manipulator.setManipulatorSpeed(manipulatorConstants.manipulatorIntakeSpeed);
    else if (RobotContainer.xbox2.getXButton()) RobotContainer.manipulator.setManipulatorSpeed(manipulatorConstants.manipulatorOutakeSpeed);
    else RobotContainer.manipulator.setManipulatorSpeed(0);

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
