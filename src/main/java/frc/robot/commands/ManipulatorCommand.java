// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.manipulatorConstants;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManipulatorCommand extends Command {
  /** Creates a new ManipulatorCommand. */
  public ManipulatorCommand() {
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.manipulator.rotateWrist();

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
