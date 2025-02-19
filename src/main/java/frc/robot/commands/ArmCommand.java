// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.armConstants;
import frc.robot.Constants.intakeConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmCommand extends Command {
  /** Creates a new ArmCommand. */
  public ArmCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Set the extend speed to the x-axis of the right stick
    RobotContainer.arm.extend(RobotContainer.xbox2.getRightX());
    //Set the rotate speed to the y-axis of the right stick
    RobotContainer.arm.rotate(-RobotContainer.xbox2.getRightY());

    // if (RobotContainer.xbox2.getLeftBumperButtonPressed()) RobotContainer.arm.setTargetExtend(armConstants.ExtendBackwardLimit + 0.5);
    // if (RobotContainer.xbox2.getRightBumperButtonPressed()) RobotContainer.arm.setTargetExtend(armConstants.ExtendForwardLimit - 0.5);

    // if (RobotContainer.xbox3.getAButtonPressed()) intakeConstants.LeftSpeed += 0.05;
    // if (RobotContainer.xbox3.getYButtonPressed()) intakeConstants.LeftSpeed -= 0.05;
    // if (RobotContainer.xbox3.getBButtonPressed()) intakeConstants.RightSpeed += 0.05;
    // if (RobotContainer.xbox3.getXButtonPressed()) intakeConstants.RightSpeed -= 0.05;
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
