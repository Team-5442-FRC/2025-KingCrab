// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ButtonBox;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PositionManagerCommand extends Command {
  /** Creates a new PositionManagerCommand. */
  public PositionManagerCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.positionManager);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int[] target = ButtonBox.lookup(ButtonBox.readBox());
    RobotContainer.positionManager.setReefTarget(target[1] % 2 == 1, target[0], (target[1] / 2) + 1);
    if (RobotContainer.xbox2.getRightBumperButtonPressed()) {
      RobotContainer.positionManager.updatePositions(RobotContainer.positionManager.calculateArmPivot(target[0]),RobotContainer.positionManager.calculateArmExtend(target[0], 0.25),RobotContainer.positionManager.calculateHeight(target[0], 0.25, 0),RobotContainer.positionManager.calculateSideToSide(0, target[1] % 2 == 1));
    }
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
