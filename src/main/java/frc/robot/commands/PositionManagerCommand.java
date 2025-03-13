// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ButtonBox;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PositionManagerCommand extends Command {

  ButtonBox buttonBox = new ButtonBox();
  Notifier buttonThread = new Notifier(buttonBox);

  /** Creates a new PositionManagerCommand. */
  public PositionManagerCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.positionManager);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    buttonThread.startPeriodic(0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int boxValue = ButtonBox.readBox();
    int[] target = ButtonBox.lookup(boxValue);
    boolean isAlgae = ButtonBox.isAlgae(boxValue);

    RobotContainer.positionManager.setReefTarget(target[1] % 2 == 1, target[0], (target[1] / 2) + 1, isAlgae);
    // if (RobotContainer.xbox2.getRightBumperButton()) RobotContainer.isAutomaticPositioningMode = true;
    // else RobotContainer.isAutomaticPositioningMode = false;

    if (RobotContainer.xbox1.getRightBumperButton() || RobotContainer.xbox1.getXButton() || RobotContainer.xbox1.getYButton()) RobotContainer.isAutomaticDriveMode = true;
    else RobotContainer.isAutomaticDriveMode = false;

    if (RobotContainer.xbox1.getXButton()) RobotContainer.positionManager.driveLeftCoralStation = true;
    else RobotContainer.positionManager.driveLeftCoralStation = false;
    if (RobotContainer.xbox1.getBButton()) RobotContainer.positionManager.driveRightCoralStation = true;
    else RobotContainer.positionManager.driveRightCoralStation = false;

    if (RobotContainer.xbox2.getRightBumperButtonPressed()) {
      RobotContainer.positionManager.updatePositions(
        RobotContainer.positionManager.calculateArmPivot(target[0]),
        // RobotContainer.positionManager.calculateArmExtend(target[0], 0.5),
        RobotContainer.positionManager.calculateHeight(RobotContainer.positionManager.calculateArmPivot(target[0]),
        RobotContainer.positionManager.reefLevelToHeight(target[0])),
        RobotContainer.positionManager.calculateSideToSide(0, target[1] % 2 == 0),
        RobotContainer.positionManager.calculateWristAngle(target[0])
      );
    }
    if (RobotContainer.xbox2.getPOV() == 0) {
      RobotContainer.positionManager.updatePositions(
        RobotContainer.positionManager.calculateArmPivot(5),
        RobotContainer.positionManager.calculateHeight(RobotContainer.positionManager.calculateArmPivot(5),
        RobotContainer.positionManager.reefLevelToHeight(5)),
        0,
        RobotContainer.positionManager.calculateWristAngle(5)
      );
    }
    if (RobotContainer.xbox2.getPOV() == 90) { // Set to starting/lowest position
      RobotContainer.positionManager.updatePositions(
        Math.toRadians(170),
        0,
        0,
        0
      );
    }
    if (RobotContainer.xbox2.getPOV() == 180) {
      RobotContainer.positionManager.updatePositions(
        RobotContainer.positionManager.calculateArmPivot(6),
        RobotContainer.positionManager.calculateHeight(RobotContainer.positionManager.calculateArmPivot(6),
        RobotContainer.positionManager.reefLevelToHeight(6)),
        0,
        RobotContainer.positionManager.calculateWristAngle(6)
      );
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
