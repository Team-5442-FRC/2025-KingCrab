// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCommand extends Command {
  /** Creates a new ElevatorCommand. */
  public ElevatorCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.elevator);
  }                                 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //send in the inputs from the controller, calculate the speed each motor should be moving at, and set the motors to that speed
    RobotContainer.elevator.moveSide2Side(RobotContainer.xbox2.getLeftX());
    RobotContainer.elevator.moveUpAndDown(-RobotContainer.xbox2.getLeftY());

    // if (RobotContainer.xbox2.getLeftBumperButtonPressed()) RobotContainer.elevator.setSide2SidePos(elevatorConstants.ArmLeftLimit+0.5);
    // else if (RobotContainer.xbox2.getRightBumperButtonPressed()) RobotContainer.elevator.setSide2SidePos(elevatorConstants.ArmRightLimit-0.5);
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
