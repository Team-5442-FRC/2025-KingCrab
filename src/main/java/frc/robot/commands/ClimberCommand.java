// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberCommand extends Command {
  /** Creates a new ClimberCommand. */
  public ClimberCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.climber); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // tests input to see if the input is greater than the deadzone and sets the climb motor speed to its ascosatied button pressing amount
    //What if you press down both triggers?  ¯\_(ツ)_/¯
    if (RobotContainer.Deadzone(RobotContainer.xbox1.getRightTriggerAxis()) > 0) {
      RobotContainer.climber.setClimbSpeed(-RobotContainer.xbox1.getRightTriggerAxis());
      RobotContainer.climber.setServoAngle(180);
    }
    else if (RobotContainer.Deadzone(RobotContainer.xbox1.getLeftTriggerAxis()) > 0) {
      RobotContainer.climber.setClimbSpeed(RobotContainer.xbox1.getLeftTriggerAxis());
      RobotContainer.climber.setServoAngle(0);
    }
    else RobotContainer.climber.setClimbSpeed(0);
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
