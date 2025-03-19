// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RumbleCommand extends Command {

  WaitCommand rumbleTimer1 = new WaitCommand(0.2);
  WaitCommand waitTimer = new WaitCommand(0.2);
  WaitCommand rumbleTimer2 = new WaitCommand(0.25);

  /** Creates a new RumbleCommand. */
  public RumbleCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rumbleTimer1.schedule();
    RobotContainer.xbox1.setRumble(RumbleType.kBothRumble, 1);
    RobotContainer.xbox2.setRumble(RumbleType.kBothRumble, 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (rumbleTimer1.isFinished() && !waitTimer.isScheduled()) {
      waitTimer.schedule();
      RobotContainer.xbox1.setRumble(RumbleType.kBothRumble, 0);
      RobotContainer.xbox2.setRumble(RumbleType.kBothRumble, 0);
    }
    if (waitTimer.isFinished() && rumbleTimer1.isFinished() && !rumbleTimer2.isScheduled()) {
      rumbleTimer2.schedule();
      RobotContainer.xbox1.setRumble(RumbleType.kBothRumble, 1);
      RobotContainer.xbox2.setRumble(RumbleType.kBothRumble, 1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.xbox1.setRumble(RumbleType.kBothRumble, 0);
    RobotContainer.xbox2.setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rumbleTimer2.isFinished();
  }
}
