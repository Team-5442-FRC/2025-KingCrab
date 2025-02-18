// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Robot;
// import frc.robot.RobotContainer;
// import frc.robot.Constants.intakeConstants;
// import frc.robot.subsystems.Intake;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class IntakeCommand extends Command {
//   /** Creates a new IntakeCommands. */
//   public IntakeCommand() {
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(RobotContainer.intake);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if (RobotContainer.xbox2.getAButton()) RobotContainer.intake.setIntakeSpeed(intakeConstants.LeftSpeed, intakeConstants.RightSpeed); // If the A button is pressed on the second xbox, the intake motors will be set to the constant intake speed
//     else if (RobotContainer.xbox2.getXButton()) RobotContainer.intake.setIntakeSpeed(intakeConstants.LeftReverseSpeed, intakeConstants.RightReverseSpeed); // If the X button is pressed on the second xbox, the intake motors will be set to the constant reverse intake speed
//     else RobotContainer.intake.setIntakeSpeed(0, 0); // If a button is not pressed, the intake speed will be set to 0

//     if (RobotContainer.xbox2.getPOV() == 0) RobotContainer.intake.setPivotAngle(intakeConstants.PivotAngleUp); // If the top button of the plus is pressed on the second xbox, the angle of the pivot will be set to the up position
//     else if (RobotContainer.xbox2.getPOV() == 90) RobotContainer.intake.setPivotAngle(intakeConstants.PivotAngleAlgae); // If the right button is pressed, the angle will be set to the algae position
//     else if (RobotContainer.xbox2.getPOV() == 180) RobotContainer.intake.setPivotAngle(intakeConstants.PivotAngleFloor); // If the bottom button is pressed, the angle will be set to the floor position
    
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
