// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveModes;

/** Add your docs here. */
public class AutoCommands {

    public static Command armUpReef4R4 = new Command() {
        WaitCommand wait = new WaitCommand(2);

        @Override
        public void initialize() {
            wait.schedule();

            RobotContainer.positionManager.setReefTarget(true, 4, 4, false);
            RobotContainer.positionManager.updatePositions(
                RobotContainer.positionManager.calculateArmPivot(4),
                RobotContainer.positionManager.calculateHeight(RobotContainer.positionManager.calculateArmPivot(4),
                RobotContainer.positionManager.reefLevelToHeight(4)),
                0,
                RobotContainer.positionManager.calculateWristAngle(4)
            );
        }
        
        @Override
        public boolean isFinished() {
            return wait.isFinished();
        }
    };



    public static Command placeReef4R4 = new Command() {
        WaitCommand timeout = new WaitCommand(2);

        @Override
        public void initialize() {
            timeout.schedule();

            RobotContainer.isAutomaticDriveMode = true;
        }

        @Override
        public void end(boolean interrupted) {
            RobotContainer.isAutomaticDriveMode = false;
        }

        @Override
        public boolean isFinished() { // Finish if in poistion; if it takes more than 2 seconds, ends early
            return timeout.isFinished() || (RobotContainer.positionManager.xSpeed <= 0.05 && RobotContainer.positionManager.ySpeed <= 0.05 && RobotContainer.positionManager.rSpeed <= 1);
        }
    };



    public static Command dropOnReefR4 = new Command() {
        WaitCommand wait = new WaitCommand(0.25);
        
        @Override
        public void initialize() {
            RobotContainer.arm.setTargetAngle(Math.toRadians(110));
            wait.schedule();
        }

        @Override
        public boolean isFinished() {
            return wait.isFinished();
        }
    };



    public static Command backUpR4 = new Command() {
        WaitCommand wait = new WaitCommand(1);

        @Override
        public void initialize() {
            wait.schedule();
        }

        @Override
        public void execute() {
            RobotContainer.drivetrain.setControl(
                DriveModes.driveRobot
                    .withVelocityX(-0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            );
        }

        public boolean isFinished() {
            return wait.isFinished();
        }
    };
    


    public static Command positionAlgae4 = new Command() {
        WaitCommand timeout = new WaitCommand(2);

        @Override
        public void initialize() {
            timeout.schedule();

            RobotContainer.isAutomaticDriveMode = true;

            RobotContainer.positionManager.setReefTarget(false, 2, 4, true);
            RobotContainer.positionManager.updatePositions(
                RobotContainer.positionManager.calculateArmPivot(2),
                RobotContainer.positionManager.calculateHeight(RobotContainer.positionManager.calculateArmPivot(2),
                RobotContainer.positionManager.reefLevelToHeight(2)),
                0,
                RobotContainer.positionManager.calculateWristAngle(2)
            );
        }
        
        @Override
        public void end(boolean interrupted) {
            RobotContainer.isAutomaticDriveMode = false;
        }

        @Override
        public boolean isFinished() {
            return timeout.isFinished() || (RobotContainer.positionManager.xSpeed <= 0.05 && RobotContainer.positionManager.ySpeed <= 0.05 && RobotContainer.positionManager.rSpeed <= 1);
        }
    };



    public static Command grabAlgae4 = new Command() {
        WaitCommand wait = new WaitCommand(1);

        @Override
        public void initialize() {
            RobotContainer.manipulatorCommand.state = 4;
            wait.schedule();
        }

        @Override
        public void execute() {
            RobotContainer.drivetrain.setControl(
                DriveModes.driveRobot
                    .withVelocityX(0.7)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            );
        }

        @Override
        public boolean isFinished() {
            return wait.isFinished();
        }
    };



    public static Command armUpAlgae = new Command() {
        WaitCommand wait = new WaitCommand(1);

        @Override
        public void initialize() {
            wait.schedule();
            
            RobotContainer.positionManager.setReefTarget(false, 4, 4, true);
            RobotContainer.positionManager.updatePositions(
                RobotContainer.positionManager.calculateArmPivot(4),
                RobotContainer.positionManager.calculateHeight(RobotContainer.positionManager.calculateArmPivot(4),
                RobotContainer.positionManager.reefLevelToHeight(4)),
                0,
                RobotContainer.positionManager.calculateWristAngle(4)
            );
        }

        @Override
        public boolean isFinished() {
            return wait.isFinished();
        }
    };



    public static Command bargeAlgae = new Command() {
        WaitCommand shootWait = new WaitCommand(0.5);

        @Override
        public void initialize() {
            shootWait.schedule();
            RobotContainer.manipulatorCommand.state = 5;
        }

        @Override
        public void end(boolean interrupted) {
            RobotContainer.manipulatorCommand.state = 0;
        }

        @Override
        public boolean isFinished() {
            return shootWait.isFinished();
        }
    };



    public static Command endPosition = new Command() {
        @Override
        public void initialize() {
            RobotContainer.positionManager.updatePositions(
                Math.toRadians(170),
                0,
                0,
                0
            );
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    };
}
