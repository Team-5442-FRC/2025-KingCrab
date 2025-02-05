// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import frc.robot.RobotContainer;
import frc.robot.Constants.driveConstants;

import com.ctre.phoenix6.swerve.SwerveRequest;

/** Add your docs here. */
public class DriveModes {
    public static final SwerveRequest.FieldCentric driveField = new SwerveRequest.FieldCentric()
            .withDeadband(driveConstants.MaxSpeed * 0.1)
            .withRotationalDeadband(driveConstants.MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
            // .withVelocityX(Sine(RobotContainer.joystick.getLeftX(), RobotContainer.joystick.getLeftY()) * driveConstants.MaxSpeed) // Drive forward with negative Y (forward)
            // .withVelocityY(Cosine(RobotContainer.joystick.getLeftX(), RobotContainer.joystick.getLeftY()) * driveConstants.MaxSpeed) // Drive left with negative X (left)
            // .withRotationalRate(-Math.pow(Deadzone(RobotContainer.joystick.getRightX()), driveConstants.Linearity) * driveConstants.MaxAngularRate); // Drive counterclockwise with negative X (left)
    
    public static final SwerveRequest.RobotCentric driveRobot = new SwerveRequest.RobotCentric()
            .withDeadband(driveConstants.MaxSpeed * 0.1)
            .withRotationalDeadband(driveConstants.MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
            .withVelocityX(Sine(RobotContainer.joystick.getLeftX(), RobotContainer.joystick.getLeftY()) * driveConstants.MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(Cosine(RobotContainer.joystick.getLeftX(), RobotContainer.joystick.getLeftY()) * driveConstants.MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-Math.pow(Deadzone(RobotContainer.joystick.getRightX()), driveConstants.Linearity) * driveConstants.MaxAngularRate); // Drive counterclockwise with negative X (left)
    
    public static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    ///// Controller Deadzone \\\\\
    public static double Deadzone(double speed) {
        if (Math.abs(speed) > driveConstants.ControllerDeadzone) return speed;
        return 0;
    }

    ///// Controller Y value curving \\\\\
    public static double Cosine(double x, double y) {
        x = Deadzone(x);
        y = Deadzone(y);
        return Math.pow(Math.sqrt((x*x)+(y*y)), driveConstants.Linearity) * Math.cos(Math.atan2(y,x));
    }

    ///// Controller X value curving \\\\\
    public static double Sine(double x, double y) {
        x = Deadzone(x);
        y = Deadzone(y);
        return Math.pow(Math.sqrt((x*x)+(y*y)), driveConstants.Linearity) * Math.sin(Math.atan2(y,x));
    }
    
}
