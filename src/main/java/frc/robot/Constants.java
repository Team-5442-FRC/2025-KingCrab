// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.generated.TunerConstants;


/** Add your docs here. */
public class Constants {

    public final static double centerToArm = 5.25;

    public static final class driveConstants {

        public final static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public final static double MaxAngularRate = RotationsPerSecond.of(4 * Math.PI).in(RadiansPerSecond); // 2 rotations per second max angular velocity

        // public final static double MaxSpeed = 5; //Max targeted speed in M/S (15 NORMALLY)
        // public final static double MaxAngularRate = 1.5 * Math.PI * 3; //Max targeted rotations / second -- 3/4ths of a rotation for now
        public final static double MaxAcceleration = 2; //Max acceleration in M/s/s;
        public final static double MaxAngularAcceleration = 4 * Math.PI;
        public final static double SpeedDeadbandPercentage = 0; //Deadband or Deadzone of requested speed, as a percentage of the maximum speed;

        public final static double Linearity = 3; // How steep the response curve is (typically cubic, meaning 50% on stick = 12.5% speed)

        public final static double ControllerDeadzone = 0.15;

        public final static PIDController chassisPID = new PIDController(0.01, 0, 0);
        public final static double ChassisPidTolerence = Math.PI/180;

        public final static double ChassisModulePosX = 13.375; //Middle of robot to module left/right position
        public final static double ChassisModulePosY = 10.375; //Middle of robot to module front/back position

    }

    public static final class intakeConstants {

        public final static double MinAngle = 0; // Degrees
        public final static double MaxAngle = 90; // Degrees
        
        public final static double IntakePIDkp = 0.00001; // Proportion
        public final static double IntakePIDki = 0; // Integral 
        public final static double IntakePIDkd = 0; // Derivative

        public final static double PivotGearRatio = 25; // Motor rotations to get one output rotation
        public final static double PivotAngleUp = 90;
        public final static double PivotAngleAlgae = 15;
        public final static double PivotAngleFloor = 0;

        public final static double LeftSpeed = 0.5;
        public final static double RightSpeed = 0.8;
        public final static double LeftReverseSpeed = -0.1;
        public final static double RightReverseSpeed = -0.1;

    }

    public static final class elevatorConstants {
      
        public final static double ArmRightLimit = 0; //TODO add correct limit y-axis
        public final static double ArmLeftLimit = 0; //TODO add correct limit y-axis

        public final static double ArmTopLimit = 0; //TODO add correct limit z-axis
        public final static double ArmBottomLimit = 0; //TODO add correct limit z-axis

        public final static double Side2SideSpeedFactor = 1;
        public final static double UpAndDownSpeedFactor = 1;
      
    }

    public static final class armConstants {
        public final static double ExtendForwardLimit = 0; //TODO add the right limits
        public final static double ExtendBackwardLimit = 0; //TODO add the right limits

        public final static double RotateUpLimit = 170; //Hard limits TODO add the right limits
        public final static double RotateDownLimit = 10; //Hard Limits TODO add the right limits

        public final static double ExtendPIDkp = 0.00001;
        public final static double ExtendPIDki = 0;
        public final static double ExtendPIDkd = 0;

        public final static double RotatePIDkp = 0.00001;
        public final static double RotatePIDki = 0;
        public final static double RotatePIDkd = 0;

        public final static double ExtendSpeedFactor = 1;
        public final static double RotateSpeedFactor = 1;
    }

    public static final class visionConstants {

        public final static double AngleDistrust = 10; // How much the angle should impact vision trust
        public final static Transform3d MicrosoftCameraOffset = new Transform3d(0, 0, 0, new Rotation3d(0,0,0));
        public final static Transform3d ThriftyCameraOffset   = new Transform3d(0, 0, 0, new Rotation3d(0,0,0));
        public final static Transform3d GenoCameraOffset      = new Transform3d(0, 0, 0, new Rotation3d(0,0,0));
        public final static Transform3d LimelightCameraOffset = new Transform3d(0, 0, 0, new Rotation3d(0,0,0)); // Probably not needed
    
    }

    public static final class climberConstants {

        public final static double MaxPosition = 1;
        public final static double MinPosition = -1;
    }
}
