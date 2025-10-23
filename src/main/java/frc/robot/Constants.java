// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Vision.TagLayouts;


/** Add your docs here. */
public class Constants {

    public final static double centerToArm = 5.25;

    public static final class driveConstants {

        public final static double MaxSpeed = 2.5; //4 -> 3.5 -> 2.5 //TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public final static double MaxAngularRate = RotationsPerSecond.of(Math.PI / 4).in(RadiansPerSecond); // 2 rotations per second max angular velocity

        // public final static double MaxSpeed = 5; //Max targeted speed in M/S (15 NORMALLY)
        // public final static double MaxAngularRate = 1.5 * Math.PI * 3; //Max targeted rotations / second -- 3/4ths of a rotation for now
        public final static double MaxAcceleration = 1; //2 -> 1.5 -> 1  //Max acceleration in M/s/s;
        public final static double MaxAngularAcceleration = 4 * Math.PI;
        public final static double SpeedDeadbandPercentage = 0; //Deadband or Deadzone of requested speed, as a percentage of the maximum speed;

        public final static double Linearity = 3; // How steep the response curve is (typically cubic, meaning 50% on stick = 12.5% speed)

        public final static double ControllerDeadzone = 0.15;
        public final static double Controller2Deadzone = 0.3;

        public final static PIDController chassisPID = new PIDController(0.01, 0, 0);
        public final static double ChassisPidTolerence = Math.PI/180;

        public final static double ChassisModulePosX = 13.375; //Middle of robot to module left/right position
        public final static double ChassisModulePosY = 10.375; //Middle of robot to module front/back position

    }

    public static final class intakeConstants {
        
        public final static double IntakePIDkp = 2; // Proportion (was 1)
        public final static double IntakePIDki = 0; // Integral 
        public final static double IntakePIDkd = 0.01; // Derivative

        public final static double PivotGearRatio = 1/25d; // Motor rotations to get one output rotation
        public final static double PivotEncoderOffset = 0.0595; // Rotations; offset of encoder at level (near down position)
        public final static double PivotAngleUp = Math.toRadians(120); // Radians; Inside frame position
        public final static double PivotAngleAlgae = Math.toRadians(105); // Radians; Algae grabbing poisition
        public final static double PivotAngleFloor = Math.toRadians(95); // Radians; Pickup position
        public final static double PivotMinAngle = Math.toRadians(90); // Radians; Bottom limit
        public final static double PivotMaxAngle = Math.toRadians(180); // Radians; Upper limit

        public static double LeftSpeed = 0.9;
        public static double RightSpeed = -0.8;
        public final static double LeftReverseSpeed = -0.1;
        public final static double RightReverseSpeed = 0.1;

    }

    public static final class elevatorConstants {
      
        public final static double ArmRightLimit = -6.5; // Inches; -Y
        public final static double ArmLeftLimit = 3; // Inches; +Y

        public final static double ArmTopLimit = 63.75; // Was 59.5
        public final static double ArmBottomLimit = 10.5;

        public final static double Side2SideSpeedFactor = 1;
        public final static double UpAndDownSpeedFactor = 0.4;

        public final static double UpAndDownOffset = 0.059; //Rotations at the lowest point
        public final static double PivotToFloorOffset = 9.375; //Inches from floor to pivot point at lowest position
        public final static double InchesPerRotation = 10.95; // Was 11.3125 

        public final static double Side2SideOffset = 0; // Inches; offset if starting from center

        //Drive speed limiting
        public final static double SpeedLimitHeight = 17; //Inches; point at which speed should start being limited
        public final static double MinSpeed = 1.25; //Meters per second; drive speed at max elevator extension

    }

    
    public static final class armConstants {

        public final static double PivotToCoral = 22.5; // Inches; distance from pivot point to the coral piece
        public final static double VerticalCoralOffset = 7; // Inches; how much clearance to add for the coral
        public final static double CenterToPivot = 0.13335; // METERS; distance from imaginary center of drive base to pivot point

        ///// UNUSED \\\\\
        public final static double PivotToMinExtend = 15.8; // WARNING - NO LONGER USED! // Inches; Distance from pivot point to end of pivot block (not manipulator)
        public final static double MinExtendToCoral = 12.75; // WARNING - NO LONGER USED! //29 - PivotToMinExtend; // Inches; Distance from end of pivot block to coral on manipulator
        public final static double ExtendForwardLimit = 0.25 + PivotToMinExtend + MinExtendToCoral; //WASWAS 6.8 + [...] // WAS 22.8 // 24.5 Actual, Inches; Maximum extention
        public final static double ExtendBackwardLimit = 0.25 + PivotToMinExtend + MinExtendToCoral; // WAS 16 //Inches; Minimum extention
        ///// ------ \\\\\

        public final static double RotateUpLimit = Math.toRadians(165); //Hard limits (Was 170)
        public final static double RotateDownLimit = Math.toRadians(55); //Hard Limits (Was 10)
        public final static double RotateEncoderOffset = 0.459; //Offset in rotations at level (90 degrees)
        
        public final static double ExtendPIDkp = 0.1;
        public final static double ExtendPIDki = 0;
        public final static double ExtendPIDkd = 0.002;
        
        public final static double RotatePIDkp = 0.4; // Was 0.5 before spring added
        public final static double RotatePIDki = 0;
        public final static double RotatePIDkd = 0;
        
        public final static double ExtendSpeedFactor = 0; //Was 0.3
        public final static double RotateSpeedFactor = 0.2; // Was 0.1
    }

    public static final class manipulatorConstants {

        // public final static double manipulatorIntakeSpeed = 0.3;
        // public final static double manipulatorOutakeSpeed = -0.3;

        public final static double CoralIntakeSpeed = 0.45;
        public final static double CoralIntakeFloorSpeed = 0.7;
        public final static double CoralPlaceSpeed = 0.3;
        public final static double AlgaeIntakeSpeed = 0.3;
        public final static double AlgaeShootSpeed = -0.75;
        public final static double AlgaeHoldSpeed = 0.6;
        public final static double IntakeReverseSpeed = -0.3;

        public final static double WristPIDkp = 0.2;
        public final static double WristPIDki = 0;
        public final static double WristPIDkd = 0;
        
        public final static double WristRightLimit = Math.toRadians(90); // Radians; Right rotational limit
        public final static double WristLeftLimit = Math.toRadians(0); // Radians; Left rotational limit

        public final static double WristSpeedFactor = 0.2;
    }
    
    public static final class visionConstants {
        
        public final static double AngleDistrust = 10; // UNUSED - How much the angle should impact vision trust

        public static final AprilTagFieldLayout TagLayout = TagLayouts.getTagLayoutFromPath("apriltagLayouts/onlyReef.json");

        public final static Matrix<N3, N1> VisionStandardDeviationSingleTag = VecBuilder.fill(2,2,4);
        public final static Matrix<N3, N1> VisionStandardDeviationMultiTag = VecBuilder.fill(0.5,0.5,1);

        // public final static Transform3d FrontRightM1CamOffset = new Transform3d(0.12065, -0.3683, 0.301625, new Rotation3d(Math.toRadians(90), 0, Math.toRadians(42.04))); // Old Mount (in between M1 and M4)
        // public final static Transform3d FrontRightM1CamOffset = new Transform3d(0.2852, -0.1328, 0.301625, new Rotation3d(Math.toRadians(-90), Math.toRadians(16.7), Math.toRadians(14))); // Other mount (front of robot)
        public final static Transform3d FrontRightM1CamOffset = new Transform3d(0.1334, -0.3620, 0.301625, new Rotation3d(0, 0, Math.toRadians(33.5)));
        // public final static Transform3d FrontLeftM2CamOffset = new Transform3d(0.2259, 0.1287, 0.301625, new Rotation3d(Math.toRadians(-15.3), Math.toRadians(30), Math.toRadians(-28.7)));
        // public final static Transform3d FrontLeftM2CamOffset = new Transform3d(0.1249, -0.138, 0.301625, new Rotation3d(0, Math.toRadians(20), Math.toRadians(15)));
        public final static Transform3d BackRightM4CamOffset = new Transform3d(-0.3112, -0.3556, 0.301625, new Rotation3d(0, 0, Math.toRadians(-169)));
        
    }
    
    public static final class climberConstants {
        
        public final static double MaxPosition = 240;
        public final static double MinPosition = 15;

        public final static double flipperMotorPIDkp = 0.2;
        public final static double flipperAngle = Math.toRadians(80);

    }

    public static final class fieldConstants {

        // Optimal Drive Distances: Meters
        public final static double DriveL1X = 0.85; // Meters; optimal x distance away from the tag
        public final static double DriveL2andL3X = 0.5875; // Meters; optimal x distance away from the tag
        public final static double DriveL4X = 0.6; // Meters; optimal x distance away from the tag
        public final static double DriveAlgaeX = 1; // Meters; optimal x distance away from the tag
        public final static double DriveCoralX = 0.65;
        public final static double DriveLeftY = 0.1 + Units.inchesToMeters(elevatorConstants.ArmLeftLimit); // Meters; optimal Y distance from tag for left pole
        public final static double DriveRightY = 0 + Units.inchesToMeters(elevatorConstants.ArmRightLimit); // Meters; optimal Y distance from tag for left pole

        public final static double DriveMinAutoSpeedX = 0.1;
        public final static double DriveMinAutoSpeedY = 0.05;
        public final static double DriveminAutoSpeedR = 0.1;
        public final static double DriveMaxAutoSpeed = 2;
        public final static double DrivekP = 6; // Was 8
        public final static double RotatekP = 7;

        public final static double PathfindOffset = 1; // Meters; how far away from the tag Pathfind should aim for

        //Reef level angles
        public final static double L1Angle = 100;
        public final static double L2Angle = 150;
        public final static double L3Angle = 150;
        public final static double L4Angle = 150;
        public final static double AlgaeAngle = 80;
        public final static double CoralStationAngle = 165;
        public final static double ProcessorAngle = 115;
        public final static double BargeShootAngle = 160;
        public final static double FloorPickupAngle = 58;
        public final static double AlgaeFloorAngle = 55;
        public final static double ErrorAngle = 145;

        //Reef left-right length
        public final static double ReefSideToSide = 6.5; // Inches

        // All are in inches
        public final static double TagToReefYOffset = 6.497;
        public final static double TagToL2and3XOffset = 2.0786;
        public final static double TagToL4XOffset = 2.0498;

        public final static double L1Height = 22; // Was 18
        public final static double L2Height = 32; // Was 31.875
        public final static double L3Height = 47; // Was 47.75
        public final static double L4Height = 100; // Was 71.75
        public final static double AlgaeL2Height = 32;
        public final static double AlgaeL3Height = 47;
        public final static double CoralStationHeight = 27; // Bottom of station is 37.5 inches
        public final static double BargeShootHeight = 100;
        public final static double ProcessorHeight = 0;
        public final static double FloorPickupHeight = 1.75;
        public final static double AlgaeFloorHeight = 8;

      
    }
}
