// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.net.Socket;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.driveConstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.PositionManagerCommand;
// import frc.robot.commands.IntakeCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DriveModes;
// import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.PositionManager;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision.Vision;

public class RobotContainer {

    /* Setting up bindings for necessary control of the swerve drive platform */
    // private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    //         .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    // private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public static final Telemetry logger = new Telemetry(driveConstants.MaxSpeed);

    // Xbox variables
    public static CommandXboxController joystick = new CommandXboxController(0); 
    public static XboxController xbox1 = new XboxController(0);
    public static XboxController xbox2 = new XboxController(1);
    public static XboxController xbox3 = new XboxController(2); // Used for PID testing only

    // Arduino "Controller"
    public static GenericHID arduino = new GenericHID(2);
  
    // Drivetrain
    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Intake variables
    // public static Intake intake = new Intake();
    // public static IntakeCommand intakeCommand = new IntakeCommand();
    // public static SparkMax leftMotor = new SparkMax(27, MotorType.kBrushless);
    // public static SparkMax rightMotor = new SparkMax(26, MotorType.kBrushless);
    // public static SparkMax intakePivotMotor = new SparkMax(24, MotorType.kBrushless);
    // public static DutyCycleEncoder intakeEncoder = new DutyCycleEncoder(2);

    // Arm variables
    public static Arm arm = new Arm();
    public static ArmCommand armCommand = new ArmCommand();
    public static SparkMax extendMotor = new SparkMax(21, MotorType.kBrushless);
    public static SparkMax rotateMotor = new SparkMax(22, MotorType.kBrushless);
    public static DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0);
    
    // Manipulator
    public static Manipulator manipulator = new Manipulator();
    // public static SparkMax manipulatorIntakeMotor = new SparkMax(0, MotorType.kBrushless); //TODO add the right motor ids
    // public static SparkMax wristMotor = new SparkMax(0, MotorType.kBrushless);

    // Position Manager
    public static PositionManager positionManager = new PositionManager();
    public static PositionManagerCommand positionManagerCommand = new PositionManagerCommand();
    
    // Climber variables
    
    public static Climber climber = new Climber();
    // public static SparkMax climberMotor = new SparkMax(25, MotorType.kBrushless);

    // Elevator variables
    
    public static Elevator elevator = new Elevator();
    public static ElevatorCommand elevatorCommand = new ElevatorCommand();
    public static SparkMax upAndDownMotor = new SparkMax(20, MotorType.kBrushless);
    public static SparkMax side2SideMotor = new SparkMax(23, MotorType.kBrushless);
    public static DutyCycleEncoder elevatorEncoder = new DutyCycleEncoder(1);


    // Camera 
    public final static PhotonCamera camera = new PhotonCamera("PC_Camera");
    // public static DigitalInput manipulatorProxSensor = new DigitalInput(0);

    public static final Vision vision = new Vision();

    // Position Manager
    public static final boolean isAutomaticPositioningMode = false;


    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        elevator.setDefaultCommand(elevatorCommand);
        arm.setDefaultCommand(armCommand);
        // intake.setDefaultCommand(intakeCommand);
        positionManager.setDefaultCommand(positionManagerCommand);


        configureBindings();
    }
    
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    private void configureBindings() {
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                DriveModes.driveField
                    .withVelocityX(-Sine(joystick.getLeftX(), joystick.getLeftY()) * driveConstants.MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-Cosine(joystick.getLeftX(), joystick.getLeftY()) * driveConstants.MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-Math.pow(Deadzone(joystick.getRightX()), driveConstants.Linearity) * driveConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.leftBumper().whileTrue(drivetrain.applyRequest(() -> 
            DriveModes.driveRobot
                .withVelocityX(-Sine(RobotContainer.joystick.getLeftX(), RobotContainer.joystick.getLeftY()) * driveConstants.MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-Cosine(RobotContainer.joystick.getLeftX(), RobotContainer.joystick.getLeftY()) * driveConstants.MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-Math.pow(Deadzone(RobotContainer.joystick.getRightX()), driveConstants.Linearity) * driveConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> DriveModes.brake));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // What is this? We don't know! 2/6/25
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.povDown().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    /** Function that returns a given speed, as long as it is above the deadzone set in Constants. */
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

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
