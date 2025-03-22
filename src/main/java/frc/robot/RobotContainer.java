// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.driveConstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ManipulatorCommand;
import frc.robot.commands.PositionManagerCommand;
import frc.robot.commands.autoCommands.AutoCommands;
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
    public static double driveSpeed = driveConstants.MaxSpeed;

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
    // public static SparkMax extendMotor = new SparkMax(21, MotorType.kBrushless);
    public static SparkMax rotateMotor = new SparkMax(22, MotorType.kBrushless);
    public static DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0);
    
    // Manipulator
    public static Manipulator manipulator = new Manipulator();
    public static ManipulatorCommand manipulatorCommand = new ManipulatorCommand(); // Try to say that five times fast
    public static TalonFX manipulatorIntakeMotor = new TalonFX(26);
    public static SparkMax wristMotor = new SparkMax(21, MotorType.kBrushless);
    public static DigitalInput manipulatorProxSensor = new DigitalInput(8);

    // Position Manager
    public static PositionManager positionManager = new PositionManager();
    public static PositionManagerCommand positionManagerCommand = new PositionManagerCommand();
    public static boolean isAutomaticPositioningMode = false;
    public static boolean isAutomaticDriveMode = false;

    
    // Climber variables
    public static Climber climber = new Climber();
    public static ClimberCommand climberCommand = new ClimberCommand();
    public static SparkMax climberMotor = new SparkMax(25, MotorType.kBrushless);
    public static Servo climberServo = new Servo(9);

    // Elevator variables
    public static Elevator elevator = new Elevator();
    public static ElevatorCommand elevatorCommand = new ElevatorCommand();
    public static SparkMax upAndDownMotor = new SparkMax(20, MotorType.kBrushless);
    // public static SparkMax side2SideMotor = new SparkMax(23, MotorType.kBrushless);
    public static DutyCycleEncoder elevatorEncoder = new DutyCycleEncoder(1);

    // Vision
    public static Vision vision = new Vision();
    public static Notifier visionThread = new Notifier(vision);

    // Autonomous
    public static Command TEST = AutoCommands.test;
    public static Command ArmUpReef = AutoCommands.armUpReef4R4;
    public static Command PlaceReef = AutoCommands.placeReef4R4;
    public static Command DropR4 = AutoCommands.dropOnReefR4;
    public static Command BackUp = AutoCommands.backUpR4;
    public static Command PositionAlgae = AutoCommands.positionAlgae4;
    public static Command GrabAlgae = AutoCommands.grabAlgae4;
    public static Command ArmUpAlgae = AutoCommands.armUpAlgae;
    public static Command BargeAlgae = AutoCommands.bargeAlgae;
    public static Command EndPosition = AutoCommands.endPosition;

    // Other?
    static SlewRateLimiter xSlew = new SlewRateLimiter(2);
    static SlewRateLimiter ySlew = new SlewRateLimiter(2);
    static SlewRateLimiter rSlew = new SlewRateLimiter(4);

    public static Command autoDriveCommand = new Command() {
        @Override
        public void execute() {
            RobotContainer.drivetrain.setControl( // go nyoom
                DriveModes.driveRobot
                    .withVelocityX(xSlew.calculate(positionManager.xSpeed))
                    .withVelocityY(ySlew.calculate(positionManager.ySpeed))
                    .withRotationalRate(rSlew.calculate(positionManager.rSpeed))
            );
        }

        @Override
        public boolean isFinished() {
            return !isAutomaticDriveMode;
        }
    };

    public static boolean hasFieldOriented = false;
    // public static Trigger autoDriveToTag = new Trigger(new BooleanSupplier() {
    //     @Override
    //     public boolean getAsBoolean() {
    //         return isAutomaticDriveMode;
    //     };
    // });

    public static void disableDefaultCommand() {
        drivetrain.setDefaultCommand(null);
    }
    public static void enableDefaultCommand() {
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                DriveModes.driveField
                    .withVelocityX(-Sine(joystick.getLeftX(), joystick.getLeftY()) * driveSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-Cosine(joystick.getLeftX(), joystick.getLeftY()) * driveSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-Math.pow(Deadzone(joystick.getRightX()), driveConstants.Linearity) * driveConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
    }


    /* Path follower */
    private final SendableChooser<Command> autoChooser;
    // private final SendableChooser<String> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        elevator.setDefaultCommand(elevatorCommand);
        arm.setDefaultCommand(armCommand);
        manipulator.setDefaultCommand(manipulatorCommand);
        // intake.setDefaultCommand(intakeCommand);
        climber.setDefaultCommand(climberCommand);
        positionManager.setDefaultCommand(positionManagerCommand);

        // Named commands must be placed before autochooser!
        NamedCommands.registerCommand("TEST", TEST);
        NamedCommands.registerCommand("Arm Up Reef", ArmUpReef);
        NamedCommands.registerCommand("Place Reef", PlaceReef);
        NamedCommands.registerCommand("Drop R4", DropR4);
        NamedCommands.registerCommand("Back Up", BackUp);
        NamedCommands.registerCommand("Position Algae", PositionAlgae);
        NamedCommands.registerCommand("Grab Algae", GrabAlgae);
        NamedCommands.registerCommand("Arm Up Algae", ArmUpAlgae);
        NamedCommands.registerCommand("Barge Algae", BargeAlgae);
        NamedCommands.registerCommand("End Position", EndPosition);

        autoChooser = AutoBuilder.buildAutoChooser("Blue Center");
        // autoChooser.addOption("Blue Center", "Blue Center");
        // autoChooser.addOption("TEST", "TEST");
        SmartDashboard.putData("Auto Mode", autoChooser);

        visionThread.startPeriodic(0.05);

        configureBindings();
    }
    
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    private void configureBindings() {
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                DriveModes.driveField
                    .withVelocityX(-Sine(joystick.getLeftX(), joystick.getLeftY()) * driveSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-Cosine(joystick.getLeftX(), joystick.getLeftY()) * driveSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-Math.pow(Deadzone(joystick.getRightX()), driveConstants.Linearity) * driveConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.leftBumper().whileTrue(drivetrain.applyRequest(() -> 
            DriveModes.driveRobot
                .withVelocityX(-Sine(RobotContainer.joystick.getLeftX(), RobotContainer.joystick.getLeftY()) * driveSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-Cosine(RobotContainer.joystick.getLeftX(), RobotContainer.joystick.getLeftY()) * driveSpeed) // Drive left with negative X (left)
                .withRotationalRate(-Math.pow(Deadzone(RobotContainer.joystick.getRightX()), driveConstants.Linearity) * driveConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> DriveModes.brake));

        // joystick.rightBumper().whileTrue(drivetrain.applyRequest(() ->
        //     DriveModes.driveRobot
        //         .withVelocityX(positionManager.xSpeed)
        //         .withVelocityY(positionManager.ySpeed)
        //         .withRotationalRate(positionManager.rSpeed)
        // ));

        // autoDriveToTag.whileTrue(drivetrain.applyRequest(() ->
        //     DriveModes.driveRobot
        //         .withVelocityX(positionManager.xSpeed)
        //         .withVelocityY(positionManager.ySpeed)
        //         .withRotationalRate(positionManager.rSpeed)
        // ));
        
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

    /** Function that returns a given speed, as long as it is above the given deadzone. */
    public static double Deadzone(double speed, double minValue) {
        if (Math.abs(speed) > minValue) return speed;
        return 0;
    }

    ///// Controller Y value curving \\\\\
    public static double Cosine(double x, double y) {
        return Math.pow(Deadzone(Math.sqrt((x*x)+(y*y))), driveConstants.Linearity) * Math.cos(Math.atan2(y,x));
    }
    public static double Cosine(double x, double y, double exp) {
        return Math.pow(Math.sqrt((x*x)+(y*y)), exp) * Math.cos(Math.atan2(y,x));
    }

    ///// Controller X value curving \\\\\
    public static double Sine(double x, double y) {
        return Math.pow(Deadzone(Math.sqrt((x*x)+(y*y))), driveConstants.Linearity) * Math.sin(Math.atan2(y,x));
    }
    public static double Sine(double x, double y, double exp) {
        return Math.pow(Math.sqrt((x*x)+(y*y)), exp) * Math.sin(Math.atan2(y,x));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        // return AutoBuilder.buildAuto(autoChooser.getSelected());
        return autoChooser.getSelected();
    }
}
