// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj.DriverStation;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.commands.*;
import com.pathplanner.lib.auto.AutoBuilder;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.subsystems.ArmSubsystem;
import java.io.IOException;
import org.json.simple.parser.ParseException;



public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.25).in(RadiansPerSecond); // 1/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.065) // Add a 6.5% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed); 

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ArmSubsystem arm = new ArmSubsystem();

    // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
    PathConstraints constraints = new PathConstraints(
        3.0, 3.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

    public RobotContainer() {
        if (DriverStation.getJoystickIsXbox(0)) {
            configureBindingsXBox();
        } else {
            configureBindingsPS4();
        }

        // Use to warmup the library (Yes, this is needed in Java)
        PathfindingCommand.warmupCommand().schedule();
    }

    private void configureBindingsXBox() {

        final CommandXboxController joystick = new CommandXboxController(0);

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed / 4) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed / 4) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate / 4) // Drive counterclockwise with negative X (left)
            )
        );

        // Brake while the robot is disabled. This moves the wheels into
        // an 'x' arrangment to resist all force from all angles
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> brake).ignoringDisable(true)
        );

        // While 'A' is pressed, make the robot brake.
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // Reset the field-centric heading on Right bumper press
        joystick.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        joystick.x().onTrue(arm.runOnce(() -> arm.goToAngle(20)));
        joystick.y().onTrue(arm.runOnce(() -> arm.goToAngle(75)));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureBindingsPS4() {

        final CommandPS4Controller joystick = new CommandPS4Controller(0);

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getRawAxis(1) * MaxSpeed / 4) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getRawAxis(0) * MaxSpeed / 4) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRawAxis(2) * MaxAngularRate / 4) // Drive counterclockwise with negative X (left)
            )
        );

        // Brake while the robot is disabled. This moves the wheels into
        // an 'x' arrangment to resist all force from all angles 
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> brake).ignoringDisable(true)
        );

        // While "Cross" is pressed, make the robot brake.
        joystick.cross().whileTrue(drivetrain.applyRequest(() -> brake));

        // Reset the field-centric heading on R1 press
        joystick.R1().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        joystick.triangle().onTrue(arm.runOnce(() -> arm.goToAngle(20)));
        joystick.square().onTrue(arm.runOnce(() -> arm.goToAngle(75)));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Load the path we want to pathfind to and follow
    PathPlannerPath getCoral;   
    PathPlannerPath placeCoral;
    try {
        getCoral = PathPlannerPath.fromPathFile("getCoral");
        placeCoral = PathPlannerPath.fromPathFile("placeCoral");
    } catch (IOException | ParseException e) {
        e.printStackTrace();
        return null; // Return null or handle the error appropriately
    }
    
    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command getCoralCommand = AutoBuilder.pathfindThenFollowPath(
        getCoral,
        constraints);

    Command placeCoralCommand = AutoBuilder.pathfindThenFollowPath(
        placeCoral, 
        constraints);

    SequentialCommandGroup command = new SequentialCommandGroup(getCoralCommand, placeCoralCommand);

    return command;
    }
}
