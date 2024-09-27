// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.limelight.LimelightAimCommand;
import frc.robot.commands.ShooterSpinUpCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.AngleSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.shooter.ShooterBottomSubsystem;
import frc.robot.subsystems.shooter.ShooterTopSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController drivingController = new CommandXboxController(
            OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController subsystemController = new CommandXboxController(
            OperatorConstants.SUBSYSTEM_CONTROLLER_PORT);

    private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final AngleSubsystem angleSubsystem = new AngleSubsystem();
    private final FeederSubsystem feederSubsystem = new FeederSubsystem();
    private final ShooterTopSubsystem shooterTopSubsystem = new ShooterTopSubsystem();
    private final ShooterBottomSubsystem shooterBottomSubsystem = new ShooterBottomSubsystem();
    private final ClimberSubsystem climberLeftSubsystem = new ClimberSubsystem("left climber",
            Constants.CLIMBER_MOTOR_LEFT_ID);
    private final ClimberSubsystem climberRightSubsystem = new ClimberSubsystem("right climber",
            Constants.CLIMBER_MOTOR_RIGHT_ID);
    private final DrivetrainSubsystem drivetrain = TunerConstants.DRIVE_TRAIN;

    public static Supplier<Boolean> getFieldCentric = () -> true;
    private final Telemetry telemetry = new Telemetry(.1);

    private double slowMultiplier = DrivetrainConstants.SLOW_SPEED;
    private double normalMultiplier = DrivetrainConstants.NORMAL_SPEED;
    private double turboMultiplier = DrivetrainConstants.TURBO_SPEED;

    private final SendableChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and
     * commands.
     */
    public RobotContainer() {
        drivetrain.registerTelemetry(telemetry::telemeterize);

        AutoCommands.registerCommands(intakeSubsystem, feederSubsystem, shooterTopSubsystem,
                shooterBottomSubsystem);
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("pick an auto", autoChooser);

        configureDriverBindings();
    }

    private static double deadband(double input) {
        if (Math.abs(input) <= DrivetrainConstants.DEADBAND) {
            return 0;
        }
        return input;
    }

    /**
     * Use this method to define your trigger → command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureDriverBindings() {
        // Outtake and intake, respectively
        drivingController.leftBumper().whileTrue(new ParallelCommandGroup(
                intakeSubsystem.pushStopCommand(),
                feederSubsystem.pushStopCommand()));
        drivingController.rightBumper()
                .whileTrue(Compositions.feedNIn(feederSubsystem, intakeSubsystem));

        // Drive the robot
        drivetrain.setDefaultCommand(
                new DriveCommand(drivetrain,
                        () -> -deadband(drivingController.getLeftX()),
                        () -> -deadband(drivingController.getLeftY()),
                        () -> {
                            return deadband(
                                    drivingController.getLeftTriggerAxis()
                                            - drivingController
                                                    .getRightTriggerAxis());
                        },
                        () -> {
                            return drivingController.getHID().getRightStickButton()
                                    ? turboMultiplier
                                    : (drivingController.getHID()
                                            .getLeftStickButton()
                                                    ? slowMultiplier
                                                    : normalMultiplier);
                        }));

        // Intake, feed and shoot- all in one button!
        drivingController.a().whileTrue(
                Compositions.feedAndShootAlsoIntake(
                        feederSubsystem, intakeSubsystem, shooterTopSubsystem,
                        shooterBottomSubsystem,
                        SmartDashboard.getNumber("shootTopRPM",
                                ShooterConstants.TOP_DEFAULT_RPM),
                        SmartDashboard.getNumber("shootBottomRPM",
                                ShooterConstants.BOTTOM_DEFAULT_RPM)));

        // Aim the robot (never worked :c)
        drivingController.x().whileTrue(
                new LimelightAimCommand(limelightSubsystem, drivetrain, angleSubsystem));

        // Reset the robot so that it understands that where it's currently facing is
        // "forwards"
        drivingController.y().onTrue(Commands.runOnce(() -> {
            drivetrain.seedFieldRelative();
            drivetrain.getPigeon2().setYaw(0);
        }, drivetrain));

        // Angle the angle changer straight up
        drivingController.povUp().onTrue(
                Compositions.angleUpdateWithIntake(angleSubsystem.angleToMax(), angleSubsystem,
                        intakeSubsystem));
        // Reset the angle changer to its base position
        drivingController.povDown().onTrue(
                angleSubsystem.angleToBase());

        // Shoot the NOTE (I think) (why is this here again?)
        drivingController.povRight()
                .whileTrue(new ParallelCommandGroup(
                        angleSubsystem.setSmartDashboardCommand(),
                        new SequentialCommandGroup(
                                new ShooterSpinUpCommand(
                                        shooterTopSubsystem, shooterBottomSubsystem,
                                        ShooterConstants.TOP_DEFAULT_RPM,
                                        ShooterConstants.BOTTOM_DEFAULT_RPM),
                                new ParallelDeadlineGroup(
                                        feederSubsystem.pullStopCommand(),
                                        Compositions.shootersHoldNStop(shooterTopSubsystem,
                                                shooterBottomSubsystem)))));

    }

    void configureSubsystemBindings() {
        subsystemController.a().whileTrue(
                Compositions.feedAndShootAlsoIntake(
                        feederSubsystem, intakeSubsystem, shooterTopSubsystem,
                        shooterBottomSubsystem,
                        SmartDashboard.getNumber("shootTopRPM",
                                ShooterConstants.TOP_DEFAULT_RPM),
                        SmartDashboard.getNumber("shootBottomRPM",
                                ShooterConstants.BOTTOM_DEFAULT_RPM)));
        subsystemController.x().whileTrue(new ParallelCommandGroup(
                intakeSubsystem.pushStopCommand(),
                feederSubsystem.pushStopCommand()));
        subsystemController.y().whileTrue(Compositions.shootNAngleFromStageBack(
                angleSubsystem, shooterTopSubsystem, shooterBottomSubsystem, feederSubsystem,
                intakeSubsystem));

        subsystemController.leftBumper().whileTrue(climberLeftSubsystem.upStopCommand());
        subsystemController.rightBumper().whileTrue(climberRightSubsystem.upStopCommand());
        subsystemController.leftTrigger(0.1).whileTrue(climberLeftSubsystem.downStopCommand());
        subsystemController.rightTrigger(0.1).whileTrue(climberRightSubsystem.downStopCommand());

        angleSubsystem.setDefaultCommand(angleSubsystem.anglePIDCommand(angleSubsystem));
        shooterTopSubsystem.setDefaultCommand(shooterTopSubsystem.shootPIDCommand());
        shooterBottomSubsystem
                .setDefaultCommand(shooterBottomSubsystem.shootPIDCommand());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        Command auto = autoChooser.getSelected();
        if (auto == null) {
            System.out.println("auto is null");
            return null;
        }

        // we need to get the starting pose from the Limelight
        return auto;
    }

    public void resetSetpoints() {
        shooterTopSubsystem.setSetpoint(0);
        shooterBottomSubsystem.setSetpoint(0);
        angleSubsystem.setSetpoint(235);
    }
}
