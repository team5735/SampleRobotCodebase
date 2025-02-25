package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ShooterSpinUpCommand;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.AngleSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * A collection of composition commands which don't have a clear subsystem they
 * belong to
 */
public class Compositions {
    /*
     * Creates and returns a new SequentialCommandGroup that first spins up the
     * shooter, that is to say it gets the shooter to full speed, and then has a
     * ParallelCommandGroup that feeds the NOTE in, and simultaneously keeps the
     * shooter at full speed.
     * 
     * <p>
     * This command does not run the intake until 0.5 seconds have passed AND the
     * shooter is at full speed.
     */
    static Command feedAndShootAlsoIntake(FeederSubsystem feeder, IntakeSubsystem intake,
            ShooterSubsystem shooterTop,
            ShooterSubsystem shooterBottom, double topRPM, double bottomRPM) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new ShooterSpinUpCommand(shooterTop, shooterBottom, topRPM, bottomRPM),
                        new WaitCommand(0.5)),
                new ParallelCommandGroup(
                        feeder.pullStopCommand(),
                        intake.pullStopCommand(),
                        shootersHoldNStop(shooterTop, shooterBottom)));
    }

    public static Command angleUpdateWithIntake(Command angleSetCommand, AngleSubsystem angler,
            IntakeSubsystem intake) {
        return (angler.isAtBase(angler.getMeasurement()))
                ? new ParallelCommandGroup(
                        angleSetCommand,
                        new ParallelDeadlineGroup(
                                new WaitCommand(2),
                                intake.pullStopCommand()))
                : angleSetCommand;
    }

    public static Command shootNAngleFromStageBack(AngleSubsystem angle, ShooterSubsystem top,
            ShooterSubsystem bottom, FeederSubsystem feeder, IntakeSubsystem intake) {
        return new SequentialCommandGroup(
                angle.angleToStageBack(),
                feedAndShootAlsoIntake(
                        feeder, intake, top, bottom, ShooterConstants.TOP_STAGE_BACK_RPM,
                        ShooterConstants.BOTTOM_STAGE_BACK_RPM));
    }

    public static Command shootNAngleFromStageFront(AngleSubsystem angle, ShooterSubsystem top,
            ShooterSubsystem bottom, FeederSubsystem feeder, IntakeSubsystem intake) {
        return new SequentialCommandGroup(
                angle.angleToStageFront(),
                feedAndShootAlsoIntake(
                        feeder, intake, top, bottom, ShooterConstants.TOP_STAGE_FRONT_RPM,
                        ShooterConstants.BOTTOM_STAGE_FRONT_RPM));
    }

    public static Command feedNIn(FeederSubsystem feeder, IntakeSubsystem intake) {
        return new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        feeder.primeNoteCommand(),
                        intake.pullStopCommand()),
                feeder.unprimeNoteCommand(),
                feeder.primeNoteCommand());
    }

    /**
     * Returns a command that stops the shooters when interrupted. This does not
     * require either subsystem passed into it.
     *
     * @param shooterTop    The top shooter subsystem, .stop()ed when interruped
     * @param shooterBottom The bottom shooter subsystem, .stop()ed when interruped
     *
     * @return The Command that stops both when interrupted
     */
    public static Command shootersHoldNStop(ShooterSubsystem shooterTop, ShooterSubsystem shooterBottom) {
        return Commands.startEnd(() -> {
        }, () -> {
            shooterTop.stop();
            shooterBottom.stop();
        });
    }
}
