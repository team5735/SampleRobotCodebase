package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Compositions;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// WAIT!!! BEFORE YOU DO ANYTHING:
// Any command registered in auto for a subsystem that uses a default command should NEVER
// use addRequirements or anything that uses requirements (i.e. instead of subsystem.runOnce()
// you should use Commands.runOnce()) because the default command won't run in auto otherwise!
public class AutoCommands {
    public static void registerCommands(final IntakeSubsystem intake, final FeederSubsystem feeder,
            final ShooterSubsystem shooterTop, final ShooterSubsystem shooterBottom) {
        Map<String, Command> commandsToRegister = new HashMap<>();

        commandsToRegister.put("getNote", Compositions.feedNIn(feeder, intake));
        commandsToRegister.put("shooterStart", shooterStart(shooterTop, shooterBottom));

        commandsToRegister.put("stopShooter", stopShooter(shooterTop, shooterBottom));
        commandsToRegister.put("shootNote",
                new ParallelCommandGroup(feeder.pullStopCommand(), intake.pullStopCommand()));
        commandsToRegister.put("waitShootSpinup", spunUpDeadline(shooterTop, shooterBottom));

        NamedCommands.registerCommands(commandsToRegister);
    }

    public static Command getNote(IntakeSubsystem intake, FeederSubsystem feeder) {
        return new SequentialCommandGroup(
                new ParallelDeadlineGroup(feeder.primeNoteCommand(), intake.pullStopCommand()),
                feeder.unprimeNoteCommand());
    }

    public static Command shooterStart(ShooterSubsystem top, ShooterSubsystem bottom) {
        return new ParallelCommandGroup(
                Commands.runOnce(() -> top.setSetpoint(
                        SmartDashboard.getNumber("shootTopRPM", ShooterConstants.TOP_DEFAULT_RPM))),
                Commands.runOnce(() -> bottom.setSetpoint(
                        SmartDashboard.getNumber("shootBottomRPM", ShooterConstants.BOTTOM_DEFAULT_RPM))));
    }

    public static Command stopShooter(ShooterSubsystem top, ShooterSubsystem bottom) {
        return new ParallelCommandGroup(Commands.runOnce(() -> top.stop()), Commands.runOnce(() -> bottom.stop()));
    }

    public static Command spunUpDeadline(ShooterSubsystem top, ShooterSubsystem bottom) {
        return Commands.waitUntil(() -> top.isSpunUp() && bottom.isSpunUp());
    }
}
