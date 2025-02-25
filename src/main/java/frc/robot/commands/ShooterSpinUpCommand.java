package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterSpinUpCommand extends Command {
    ShooterSubsystem subsystemTop;
    ShooterSubsystem subsystemBottom;
    double setpoint_top, setpoint_bottom;

    public ShooterSpinUpCommand(ShooterSubsystem topShooter, ShooterSubsystem bottomShooter,
            double topShooterSpeed_rpm, double bottomShooterSpeed_rpm) {
        subsystemTop = topShooter;
        subsystemBottom = bottomShooter;
        setpoint_top = topShooterSpeed_rpm;
        setpoint_bottom = bottomShooterSpeed_rpm;
    }

    @Override
    public void initialize() {
        subsystemTop.setSetpoint(setpoint_top);
        subsystemBottom.setSetpoint(setpoint_bottom);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            subsystemTop.stop();
            subsystemBottom.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return subsystemTop.isSpunUp() && subsystemBottom.isSpunUp();
    }
}
