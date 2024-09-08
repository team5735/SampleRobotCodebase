package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterBottomSubsystem;
import frc.robot.subsystems.shooter.ShooterTopSubsystem;

public class ShooterSpinUpCommand extends Command {
    ShooterTopSubsystem m_subsystemTop;
    ShooterBottomSubsystem m_subsystemBottom;
    double m_setpoint_top, m_setpoint_bottom;

    public ShooterSpinUpCommand(ShooterTopSubsystem topShooter, ShooterBottomSubsystem bottomShooter,
            double topShooterSpeed_rpm, double bottomShooterSpeed_rpm) {
        m_subsystemTop = topShooter;
        m_subsystemBottom = bottomShooter;
        m_setpoint_top = topShooterSpeed_rpm;
        m_setpoint_bottom = bottomShooterSpeed_rpm;
    }

    @Override
    public void initialize() {
        m_subsystemTop.setSetpoint(m_setpoint_top);
        m_subsystemBottom.setSetpoint(m_setpoint_bottom);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_subsystemTop.stop();
            m_subsystemBottom.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return m_subsystemTop.isSpunUp() && m_subsystemBottom.isSpunUp();
    }
}
