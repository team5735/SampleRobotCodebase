package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterConstants.ShooterType;

public class ShooterSubsystem extends SubsystemBase {
    private PIDController pid;
    private SimpleMotorFeedforward feedForward;
    private double setpoint;
    private final String name;

    private final TalonFX talon;

    public ShooterSubsystem(ShooterType type) {
        switch (type) {
            case TOP:
                name = "top";
                talon = new TalonFX(Constants.SHOOTER_MOTOR_TOP_ID);
                pid = new PIDController(
                        ShooterConstants.TOP_KP,
                        ShooterConstants.TOP_KI,
                        ShooterConstants.TOP_KD);
                feedForward = new SimpleMotorFeedforward(
                    ShooterConstants.TOP_KS,
                    ShooterConstants.TOP_KV);
                break;
            case BOTTOM:
                name = "bottom";
                talon = new TalonFX(Constants.SHOOTER_MOTOR_BOTTOM_ID);
                pid = new PIDController(
                    ShooterConstants.BOTTOM_KP,
                    ShooterConstants.BOTTOM_KI,
                    ShooterConstants.BOTTOM_KD);
                feedForward = new SimpleMotorFeedforward(
                    ShooterConstants.BOTTOM_KS,
                    ShooterConstants.BOTTOM_KV);
                break;
            default:
                name = "BIG ERROR!!!";
                talon = new TalonFX(700);
                pid = new PIDController(0, 0, 0);
                feedForward = new SimpleMotorFeedforward(0, 0);
        }

        talon.setNeutralMode(NeutralModeValue.Coast);
        talon.setInverted(false);

        feedForward = new SimpleMotorFeedforward(0, 0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shoot" + name + "Output", Math.abs(getMeasurement()));
        SmartDashboard.putNumber("shoot" + name + "PIDError", Math.abs(pid.getPositionError()));
        SmartDashboard.putNumber("shoot" + name + "Setpoint", setpoint);
        SmartDashboard.putNumber("shoot" + name + "Amps", talon.getStatorCurrent().getValueAsDouble());
    }

    public void useOutput(double pidOutput) {
        if (pid.getSetpoint() != 0) {
            double feedOutput = feedForward.calculate(pid.getSetpoint());
            talon.setVoltage(
                    pidOutput + feedOutput);
        } else {
            talon.setVoltage(0);
        }
    }

    public double getMeasurement() {
        return talon.getVelocity().getValueAsDouble() * 60;
    }

    public void setSetpoint(double setpoint) {
        if (setpoint >= 0)
            this.setpoint = setpoint;
    }

    public void stop() {
        setSetpoint(0);
    }

    public boolean isSpunUp() {
        return (Math.abs(pid.getPositionError()) < 100);
    }

    public PIDCommand shootPIDCommand() {
        return new PIDCommand(pid, () -> getMeasurement(), () -> setpoint, a -> useOutput(a),
                this);
    }
}
