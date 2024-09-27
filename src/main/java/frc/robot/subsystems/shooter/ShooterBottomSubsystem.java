package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.ShooterConstants;

public class ShooterBottomSubsystem extends SubsystemBase {
    private PIDController pid_bottom;
    private SimpleMotorFeedforward feedForward_bottom;
    private double setpoint;

    private final TalonFX talon_bottom = new TalonFX(Constants.MOTOR_BOTTOM_ID);

    public ShooterBottomSubsystem() {
        talon_bottom.setNeutralMode(NeutralModeValue.Coast);

        pid_bottom = new PIDController(0, 0, 0);
        feedForward_bottom = new SimpleMotorFeedforward(0, 0);

        updateProportions();
    }

    /**
     * Gets values from {@link SmartDashboard} for the {@link PIDController} and the
     * {@link SimpleMotorFeedforward}. Then, pid_bottom and feedForward_bottom
     * are reconstructed based on the values acquired from {@link SmartDashboard.}
     *
     * <p>
     * Uses the following values:
     * <ul>
     * <li>shootBottomKP
     * <li>shootBottomKI
     * <li>shootBottomKD
     * <li>shootBottomKS
     * <li>shootBottomKV
     * </ul>
     */
    public void updateProportions() {
        double bkp = ShooterConstants.BOTTOM_KP;
        double bki = ShooterConstants.BOTTOM_KI;
        double bkd = ShooterConstants.BOTTOM_KD;

        double bks = ShooterConstants.BOTTOM_KS;
        double bkv = ShooterConstants.BOTTOM_KV;

        pid_bottom.setPID(bkp, bki, bkd);
        feedForward_bottom = new SimpleMotorFeedforward(bks, bkv);
    }

    @Override
    public void periodic() {
        updateProportions();

        SmartDashboard.putNumber("shootBottomOutput", Math.abs(getBottomMeasurement()));
        SmartDashboard.putNumber("shootBottomPIDError", Math.abs(pid_bottom.getPositionError()));
        SmartDashboard.putNumber("shootBottomSetpoint", setpoint);
        SmartDashboard.putNumber("shootTopAmps", talon_bottom.getStatorCurrent().getValueAsDouble());

    }

    public void useOutput(double pidOutput) {
        if (pid_bottom.getSetpoint() != 0) {
            double feedOutput = feedForward_bottom.calculate(pid_bottom.getSetpoint());
            talon_bottom.setVoltage(
                    pidOutput + feedOutput);
        } else {
            talon_bottom.setVoltage(0);
        }
    }

    public double getBottomMeasurement() {
        return talon_bottom.getVelocity().getValueAsDouble() * 60;
    }

    public void setSetpoint(double setpoint) {
        if (setpoint >= 0)
            this.setpoint = setpoint;
    }

    public void stop() {
        setSetpoint(0);
    }

    public boolean isSpunUp() {
        return Math.abs(pid_bottom.getPositionError()) < 100;
    }

    public PIDCommand shootPIDCommand() {
        return new PIDCommand(pid_bottom, () -> getBottomMeasurement(), () -> setpoint,
                a -> useOutput(a), this);
    }
}
