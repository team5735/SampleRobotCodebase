package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax sparkMax_pull = new CANSparkMax(Constants.INTAKE_MOTOR_ID, MotorType.kBrushless);

    public IntakeSubsystem() {
        sparkMax_pull.setInverted(true);
    }

    public void pull() {
        double pullSpeed = IntakeConstants.PULL_VOLTS;

        sparkMax_pull.setVoltage(pullSpeed);
    }

    public void push() {
        double pushSpeed = IntakeConstants.PUSH_VOLTS;

        sparkMax_pull.setVoltage(-pushSpeed);
    }

    public void stop() {
        sparkMax_pull.setVoltage(0);
    }

    public Command pullStopCommand() {
        return startEnd(() -> pull(), () -> stop());
    }

    public Command pushStopCommand() {
        return runOnce(() -> push());
    }

    public Command stopCommand() {
        return runOnce(() -> stop());
    }
}
