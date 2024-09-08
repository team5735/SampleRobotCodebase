package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.util.NTDoubleSection;

public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax sparkMax;
    private final RelativeEncoder encoder;

    NTDoubleSection doubles;

    public ClimberSubsystem(String name, int motorID) {
        sparkMax = new CANSparkMax(motorID, MotorType.kBrushless);
        encoder = sparkMax.getEncoder();
        doubles = new NTDoubleSection(name, "output", "position");
    }

    private void up() {
        sparkMax.setVoltage(-ClimberConstants.CLIMBER_UP_VOLTS);
    }

    private void down() {
        sparkMax.setVoltage(ClimberConstants.CLIMBER_DOWN_VOLTS);
    }

    private void stop() {
        sparkMax.setVoltage(0);
    }

    @Override
    public void periodic() {
        doubles.set("output", sparkMax.getOutputCurrent());
        doubles.set("position", encoder.getPosition());
    }

    public Command upStopCommand() {
        return startEnd(() -> up(), () -> stop());
    }

    public Command downStopCommand() {
        return startEnd(() -> down(), () -> stop());
    }

    public Command stopCommand() {
        return runOnce(() -> stop());
    }
}
