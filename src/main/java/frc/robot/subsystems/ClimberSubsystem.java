package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.ClimberConstants.ClimberType;
import frc.robot.util.NTDoubleSection;

public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax sparkMax;
    private final RelativeEncoder encoder;

    NTDoubleSection doubles;

    public ClimberSubsystem(ClimberType type) {
        switch (type) {
            case RIGHT:
                sparkMax = new CANSparkMax(Constants.CLIMBER_MOTOR_RIGHT_ID, MotorType.kBrushless);
                doubles = new NTDoubleSection("right climber", "output", "position");
                break;

            case LEFT:
                sparkMax = new CANSparkMax(Constants.CLIMBER_MOTOR_LEFT_ID, MotorType.kBrushless);
                doubles = new NTDoubleSection("left climber", "output", "position");
                break;

            default:
                sparkMax = new CANSparkMax(700, MotorType.kBrushless);
                doubles = new NTDoubleSection("BIG ERROR!!!", "output", "position");
                break;
        }


        encoder = sparkMax.getEncoder();
    }

    private void up() {
        sparkMax.setVoltage(-ClimberConstants.UP_VOLTS);
    }

    private void down() {
        sparkMax.setVoltage(ClimberConstants.DOWN_VOLTS);
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
