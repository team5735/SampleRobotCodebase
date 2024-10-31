package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FactoryCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {
    private final CANSparkMax sparkMax_pull = new CANSparkMax(Constants.FEEDER_MOTOR_ID, MotorType.kBrushless);

    private final DigitalInput beamBreak = new DigitalInput(Constants.FEEDER_BEAM_PIN);

    public FeederSubsystem() {
        sparkMax_pull.setInverted(true);
    }

    public void pull() {
        sparkMax_pull.setVoltage(FeederConstants.PULL_VOLTS);
    }

    public void push() {
        sparkMax_pull.setVoltage(-FeederConstants.PUSH_VOLTS);
    }

    public void stop() {
        sparkMax_pull.setVoltage(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("feederBeamBreakStatus", getBeamBreakStatus());
        SmartDashboard.putNumber("feederVoltage", sparkMax_pull.getBusVoltage());
    }

    public boolean getBeamBreakStatus() {
        return beamBreak.get();
    }

    public Command pullStopCommand() {
        return startEnd(() -> pull(), () -> stop());
    }

    public Command pushStopCommand() {
        return startEnd(() -> push(), () -> stop());
    }

    public Command primeNoteCommand() {
        return FactoryCommands.startEndUntil(() -> pull(), () -> stop(), () -> getBeamBreakStatus(), this);
    }

    public Command unprimeNoteCommand() {
        return FactoryCommands.startEndUntil(() -> push(), () -> stop(), () -> !getBeamBreakStatus(), this);
    }

    public Command stopCommand() {
        return runOnce(() -> stop());
    }
}
