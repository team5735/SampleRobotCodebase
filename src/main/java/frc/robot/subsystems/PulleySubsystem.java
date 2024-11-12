package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.PulleyConstants;

public class PulleySubsystem extends SubsystemBase {
    private final CANSparkMax sparkMax = new CANSparkMax(Constants.PULLEY_MOTOR_ID, MotorType.kBrushless);

    private void pullFast(){
        sparkMax.setVoltage(PulleyConstants.FAST_VOLTS);
    }

    private void pullSlow(){
        sparkMax.setVoltage(PulleyConstants.SLOW_VOLTS);
    }

    private void stop(){
        sparkMax.setVoltage(0);
    }
}
