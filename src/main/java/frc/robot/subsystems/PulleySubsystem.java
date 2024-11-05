package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.PulleyConstants;

public class PulleySubsystem extends SubsystemBase{

    private final CANSparkMax sparkMax_pull = new CANSparkMax(Constants.PULLEY_MOTOR_ID, MotorType.kBrushless);


    public void slow(){
        sparkMax_pull.setVoltage(PulleyConstants.VOLTS_A);
    }
    
    public void fast(){
        sparkMax_pull.setVoltage(PulleyConstants.VOLTS_B);
    }
}


