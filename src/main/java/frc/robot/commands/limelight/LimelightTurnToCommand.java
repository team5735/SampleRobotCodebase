// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.LimelightConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.util.NTBooleanSection;
import frc.robot.util.NTDoubleSection;
import frc.robot.util.TunableNumber;

public class LimelightTurnToCommand extends Command {
    DrivetrainSubsystem drivetrain;
    LimelightSubsystem limelight;
    PIDController pid;
    double pigeonStartingNumber;

    private final NTDoubleSection doubles = new NTDoubleSection("limelight", "drivetrain omega", "measurement",
            "setpoint");
    private final NTBooleanSection booleans = new NTBooleanSection("limelight", "aiming");

    private final TunableNumber kP = new TunableNumber("limelight", "kP", LimelightConstants.TURN_P);
    private final TunableNumber kI = new TunableNumber("limelight", "kI", LimelightConstants.TURN_I);
    private final TunableNumber kD = new TunableNumber("limelight", "kD", LimelightConstants.TURN_D);

    public LimelightTurnToCommand(final DrivetrainSubsystem drivetrain, final LimelightSubsystem limelight,
            final double offset) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;

        addRequirements(drivetrain);

        pid = new PIDController(kP.get(), kI.get(), kD.get());

        pid.setTolerance(Constants.TOLERANCE);
        pid.setSetpoint(LimelightAimCommand.positiveToPosNeg(drivetrain.getRotation3d().getZ() + offset));
        pid.enableContinuousInput(-Math.PI, Math.PI);

        pigeonStartingNumber = drivetrain.getRotation3d().getZ();

        doubles.set("setpiont", pid.getSetpoint());
    }

    @Override
    public void execute() {
        double measurement = getMeasurement();
        doubles.set("measurement", measurement);
        double omega = pid.calculate(measurement);
        doubles.set("drivetrain omega", omega);
        drivetrain.drive(omega);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0);
        booleans.set("aiming", false);
    }

    private double getMeasurement() {
        return drivetrain.getRotation3d().getZ();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(getMeasurement() - pid.getSetpoint()) < Constants.TOLERANCE;
    }
}
