// THIS IS A GENERATED FILE. Some of these values have been tweaked, however.

package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;
import frc.robot.SuperSwerveModuleConstantsFactory;
import frc.robot.subsystems.DrivetrainSubsystem;

// Drivetrain dimension wheel to wheel: 20.5 by 25

public class TunerConstants {
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with
    // the output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    public static final Slot0Configs STEER_GAINS = new Slot0Configs()
            .withKP(20).withKI(0).withKD(0.2)
            .withKS(0).withKV(1.5).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    public static final Slot0Configs DRIVE_GAINS = new Slot0Configs()
            .withKP(.05).withKI(0).withKD(0)
            .withKS(0).withKV(0).withKA(0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
    // Same for the drive motors
    private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double SLIP_CURRENT_A = 300.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double SPEED_AT_12_VOLTS_MPS = 6;

    // Every 1 rotation of the azimuth results in COUPLE_RATIO drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double COUPLE_RATIO = 3.5714285714285716;

    private static final double DRIVE_GEAR_RATIO = 6.746031746031747;
    private static final double STEER_GEAR_RATIO = 21.428571428571427;
    private static final double WHEEL_RADIUS_INCHES = 2;

    private static final boolean STEER_MOTOR_REVERSED = true;
    private static final boolean INVERT_LEFT_SIDE = false;
    private static final boolean INVERT_RIGHT_SIDE = true;

    private static final String CAN_BUS_NAME = "squab";
    private static final int PIGEON_ID = 13;

    // These are only used for simulation
    private static final double STEER_INERTIA = 0.00001;
    private static final double DRIVE_INERTIA = 0.001;
    // Simulated voltage necessary to overcome friction
    private static final double STEER_FRICTION_VOLTAGE = 0.25;
    private static final double DRIVE_FRICTION_VOLTAGE = 0.25;

    private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(PIGEON_ID)
            .withCANbusName(CAN_BUS_NAME);

    private static final SuperSwerveModuleConstantsFactory ConstantCreator = (SuperSwerveModuleConstantsFactory) (new SuperSwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
            .withSteerMotorGearRatio(STEER_GEAR_RATIO)
            .withWheelRadius(WHEEL_RADIUS_INCHES)
            .withSlipCurrent(SLIP_CURRENT_A)
            .withSteerMotorGains(STEER_GAINS)
            .withDriveMotorGains(DRIVE_GAINS)
            .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT)
            .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT)
            .withSpeedAt12VoltsMps(SPEED_AT_12_VOLTS_MPS)
            .withSteerInertia(STEER_INERTIA)
            .withDriveInertia(DRIVE_INERTIA)
            .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
            .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(COUPLE_RATIO)
            .withSteerMotorInverted(STEER_MOTOR_REVERSED));

    // Front Left
    private static final int FRONT_LEFT_DRIVE_MOTOR_ID = 4;
    private static final int FRONT_LEFT_STEER_MOTOR_ID = 3;
    private static final int FRONT_LEFT_ENCODER_ID = 10;
    private static final double FRONT_LEFT_ENCODER_OFFSET = -0.149170;

    private static final double FRONT_LEFT_X_POS_INCHES = 10.25;
    private static final double FRONT_LEFT_Y_POS_INCHES = 12.5;

    public static final Slot0Configs FRONT_LEFT_STEER_GAINS = new Slot0Configs()
            .withKP(95).withKI(0).withKD(0.2)
            .withKS(0).withKV(1.5).withKA(0);

    public static final Slot0Configs FRONT_LEFT_DRIVE_GAINS = new Slot0Configs()
            .withKP(.05).withKI(0).withKD(0.00)
            .withKS(0).withKV(0).withKA(0);

    // Front Right
    private static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 6;
    private static final int FRONT_RIGHT_STEER_MOTOR_ID = 5;
    private static final int FRONT_RIGHT_ENCODER_ID = 11;
    private static final double FRONT_RIGHT_ENCODER_OFFSET = 0.056885;

    private static final double FRONT_RIGHT_X_POS_INCHES = 9.75;
    private static final double FRONT_RIGHT_Y_POS_INCHES = -12.5;

    public static final Slot0Configs FRONT_RIGHT_STEER_GAINS = new Slot0Configs()
            .withKP(95).withKI(0).withKD(0.2)
            .withKS(0).withKV(1.5).withKA(0);

    public static final Slot0Configs FRONT_RIGHT_DRIVE_GAINS = new Slot0Configs()
            .withKP(.055).withKI(0).withKD(0.00)
            .withKS(0).withKV(0).withKA(0);

    // Back Left
    private static final int BACK_LEFT_DRIVE_MOTOR_ID = 2;
    private static final int BACK_LEFT_STEER_MOTOR_ID = 1;
    private static final int BACK_LEFT_ENCODER_ID = 9;
    private static final double BACK_LEFT_ENCODER_OFFSET = -0.490234;

    private static final double BACK_LEFT_X_POS_INCHES = -10.25;
    private static final double BACK_LEFT_Y_POS_INCHES = 12.5;

    public static final Slot0Configs BACK_LEFT_STEER_GAINS = new Slot0Configs()
            .withKP(95).withKI(0).withKD(0.2)
            .withKS(0).withKV(1.5).withKA(0);

    public static final Slot0Configs BACK_LEFT_DRIVE_GAINS = new Slot0Configs()
            .withKP(.05).withKI(0).withKD(0.00)
            .withKS(0).withKV(0).withKA(0);

    // Back Right
    private static final int BACK_RIGHT_DRIVE_MOTOR_ID = 8;
    private static final int BACK_RIGHT_STEER_MOTOR_ID = 7;
    private static final int BACK_RIGHT_ENCODER_ID = 12;
    private static final double BACK_RIGHT_ENCODER_OFFSET = 0.079834;

    private static final double BACK_RIGHT_X_POS_INCHES = -10.25;
    private static final double BACK_RIGHT_Y_POS_INCHES = -12.5;

    public static final Slot0Configs BACK_RIGHT_STEER_GAINS = new Slot0Configs()
            .withKP(95).withKI(0).withKD(0.2)
            .withKS(0).withKV(1.5).withKA(0);

    public static final Slot0Configs BACK_RIGHT_DRIVE_GAINS = new Slot0Configs()
            .withKP(.055).withKI(0).withKD(0.00)
            .withKS(0).withKV(0).withKA(0);

    private static final SwerveModuleConstants FRONT_LEFT = ConstantCreator.createModuleConstants(
            FRONT_LEFT_STEER_MOTOR_ID, FRONT_LEFT_DRIVE_MOTOR_ID, FRONT_LEFT_ENCODER_ID,
            FRONT_LEFT_DRIVE_GAINS, FRONT_RIGHT_STEER_GAINS, FRONT_LEFT_ENCODER_OFFSET,
            Units.inchesToMeters(FRONT_LEFT_X_POS_INCHES), Units.inchesToMeters(FRONT_LEFT_Y_POS_INCHES),
            INVERT_LEFT_SIDE);
    private static final SwerveModuleConstants FRONT_RIGHT = ConstantCreator.createModuleConstants(
            FRONT_RIGHT_STEER_MOTOR_ID, FRONT_RIGHT_DRIVE_MOTOR_ID, FRONT_RIGHT_ENCODER_ID,
            FRONT_RIGHT_DRIVE_GAINS, FRONT_RIGHT_STEER_GAINS, FRONT_RIGHT_ENCODER_OFFSET,
            Units.inchesToMeters(FRONT_RIGHT_X_POS_INCHES), Units.inchesToMeters(FRONT_RIGHT_Y_POS_INCHES),
            INVERT_RIGHT_SIDE);
    private static final SwerveModuleConstants BACK_LEFT = ConstantCreator.createModuleConstants(
            BACK_LEFT_STEER_MOTOR_ID, BACK_LEFT_DRIVE_MOTOR_ID, BACK_LEFT_ENCODER_ID,
            BACK_LEFT_DRIVE_GAINS, BACK_LEFT_STEER_GAINS, BACK_LEFT_ENCODER_OFFSET,
            Units.inchesToMeters(BACK_LEFT_X_POS_INCHES), Units.inchesToMeters(BACK_LEFT_Y_POS_INCHES),
            INVERT_LEFT_SIDE);
    private static final SwerveModuleConstants BACK_RIGHT = ConstantCreator.createModuleConstants(
            BACK_RIGHT_STEER_MOTOR_ID, BACK_RIGHT_DRIVE_MOTOR_ID, BACK_RIGHT_ENCODER_ID,
            BACK_RIGHT_DRIVE_GAINS, BACK_RIGHT_STEER_GAINS, BACK_RIGHT_ENCODER_OFFSET,
            Units.inchesToMeters(BACK_RIGHT_X_POS_INCHES), Units.inchesToMeters(BACK_RIGHT_Y_POS_INCHES),
            INVERT_RIGHT_SIDE);

    public static final DrivetrainSubsystem DRIVE_TRAIN = new DrivetrainSubsystem(DrivetrainConstants,
            RobotContainer.getFieldCentric, FRONT_LEFT,
            FRONT_RIGHT, BACK_LEFT, BACK_RIGHT);
}
