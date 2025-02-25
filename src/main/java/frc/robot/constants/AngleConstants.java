package frc.robot.constants;

public class AngleConstants {
    public static final double KP = 0.0625;
    public static final double KI = 0;
    public static final double KD = 0.005;

    public static final double KS = 0;
    public static final double KG = 0.78;
    public static final double KV = 0;

    public static final double MAX_VEL_MPS = 0;
    public static final double MAX_ACC_MPSPS = 0;

    public static final double START_POS_ROT = 235. / 360;
    public static final double START_POS_DEG = convertRotationsToDegrees(START_POS_ROT);

    public static final double HIGHEST_DEG = 237;
    public static final double LOWEST_DEG = 120;

    public static final double AMP_DEG = 125;
    public static final double STAGE_BACK_SHOOT_DEG = 190;
    public static final double STAGE_FRONT_SHOOT_DEG = 200;

    // planetary gearbox ratio is 9:1, chain drive ratio is 16:42
    public static final double MOTOR_TO_OUTPUT_RATIO = 1.0 / 9 * 16.0 / 42;
    // inverse of the previous ratio
    public static final double OUTPUT_TO_MOTOR_RATIO = 1 / MOTOR_TO_OUTPUT_RATIO;

    public static double convertRotationsToDegrees(double rotations) {
        rotations %= 1;
        if (rotations < 0)
            rotations += 1;
        rotations *= 360;

        return rotations;
    }
}
