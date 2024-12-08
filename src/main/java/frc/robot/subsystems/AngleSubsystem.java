package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FactoryCommands;
import frc.robot.constants.AngleConstants;
import frc.robot.constants.Constants;

/**
 * This class represents the angle changer subsystem. It uses a PID to try and
 * maintain a specific angle while using a feed forward to overcome gravity,
 * except for when the angle changer is at rest. The goal can be set with
 * setGoal.
 *
 * @author Jacoby
 */
public class AngleSubsystem extends SubsystemBase {
    private ProfiledPIDController pid;
    private ArmFeedforward feedForward;
    private boolean enabled = true;
    private double startPosition = 0;
    private double goalPos, activeOutput;

    private final CANSparkMax sparkMax_right = new CANSparkMax(
            Constants.ANGLE_MOTOR_RIGHT_ID, MotorType.kBrushless);
    private final CANSparkMax sparkMax_left = new CANSparkMax(
            Constants.ANGLE_MOTOR_LEFT_ID, MotorType.kBrushless);

    private final DutyCycleEncoder encoder = new DutyCycleEncoder(Constants.ANGLE_ENCODER_PIN);

    /**
     * Creates a new AngleSubsystem. Inverts both motors, and sets the left motor to
     * follow the right. Also initializes the PID and feed forward using the
     * constants in AngleConstants and gives the PID an IZone of 1. Finally, the
     * setpoint is set to the resting position and the encoder is initialized.
     */
    public AngleSubsystem() {
        sparkMax_left.setInverted(true);
        sparkMax_right.setInverted(true);

        sparkMax_left.follow(sparkMax_right, true);

        pid = new ProfiledPIDController(
                AngleConstants.KP,
                AngleConstants.KI,
                AngleConstants.KD,
                new Constraints(
                        AngleConstants.MAX_VEL_MPS,
                        AngleConstants.MAX_ACC_MPSPS));
        feedForward = new ArmFeedforward(
                AngleConstants.KS,
                AngleConstants.KG,
                AngleConstants.KV);

        encoder.setDistancePerRotation(1);

        setGoalPos(AngleConstants.START_POS_DEG);
    }

    /**
     * This implementation of periodic() simply puts a few numbers into
     * SmartDashboard for debugging purposes. The only reason we don't use an
     * {@link frc.robot.util.NTDoubleSection} here is because the code hasn't been
     * touched since that was implemented.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("anglePos", getMeasurement());
        SmartDashboard.putNumber("angleGoalPos", goalPos);
        SmartDashboard.putNumber("anglePIDError", Math.abs(pid.getPositionError()));
        SmartDashboard.putNumber("anglePIDOutput", activeOutput);
    }

    /**
     * Determines where the angle changer is in its coordinate system by looking at
     * the encoder's output. The output is calculated with
     * AngleConstants.convertRotationsToDegrees and rounded to the tenths place.
     *
     * <p>
     * The first time this function is called, the encoder's reported distance is
     * stored as its start position. The position stored there is subtracted from
     * the encoder's distance as an offset on subsequent calls.
     *
     * @return The position of the angle changer as reported by its encoder
     */
    public double getMeasurement() {
        if (startPosition == 0) {
            startPosition = encoder.getDistance();
        }
        double currentAngleDegrees = AngleConstants.convertRotationsToDegrees(
                encoder.getDistance() - startPosition + AngleConstants.START_POS_ROT);

        return 0.1 * Math.round(currentAngleDegrees * 10);
    }

    /**
     * Sets the motor voltage to the result of the PID and FeedForward calculations.
     * This function takes the pidOutput so it can be used as the output function of
     * the {@link ProfiledPIDCommand} that is this subsystems' default. If enabled
     * is false,then the pid output and the feed forward output are ignored and the
     * motor is told to stop.
     *
     * <p>
     * If the angle changer is at rest, queried using isAtBase, then the feed
     * forward output is not used. This is done so that the angle changer isn't fed
     * voltage by the feed forward while at rest.
     *
     * @param pidOutput The voltage output from the ProfiledPIDController, passed by
     *                  the {@link ProfiledPIDCommand}.
     * @param setpoint  The current {@link State} that the ProfiledPIDController is
     *                  trying to reach (this changes rapidly and should not be
     *                  confused for goal)
     */
    public void useOutput(double pidOutput, State setpoint) {
        if (enabled) {
            double feedOutput = (!isAtBase(getMeasurement()) && !isAtBase(goalPos))
                    ? feedForward.calculate(Math.toRadians(getMeasurement()), setpoint.velocity)
                    : 0;
            double volts = pidOutput + feedOutput;
            sparkMax_right.setVoltage(volts);
        } else {
            sparkMax_right.setVoltage(0);
        }
        activeOutput = pidOutput;
    }

    /**
     * Resets the ProfiledPIDController that this subsystem uses.
     */
    public void pidReset() {
        pid.reset(getMeasurement());
    }

    /**
     * Sets the goal position in degrees that the angle changer tries to reach. The
     * coordinate system for this is as follows: angle 0 is facing opposite the
     * robot, in the direction that notes are shot, with positive angles causing the
     * angle changer to lower more towards the base.
     *
     * <p>
     * Another way to describe the coordinate system is to give examples: 130 is the
     * angle at which we can shoot into the amp, and 235 is the angle at which we
     * shoot into the speaker, which is the same as the resting position.
     *
     * @param angle The angle that the subsystem will attemt to reach
     */
    public void setGoalPos(double angle) {
        if (angle > AngleConstants.LOWEST_DEG && angle < AngleConstants.HIGHEST_DEG)
            goalPos = angle;
    }

    /**
     * Sets the right motor's idle mode to coast, releasing the brakes. Also sets
     * the subsystem to be disabled, sending 0 instead of the results of the PID and
     * feed forward to the motors as voltage.
     */
    public void releaseBrakes() {
        sparkMax_right.setIdleMode(IdleMode.kCoast);
        enabled = false;
    }

    /**
     * Sets the right motor's idle mode to brake, enabling the brakes. Also sets the
     * subsystem to be enabled, sending the results of the PID and feed forward to
     * the motors as voltage.
     */
    public void engageBrakes() {
        sparkMax_right.setIdleMode(IdleMode.kBrake);
        enabled = true;
    }

    /**
     * Returns a ProfiledPIDCommand that is intended to be set as the default
     * command of
     * this subsystem. Being a ProfiledPIDCommand, it never finishes unless
     * interrupted.
     *
     * @return A ProfiledPIDCommand that runs *this* subsystem, requiring it.
     */
    public ProfiledPIDCommand anglePIDCommand() {
        return new ProfiledPIDCommand(pid, this::getMeasurement, () -> new State(goalPos, 0), (a, b) -> useOutput(a, b),
                this);
    }

    /**
     * Returns whether the absolute difference between the provided angle and the
     * start
     * position is less than 2. The start position is defined as
     * AngleConstants.ANGLE_START_POS_DEG.
     *
     * @param pos The angle to compare
     * 
     * @return Whether the measurment is less than 5 units from the start pos
     */
    public boolean isAtBase(double pos) {
        return Math.abs(pos - AngleConstants.START_POS_DEG) < 2;
    }

    /**
     * Returns whether the angle changer is at a specific position in degrees
     *
     * @param pos The position to compare against
     *
     * @return Whether or not the difference between the measurement and the
     *         position is less than 5
     */
    public boolean isAtPosition(double pos) {
        return Math.abs(getMeasurement() - pos) < 5;
    }

    /**
     * Returns a Command which will set the goal to the start position upon
     * being scheduled.
     */
    public Command angleToBase() {
        return setAngleCommand(AngleConstants.START_POS_DEG);
    }

    /**
     * Returns a Command which will set the goal to the highest physical
     * position possible (lowest degrees + 10) upon being scheduled.
     */
    public Command angleToMax() {
        return setAngleCommand(AngleConstants.LOWEST_DEG + 10);
    }

    /**
     * Returns a Command which will set the goal to the back stage shooting
     * angle upon being scheduled.
     */
    public Command angleToStageBack() {
        return setAngleCommand(AngleConstants.STAGE_BACK_SHOOT_DEG);
    }

    /**
     * Returns a Command which will set the goal to the front stage shooting
     * angle upon being scheduled.
     */
    public Command angleToStageFront() {
        return setAngleCommand(AngleConstants.STAGE_FRONT_SHOOT_DEG);
    }

    /**
     * Continuously sets the goal to 10 less than what it was when
     * the function was called.
     */
    public Command angleIncrease() {
        return setAngleCommand(goalPos - 10).repeatedly();
    }

    /**
     * Continuously sets the goal to 10 greater than what it was
     * when the function was called.
     */
    public Command angleDecrease() {
        return setAngleCommand(goalPos + 10).repeatedly();
    }

    /**
     * Returns a Command that resets the PID.
     */
    public Command pidResetCommand() {
        return Commands.runOnce(() -> pidReset());
    }

    /**
     * Returns a Command that sets the goal to the angle provided and finishes
     * when the angle changer is at the goal.
     *
     * @param angle The angle to set as the goal position
     */
    public Command setAngleCommand(double angle) {
        return FactoryCommands.runOnceUntil(() -> setGoalPos(angle), () -> pid.atGoal());
    }

    /**
     * Returns a Command that sets the angle to the angle specified in
     * angleNewGoalPos in SmartDashboard.
     */
    public Command setSmartDashboardCommand() {
        return FactoryCommands.runOnceUntil(() -> {
            double goal = SmartDashboard.getNumber("angleNewGoalPos",
                    AngleConstants.START_POS_DEG);
            setGoalPos(goal);
        }, () -> pid.atGoal());
    }

    /**
     * This factory command releases the brakes on initialize and then engages the
     * brakes once more when interrupted.
     */
    public Command releaseMotorsCommand() {
        return startEnd(() -> releaseBrakes(), () -> engageBrakes());
    }
}
