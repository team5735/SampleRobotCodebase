package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.DrivetrainConstants.SlewRateLimiterMode;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveCommand extends Command {
    private final DrivetrainSubsystem drivetrain;
    private final Supplier<Double> stickX;
    private final Supplier<Double> stickY;
    private final Supplier<Double> rotate;
    private final Supplier<Double> multiplierSupplier;
    private Watchdog watchdog = new Watchdog(0.02, () -> {
    });
    private SlewRateLimiter thetaLimiter = new SlewRateLimiter(DrivetrainConstants.ACCEL_LIMIT_THETA_MAGNITUDE);
    private SlewRateLimiter magnitudeLimiter = new SlewRateLimiter(DrivetrainConstants.ACCEL_LIMIT_THETA_MAGNITUDE);
    private SlewRateLimiter xLimiter = new SlewRateLimiter(DrivetrainConstants.ACCEL_LIMIT_AXES);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(DrivetrainConstants.ACCEL_LIMIT_AXES);
    private SlewRateLimiter omegaLimiter = new SlewRateLimiter(DrivetrainConstants.ACCEL_LIMIT_OMEGA);

    /**
     * Creates a new DriveCommand. This class takes the drivetrain to drive, the
     * stick x and y inputs, the rotate inputs, and a final supplier that it
     * multiplies the stick inputs by to determine the speed to drive the drivetrain
     * at. Depending on the value of {@link DrivetrainConstants.SLEW_RATE_LIMITER},
     * behavior changes. See execute() to understand what happens based on each
     * mode.
     */
    public DriveCommand(DrivetrainSubsystem drivetrain, Supplier<Double> stickX, Supplier<Double> stickY,
            Supplier<Double> rotate, Supplier<Double> multiplier) {
        this.drivetrain = drivetrain;

        this.stickX = stickX;
        this.stickY = stickY;
        this.rotate = rotate;
        this.multiplierSupplier = multiplier;

        addRequirements(drivetrain);
    }

    public void execute() {
        watchdog.reset();

        double multiplier = multiplierSupplier.get();
        double speedX = stickY.get() * multiplier;
        double speedY = stickX.get() * multiplier;
        double speedOmega = omegaLimiter.calculate(rotate.get() * multiplier);
        if (DrivetrainConstants.SLEW_RATE_LIMITER_MODE == SlewRateLimiterMode.THETA_MAGNITUDE) {
            driveThetaMagnitudeSRL(speedX, speedY, speedOmega);
        } else if (DrivetrainConstants.SLEW_RATE_LIMITER_MODE == SlewRateLimiterMode.AXES) {
            driveAxesSRL(speedX, speedY, speedOmega);
        } else {
            drivetrain.drive(speedX, speedY, speedOmega);
        }

        watchdog.addEpoch("drivetrain_update");
        watchdog.disable();
        if (watchdog.isExpired()) {
            System.out.println("watchdog expired :( ");
            watchdog.printEpochs();
        }
    }

    /**
     * This passes the speedX and speedY values into a {@link SlewRateLimiter}. It
     * doesn't limit the acceleration at which the drivetrain can change direction
     * of movement, which means that the steer motors can accelerate unbounded.
     */
    private void driveAxesSRL(double speedX, double speedY, double speedOmega) {
        speedX = xLimiter.calculate(stickY.get());
        speedY = yLimiter.calculate(stickX.get());
        SmartDashboard.putNumber("drive_speedX", speedX);
        SmartDashboard.putNumber("drive_speedY", speedY);
        SmartDashboard.putNumber("drive_speedOmega", speedOmega);
        drivetrain.drive(speedX, speedY, speedOmega);
    }

    /**
     * This interprets speedX and speedY as a vector and uses
     * {@link SlewRateLimiter} to interpolate between one vector and another.
     *
     * <p>
     * Currently, this is implemented using complex numbers. It may be easier to
     * visualize as vectors, but you should learn complex numbers anyway.
     *
     * <ol>
     * <li>Interpret speedX and speedY as a complex number c, where c = speedX +
     * speedY * i.
     * <li>Convert c to polar form, that is, r * e^i*theta.
     * <li>Pass r and theta to a slew rate limiter.
     * <li>Convert the resulting polar number back into rectangular form, d =
     * speedX'
     * + speedY' * i.
     * <li>Pass speedX' and speedY' to the drivetrain, and pass speedOmega
     * unmodified.
     */
    private void driveThetaMagnitudeSRL(double speedX, double speedY, double speedOmega) {
        double theta = thetaLimiter.calculate(new Rotation2d(speedX, speedY).getRadians());
        double r = magnitudeLimiter.calculate(Math.sqrt(speedX * speedX + speedY * speedY));
        Translation2d thetaMagnitudeMovement = new Translation2d(r, theta);
        SmartDashboard.putNumber("drive_theta", theta);
        SmartDashboard.putNumber("drive_magnitude", r);
        drivetrain.drive(thetaMagnitudeMovement, speedOmega);
    }
}
