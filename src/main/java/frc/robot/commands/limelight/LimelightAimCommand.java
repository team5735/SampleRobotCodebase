// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LimelightConstants;
import frc.robot.subsystems.AngleSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.util.AllianceSwitcher;
import frc.robot.util.NTBooleanSection;
import frc.robot.util.NTDoubleSection;

public class LimelightAimCommand extends Command {
    private LimelightSubsystem limelight;
    private DrivetrainSubsystem drivetrain;
    private AngleSubsystem angleChanger;
    private boolean targetAcquired = false;
    private Watchdog watchdog = new Watchdog(0.02, () -> {
    });

    private final NTDoubleSection doubles = new NTDoubleSection("limelight", "current rotation", "hood distance",
            "cannot aim distance", "drivetrain speed x", "drivetrain speed y", "desired drivetrain angle",
            "hood vector x", "hood vector y", "angle changer radians");

    private final NTBooleanSection booleans = new NTBooleanSection("limelight", "aiming", "spinning");

    private static final Translation3d hoodPos = AllianceSwitcher.compute(LimelightConstants.HOOD_POS);

    /**
     * Creates a new LimelightAimCommand. This is responsible for turning the
     * robot to face the hood and for setting the angle changer to the correct angle
     * to shoot a NOTE into the hood.
     *
     * @param limelight  The limelight that is used for aiming
     * @param drivetrain The drivetrain, used to turn automatically and aim
     *                   horizontally
     * @param angle      The angle changer, used to aim vertically
     */
    public LimelightAimCommand(final LimelightSubsystem limelight, final DrivetrainSubsystem drivetrain,
            final AngleSubsystem angleChanger) {
        addRequirements(limelight, drivetrain);
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.angleChanger = angleChanger;
        targetAcquired = false;
    }

    @Override
    public void execute() {
        watchdog.reset();

        Pose3d botPose = limelight.getBotPose();
        watchdog.addEpoch("get bot pose");
        // we're ignoring the fact that 1 Apriltag isn't enough for a precise position
        // estimate
        if (!limelight.hasTarget()) {
            booleans.set("spinning", true);
            drivetrain.drive(LimelightConstants.CLUELESS_TURN_SPEED);
            return;
        }
        watchdog.addEpoch("check num targets");

        booleans.set("aiming", true);
        targetAcquired = true;
        watchdog.addEpoch("a target has been acquired");

        // coordinate system for field-oriented limelight targeting: x along long side
        // with positive towards red alliance, y along short side such that positive is
        // facing the long side that has red on the right and blue on the left, z up,
        // positive theta is counterclockwise and theta 0 is facing the red alliance
        // speaker.

        Pose2d currentRobotPose = botPose.toPose2d();
        checkBotCanAim(currentRobotPose.getTranslation(), hoodPos);
        watchdog.addEpoch("checked bot can aim");

        Translation2d robotToHood = hoodPos.toTranslation2d().minus(currentRobotPose.getTranslation());
        aimHorizontally(robotToHood, currentRobotPose.getRotation().getRadians());
        doubles.set("current rotation", currentRobotPose.getRotation().getRadians());
        watchdog.addEpoch("aimed horizontally");

        Translation3d robotPosTranslation3d = new Translation3d(currentRobotPose.getX(), currentRobotPose.getY(), 0);
        double robotRotation = currentRobotPose.getRotation().getRadians();
        Translation3d angleChangerPosition = robotPosTranslation3d.plus(LimelightConstants.ANGLE_CHANGER_OFFSET
                .rotateBy(new Rotation3d(0, 0, robotRotation)));
        aimVertically(angleChangerPosition, hoodPos);
        watchdog.addEpoch("aimed vertically");

        doubles.set("hood distance", robotToHood.getNorm());
        watchdog.disable();
        watchdog.printEpochs();
    }

    /*
     * Checks to see if the robot is within reasonable shooting range of the target
     */
    private void checkBotCanAim(Translation2d robotPosition, Translation3d targetPosition) {
        boolean botIsCloseEnough = robotPosition
                .getDistance(targetPosition.toTranslation2d()) < LimelightConstants.BOT_SHOOTING_DISTANCE;

        if (botIsCloseEnough) {
            return;
        }

        doubles.set("cannot aim distance",
                robotPosition.getDistance(hoodPos.toTranslation2d()));
        Translation2d robotToTarget = targetPosition.toTranslation2d().minus(robotPosition);
        Translation2d desiredVelocity = robotToTarget.div(robotToTarget.getNorm())
                .times(LimelightConstants.DRIVETRAIN_MOVEMENT_SPEED); // set magnitude to allowed drivetrain
                                                                      // movement speed
        doubles.set("drivetrain speed x", desiredVelocity.getX());
        doubles.set("drivetrain speed y", desiredVelocity.getY());
        return;
    }

    private void aimHorizontally(Translation2d currentRobotPoseToTarget, double curRobotRot) {
        double drivetrainDesiredAngle = Math.atan2(currentRobotPoseToTarget.getY(), currentRobotPoseToTarget.getX());
        double offset = posNegToPositive(drivetrainDesiredAngle);

        doubles.set("desired drivetrain angle", drivetrainDesiredAngle);
        doubles.set("hood vector x", currentRobotPoseToTarget.getX());
        doubles.set("hood vector y", currentRobotPoseToTarget.getY());

        new LimelightTurnToCommand(drivetrain, limelight, offset).schedule();
    }

    private double radiansEnsureInBounds(double angle) {
        if (angle > -Math.PI && angle < Math.PI) {
            return angle;
        }
        double diff = Math.abs(Math.abs(angle) - Math.PI);
        return Math.PI * -Math.signum(angle) + diff * Math.signum(angle);
    }

    private void aimVertically(Translation3d angler, Translation3d target) {
        // theta 0 is parallel to the ground and facing the front of the robot
        Translation3d anglerToTarget = target.minus(angler);
        double anglerToTargetAngle1 = Math.atan2(anglerToTarget.getZ(), anglerToTarget.getX());
        double anglerToTargetAngle2 = Math.acos(LimelightConstants.ANGLE_CHANGER_RADIUS / anglerToTarget.getNorm());

        double angleChangerDesiredAngle = radiansEnsureInBounds(anglerToTargetAngle1 + anglerToTargetAngle2);
        double anglerSetpoint = -Math.toDegrees(angleChangerDesiredAngle) + 180;

        doubles.set("angle changer radians", angleChangerDesiredAngle);

        angleChanger.setGoalPos(anglerSetpoint);
    }

    public static double positiveToPosNeg(double in) {
        if (in > Math.PI) {
            return -(in - Math.PI);
        }
        return in;
    }

    public static double posNegToPositive(double in) {
        if (in < 0) {
            return Math.PI + (-in);
        }
        return in;
    }

    @Override
    public boolean isFinished() {
        if (LimelightConstants.INFINITE_AIM) {
            return false;
        }

        return targetAcquired;
    }
}
