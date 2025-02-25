// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.AngleConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.FeederConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.LimelightConstants;
import frc.robot.constants.ShooterConstants;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private RobotContainer robotContainer;

    private PowerDistribution PD = new PowerDistribution(Constants.PDH_ID, ModuleType.kRev);

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button
        // bindings, and put our autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();

        // Put initial values of numbers that are continuously updated from
        // SmartDashboard.
        SmartDashboard.putNumber("angleKP", AngleConstants.KP);
        SmartDashboard.putNumber("angleKI", AngleConstants.KI);
        SmartDashboard.putNumber("angleKD", AngleConstants.KD);

        SmartDashboard.putNumber("angleKS", AngleConstants.KS);
        SmartDashboard.putNumber("angleKG", AngleConstants.KG);
        SmartDashboard.putNumber("angleKV", AngleConstants.KV);

        SmartDashboard.putNumber("anglePos", 0);

        SmartDashboard.putNumber("angleGoalPos", AngleConstants.START_POS_DEG);
        SmartDashboard.putNumber("angleNewGoalPos", AngleConstants.START_POS_DEG);

        SmartDashboard.putNumber("climbRightPos", 0);
        SmartDashboard.putNumber("climbLeftPos", 0);

        SmartDashboard.putNumber("climbRightOutput", 0);
        SmartDashboard.putNumber("climbLeftOutput", 0);

        SmartDashboard.putNumber("feederPullVoltage", FeederConstants.PULL_VOLTS);
        SmartDashboard.putNumber("feederPushVoltage", FeederConstants.PUSH_VOLTS);

        SmartDashboard.putBoolean("feederSwitchStatus", false);

        SmartDashboard.putNumber("intakePullVoltage", IntakeConstants.PULL_VOLTS);
        SmartDashboard.putNumber("intakePushVoltage", IntakeConstants.PUSH_VOLTS);

        SmartDashboard.putNumber("shootTopKP", ShooterConstants.TOP_KP);
        SmartDashboard.putNumber("shootTopKI", ShooterConstants.TOP_KI);
        SmartDashboard.putNumber("shootTopKD", ShooterConstants.TOP_KD);

        SmartDashboard.putNumber("shootTopKS", ShooterConstants.TOP_KS);
        SmartDashboard.putNumber("shootTopKV", ShooterConstants.TOP_KV);

        SmartDashboard.putNumber("shootBottomKP", ShooterConstants.BOTTOM_KP);
        SmartDashboard.putNumber("shootBottomKI", ShooterConstants.BOTTOM_KI);
        SmartDashboard.putNumber("shootBottomKD", ShooterConstants.BOTTOM_KD);

        SmartDashboard.putNumber("shootBottomKS", ShooterConstants.BOTTOM_KS);
        SmartDashboard.putNumber("shootBottomKV", ShooterConstants.BOTTOM_KV);

        SmartDashboard.putNumber("shootTopRPM", ShooterConstants.TOP_DEFAULT_RPM);
        SmartDashboard.putNumber("shootBottomRPM", ShooterConstants.BOTTOM_DEFAULT_RPM);
        SmartDashboard.putNumber("shootTopOutput", 0);
        SmartDashboard.putNumber("shootBottomOutput", 0);

        SmartDashboard.putNumber("drivetrain_slowSpeed", DrivetrainConstants.SLOW_SPEED);
        SmartDashboard.putNumber("drivetrain_normalSpeed", DrivetrainConstants.NORMAL_SPEED);
        SmartDashboard.putNumber("drivetrain_turboSpeed", DrivetrainConstants.TURBO_SPEED);

        SmartDashboard.putNumber("llv2_turnP", LimelightConstants.TURN_P);
        SmartDashboard.putNumber("llv2_turnI", LimelightConstants.TURN_I);
        SmartDashboard.putNumber("llv2_turnD", LimelightConstants.TURN_D);

        SmartDashboard.putNumber("testShootAngle", AngleConstants.START_POS_DEG);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for
     * items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands,
        // removing finished or interrupted commands, and running subsystem
        // periodic() methods. This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        SmartDashboard.putNumber("PD 6", PD.getCurrent(6));
        SmartDashboard.putNumber("PD total", PD.getTotalCurrent());
        SmartDashboard.putNumber("PD total voltage", PD.getVoltage());
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link
     * RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        robotContainer.resetSetpoints();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }
}
