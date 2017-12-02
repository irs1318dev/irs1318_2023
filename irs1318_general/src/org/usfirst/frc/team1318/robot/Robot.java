package org.usfirst.frc.team1318.robot;

import org.usfirst.frc.team1318.robot.common.IDashboardLogger;
import org.usfirst.frc.team1318.robot.common.MechanismManager;
import org.usfirst.frc.team1318.robot.common.wpilib.ITimer;
import org.usfirst.frc.team1318.robot.driver.common.Driver;
import org.usfirst.frc.team1318.robot.driver.common.autonomous.AutonomousDriver;
import org.usfirst.frc.team1318.robot.driver.common.user.UserDriver;

import com.google.inject.Guice;
import com.google.inject.Injector;

import edu.wpi.first.wpilibj.IterativeRobot;

/**
 * Main class for the FRC ? [competition name] Competition
 * Robot for IRS1318 - [robot name]
 * 
 * 
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package, you
 * must also update the manifest file in the resource directory.
 * 
 * 
 * General design comments:
 * We have the following primary types of objects dealt with here:
 * - Driver - describes the driver/operator of the robot ("autonomous" or "user")
 * - Mechanisms - define the logic that controls a mechanism given inputs/outputs.
 * - Logger - defines what should be logged and to where (dashboard, etc.).
 * 
 * @author Will
 */
public class Robot extends IterativeRobot
{
    // smartdash logging constants
    private static final String LogName = "r";

    // Driver.  This could either be the UserDriver (joystick) or the AutonomousDriver
    private Driver driver;

    // Mechanisms and injector
    private MechanismManager mechanisms;
    private IDashboardLogger logger;
    private Injector injector;

    private ITimer timer;

    /**
     * Robot-wide initialization code should go here.
     * This default Robot-wide initialization code will be called when 
     * the robot is first powered on.  It will be called exactly 1 time.
     */
    public void robotInit()
    {
        // create mechanisms
        this.mechanisms = this.getInjector().getInstance(MechanismManager.class);
        this.logger = this.getInjector().getInstance(IDashboardLogger.class);
        this.logger.logString(Robot.LogName, "state", "Init");

        this.timer = this.getInjector().getInstance(ITimer.class);
        this.logger.logNumber(Robot.LogName, "time", this.timer.get());
    }

    /**
     * Initialization code for disabled mode should go here.
     * This code will be called each time the robot enters disabled mode.
     */
    public void disabledInit()
    {
        this.timer.stop();
        this.timer.reset();

        if (this.driver != null)
        {
            this.driver.stop();
        }

        if (this.mechanisms != null)
        {
            this.mechanisms.stop();
        }

        this.logger.logString(Robot.LogName, "state", "Disabled");
    }

    /**
     * Initialization code for autonomous mode should go here.
     * This code will be called each time the robot enters autonomous mode.
     */
    public void autonomousInit()
    {
        // Create an autonomous driver
        this.driver = this.getInjector().getInstance(AutonomousDriver.class);

        this.generalInit();

        // log that we are in autonomous mode
        this.logger.logString(Robot.LogName, "state", "Autonomous");
    }

    /**
     * Initialization code for teleop mode should go here.
     * This code will be called each time the robot enters teleop mode.
     */
    public void teleopInit()
    {
        // create driver for user's joystick
        this.driver = this.getInjector().getInstance(UserDriver.class);

        this.generalInit();

        // log that we are in teleop mode
        this.logger.logString(Robot.LogName, "state", "Teleop");
    }

    /**
     * General initialization code for teleop/autonomous mode should go here.
     */
    public void generalInit()
    {
        // apply the driver to the mechanisms
        this.mechanisms.setDriver(this.driver);

        this.timer.start();
    }

    /**
     * Periodic code for disabled mode should go here.
     * This code will be called periodically at a regular rate while the robot is in disabled mode.
     */
    public void disabledPeriodic()
    {
    }

    /**
     * Periodic code for autonomous mode should go here.
     * This code will be called periodically at a regular rate while the robot is in autonomous mode.
     */
    public void autonomousPeriodic()
    {
        this.generalPeriodic();
    }

    /**
     * Periodic code for teleop mode should go here.
     * This code will be called periodically at a regular rate while the robot is in teleop mode.
     */
    public void teleopPeriodic()
    {
        this.generalPeriodic();
    }

    /**
     * General periodic code for teleop/autonomous mode should go here.
     */
    public void generalPeriodic()
    {
        this.mechanisms.readSensors();

        this.driver.update();

        // run each mechanism
        this.mechanisms.update();

        this.logger.logNumber(Robot.LogName, "time", this.timer.get());
        this.logger.flush();
    }

    /**
     * Lazily initializes and retrieves the injector.
     * @return the injector to use for this robot
     */
    Injector getInjector()
    {
        if (this.injector == null)
        {
            this.injector = Guice.createInjector(new RobotModule());
        }

        return this.injector;
    }
}
