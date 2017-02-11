package org.usfirst.frc.team1318.robot;

import org.usfirst.frc.team1318.robot.common.IDashboardLogger;
import org.usfirst.frc.team1318.robot.driver.Driver;
import org.usfirst.frc.team1318.robot.driver.autonomous.AutonomousDriver;
import org.usfirst.frc.team1318.robot.driver.user.UserDriver;

import com.google.inject.Guice;
import com.google.inject.Injector;

import edu.wpi.first.wpilibj.IterativeRobot;

/**
 * Main class for the FRC 201? [competition name] Competition
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
 * We have three types of objects:
 * - Driver - describes the driver/operator of the robot ("autonomous" or "user")
 * - Components - describe the electronics of an mechanism and defines the abstract way to control those electronics.
 * - Controllers - define the logic that controls a mechanism given inputs/outputs.
 * 
 * @author Will
 */
public class Robot extends IterativeRobot
{
    // smartdash logging constants
    private static final String LogName = "r";

    // Driver.  This could either be the UserDriver (joystick) or the AutonomousDriver
    private Driver driver;

    // Controllers and injector
    private ControllerManager controllers;
    private IDashboardLogger logger;
    private Injector injector;

    /**
     * Robot-wide initialization code should go here.
     * This default Robot-wide initialization code will be called when 
     * the robot is first powered on.  It will be called exactly 1 time.
     */
    public void robotInit()
    {
        // create mechanism components and controllers
        this.controllers = this.getInjector().getInstance(ControllerManager.class);
        this.logger = this.getInjector().getInstance(IDashboardLogger.class);
        this.logger.logString(Robot.LogName, "state", "Init");
    }

    /**
     * Initialization code for disabled mode should go here.
     * This code will be called each time the robot enters disabled mode.
     */
    public void disabledInit()
    {
        if (this.driver != null)
        {
            this.driver.stop();
        }

        if (this.controllers != null)
        {
            this.controllers.stop();
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
        // apply the driver to the controllers
        this.controllers.setDriver(this.driver);
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
        this.driver.update();

        // run each controller
        this.controllers.update();
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
