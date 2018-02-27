package org.usfirst.frc.team1318.robot.general;

import org.usfirst.frc.team1318.robot.HardwareConstants;
import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.common.IDashboardLogger;
import org.usfirst.frc.team1318.robot.common.IMechanism;
import org.usfirst.frc.team1318.robot.driver.common.Driver;
import org.usfirst.frc.team1318.robot.driver.common.autonomous.AutonomousDriver;
import org.usfirst.frc.team1318.robot.drivetrain.DriveTrainMechanism;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

/**
 * Position manager
 * 
 * This class maintains the approximate current location and orientation of the robot relative to its starting point.
 * This uses Jim's differential odometry algorithm. In the future we can consider adding other sensors to help correct for error.
 * 
 */
@Singleton
public class PositionManager implements IMechanism
{
    private final static String LogName = "pos";

    private final IDashboardLogger logger;
    private final DriveTrainMechanism driveTrainMechanism;
    private final AHRS navx;

    // Position coordinates
    private double odometryX;
    private double odometryY;
    private double navxX;
    private double navxY;
    private double navxZ;

    // Orientation
    private double odometryAngle;
    private double navxAngle;

    // previous data (from which we will calculate changes)
    private double prevLeftDistance;
    private double prevRightDistance;

    /**
     * Initializes a new PositionManager
     * @param logger to use
     * @param provider for obtaining electronics objects
     */
    @Inject
    public PositionManager(
        IDashboardLogger logger,
        DriveTrainMechanism driveTrainMechanism)
    {
        this.logger = logger;
        this.driveTrainMechanism = driveTrainMechanism;
        this.navx = new AHRS(SPI.Port.kMXP);

        this.odometryX = 0.0;
        this.odometryY = 0.0;
        this.navxX = 0.0;
        this.navxY = 0.0;
        this.navxZ = 0.0;

        this.odometryAngle = 0.0;
        this.navxAngle = 0.0;

        this.prevLeftDistance = 0.0;
        this.prevRightDistance = 0.0;
    }

    /**
     * set the driver that the mechanism should use
     * @param driver to use
     */
    @Override
    public void setDriver(Driver driver)
    {
        // At the beginning of autonomous, reset the position manager so that we consider ourself at the origin (0,0) and facing the 0 direction.
        if (driver instanceof AutonomousDriver)
        {
            this.reset();
        }
    }

    /**
     * read all of the sensors for the mechanism that we will use in macros/autonomous mode and record their values
     */
    @Override
    public void readSensors()
    {
        // check the current distance recorded by the encoders
        double leftDistance = 0.0;
        double rightDistance = 0.0;

        if (this.driveTrainMechanism != null)
        {
            leftDistance = this.driveTrainMechanism.getLeftPosition() * HardwareConstants.DRIVETRAIN_LEFT_PULSE_DISTANCE;
            rightDistance = this.driveTrainMechanism.getRightPosition() * HardwareConstants.DRIVETRAIN_RIGHT_PULSE_DISTANCE;
        }

        // calculate the angle (in radians) based on the total distance traveled
        double angleR = ((leftDistance - rightDistance) / HardwareConstants.DRIVETRAIN_WHEEL_SEPARATION_DISTANCE);

        // correct for odometry angle inconsistencies
        angleR *= TuningConstants.DRIVETRAIN_ENCODER_ODOMETRY_ANGLE_CORRECTION;

        // calculate the average distance traveled
        double averagePositionChange = ((leftDistance - this.prevLeftDistance) + (rightDistance - this.prevRightDistance)) / 2;

        // calculate the change since last time, and update our relative position
        this.odometryX += averagePositionChange * Math.cos(angleR);
        this.odometryY += averagePositionChange * Math.sin(angleR);

        this.odometryAngle = (angleR * 360 / (2 * Math.PI)) % 360;

        // record distance for next time
        this.prevLeftDistance = leftDistance;
        this.prevRightDistance = rightDistance;

        this.navxAngle = this.navx.getAngle();
        this.navxX = this.navx.getDisplacementX() * 100.0;
        this.navxY = this.navx.getDisplacementY() * 100.0;
        this.navxZ = this.navx.getDisplacementZ() * 100.0;

        // log the current position and orientation
        this.logger.logNumber(PositionManager.LogName, "odom_angle", this.odometryAngle);
        this.logger.logNumber(PositionManager.LogName, "odom_x", this.odometryX);
        this.logger.logNumber(PositionManager.LogName, "odom_y", this.odometryY);
        this.logger.logNumber(PositionManager.LogName, "navx_angle", this.navxAngle);
        this.logger.logNumber(PositionManager.LogName, "navx_x", this.navxX);
        this.logger.logNumber(PositionManager.LogName, "navx_y", this.navxY);
        this.logger.logNumber(PositionManager.LogName, "navx_z", this.navxZ);
    }

    /**
     * calculate the various outputs to use based on the inputs and apply them to the outputs for the relevant mechanism
     */
    @Override
    public void update()
    {
    }

    /**
     * stop the relevant component
     */
    @Override
    public void stop()
    {
        this.reset();
    }

    /**
     * Retrieve the current angle in degrees
     * @return the current angle in degrees
     */
    public double getOdometryAngle()
    {
        return this.odometryAngle;
    }

    /**
     * Retrieve the current x position
     * @return the current x position
     */
    public double getOdometryX()
    {
        return this.odometryX;
    }

    /**
     * Retrieve the current y position
     * @return the current y position
     */
    public double getOdometryY()
    {
        return this.odometryY;
    }

    /**
     * Retrieve the current angle in degrees
     * @return the current angle in degrees
     */
    public double getNavxAngle()
    {
        return this.navxAngle;
    }

    /**
     * Retrieve the current x position
     * @return the current x position
     */
    public double getNavxX()
    {
        return this.navxX;
    }

    /**
     * Retrieve the current y position
     * @return the current y position
     */
    public double getNavxY()
    {
        return this.navxY;
    }

    /**
     * Retrieve the current z position
     * @return the current z position
     */
    public double getNavxZ()
    {
        return this.navxZ;
    }

    /**
     * reset the position manager so it considers the current location to be "0"
     */
    public void reset()
    {
        this.odometryX = 0.0;
        this.odometryY = 0.0;
        this.navxX = 0.0;
        this.navxY = 0.0;
        this.navxZ = 0.0;

        this.odometryAngle = 0.0;
        this.navxAngle = 0.0;

        this.prevLeftDistance = 0.0;
        this.prevRightDistance = 0.0;

        this.navx.reset();
        this.navx.resetDisplacement();
    }
}
