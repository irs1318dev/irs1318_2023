package org.usfirst.frc.team1318.robot.general;

import org.usfirst.frc.team1318.robot.HardwareConstants;
import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.common.DashboardLogger;
import org.usfirst.frc.team1318.robot.common.IController;
import org.usfirst.frc.team1318.robot.driver.Driver;
import org.usfirst.frc.team1318.robot.driver.autonomous.AutonomousDriver;
import org.usfirst.frc.team1318.robot.drivetrain.DriveTrainComponent;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * Position manager
 * 
 * This class maintains the approximate current location and orientation of the robot relative to its starting point.
 * This uses Jim's differential odometry algorithm. In the future we can consider adding other sensors to help correct for error.
 * 
 */
@Singleton
public class PositionManager implements IController
{
    private final static String LogName = "pos";

    // drivetrain component
    private final DriveTrainComponent driveTrainComponent;
    //private final AHRS navx;

    // Position coordinates
    private double x;
    private double y;

    // Orientation
    private double angle;

    // previous data (from which we will calculate changes)
    private double prevLeftDistance;
    private double prevRightDistance;

    /**
     * Initializes a new PositionManager
     * @param driveTrainComponent to use to determine position changes
     */
    @Inject
    public PositionManager(DriveTrainComponent driveTrainComponent)
    {
        this.driveTrainComponent = driveTrainComponent;
        //this.navx = new AHRS(SPI.Port.kMXP);

        this.x = 0.0;
        this.y = 0.0;

        this.angle = 0.0;

        this.prevLeftDistance = 0.0;
        this.prevRightDistance = 0.0;
    }

    /**
     * set the driver that the controller should use
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
     * calculate the various outputs to use based on the inputs and apply them to the outputs for the relevant component
     */
    @Override
    public void update()
    {
        // check the current distance recorded by the encoders
        double leftDistance = 0.0;
        double rightDistance = 0.0;

        if (this.driveTrainComponent != null)
        {
            leftDistance = this.driveTrainComponent.getLeftEncoderDistance();
            rightDistance = this.driveTrainComponent.getRightEncoderDistance();
        }

        // calculate the angle (in radians) based on the total distance traveled
        double angleR = ((leftDistance - rightDistance) / HardwareConstants.DRIVETRAIN_WHEEL_SEPARATION_DISTANCE);

        // correct for odometry angle inconsistencies
        angleR *= TuningConstants.DRIVETRAIN_ENCODER_ODOMETRY_ANGLE_CORRECTION;

        // calculate the average distance traveled
        double averagePositionChange = ((leftDistance - this.prevLeftDistance) + (rightDistance - this.prevRightDistance)) / 2;

        // calculate the change since last time, and update our relative position
        this.x += averagePositionChange * Math.cos(angleR);
        this.y += averagePositionChange * Math.sin(angleR);

        this.angle = (angleR * 360 / (2 * Math.PI)) % 360;

        // record distance for next time
        this.prevLeftDistance = leftDistance;
        this.prevRightDistance = rightDistance;

        // log the current position and orientation
        DashboardLogger.logNumber(PositionManager.LogName, "odom_angle", this.getOdometryAngle());
        DashboardLogger.logNumber(PositionManager.LogName, "odom_x", this.getOdometryX());
        DashboardLogger.logNumber(PositionManager.LogName, "odom_y", this.getOdometryY());
        DashboardLogger.logNumber(PositionManager.LogName, "navx_angle", this.getNavxAngle());
        DashboardLogger.logNumber(PositionManager.LogName, "navx_x", this.getNavxX());
        DashboardLogger.logNumber(PositionManager.LogName, "navx_y", this.getNavxY());
    }

    /**
     * stop the relevant component
     */
    @Override
    public void stop()
    {
    }

    /**
     * Retrieve the current angle in degrees
     * @return the current angle in degrees
     */
    public double getOdometryAngle()
    {
        return this.angle;
    }

    /**
     * Retrieve the current x position
     * @return the current x position
     */
    public double getOdometryX()
    {
        return this.x;
    }

    /**
     * Retrieve the current y position
     * @return the current y position
     */
    public double getOdometryY()
    {
        return this.y;
    }

    /**
     * Retrieve the current angle in degrees
     * @return the current angle in degrees
     */
    public double getNavxAngle()
    {
        return 0.0; //this.navx.getAngle();
    }

    /**
     * Retrieve the current x position
     * @return the current x position
     */
    public double getNavxX()
    {
        return 0.0; //this.navx.getDisplacementX() * 100.0;
    }

    /**
     * Retrieve the current y position
     * @return the current y position
     */
    public double getNavxY()
    {
        return 0.0; //this.navx.getDisplacementY() * 100.0;
    }

    /**
     * reset the position manager so it considers the current location to be "0"
     */
    public void reset()
    {
        this.x = 0.0;
        this.y = 0.0;

        this.angle = 0.0;

        this.prevLeftDistance = 0.0;
        this.prevRightDistance = 0.0;
    }
}
