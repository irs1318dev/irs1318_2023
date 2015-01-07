package org.usfirst.frc.team1318.robot.Autonomous;

import java.util.Queue;

import org.usfirst.frc.team1318.robot.Common.IDriver;
import org.usfirst.frc.team1318.robot.Common.SmartDashboardLogger;

/**
 * Driver for autonomous mode.  Autonomous driver acts as the operator of the robot,
 * telling it what actions to perform as determined by the current task and tasks that have
 * come before it that intentionally don't reset their state.
 * 
 * @author Will
 *
 */
public class AutonomousDriver implements IDriver
{
    // logging constants
    private static final String DRIVETRAIN_X_VELOCITY_LOG_KEY = "a.dxv";
    private static final String DRIVETRAIN_Y_VELOCITY_LOG_KEY = "a.dyv";
    private static final String DRIVETRAIN_SIMPLE_MODE_LOG_KEY = "a.dsm";
    private static final String DRIVETRAIN_SHIFTER_MODE_LOG_KEY = "a.dshm";
    private static final String DRIVETRAIN_LEFT_POSITION_LOG_KEY = "a.dlp";
    private static final String DRIVETRAIN_RIGHT_POSITION_LOG_KEY = "a.drp";
    private static final String DRIVETRAIN_POSITION_MODE_LOG_KEY = "a.dpm";

    private Queue<IAutonomousTask> autonomousTasks;
    private IAutonomousTask currentTask;
    private AutonomousControlData controlData;

    /**
     * Initializes a new AutonomousDriver
     * @param autonomousTasks to execute as a part of this driver
     */
    public AutonomousDriver(Queue<IAutonomousTask> autonomousTasks)
    {
        this.autonomousTasks = autonomousTasks;
        this.currentTask = null;
        this.controlData = new AutonomousControlData();
    }

    /**
     * Tell the driver that some time has passed
     */
    public void update()
    {
        // check whether we should continue with the current task
        if (this.currentTask != null)
        {
            if (!this.currentTask.shouldContinue())
            {
                this.currentTask.end(this.controlData);
                this.currentTask = null;
            }
        }

        // if there's no current task, find the next one and start it (if any)
        if (this.currentTask == null)
        {
            this.currentTask = this.autonomousTasks.poll();

            // if there's no next task to run, then we are done
            if (this.currentTask == null)
            {
                return;
            }

            this.currentTask.begin();
        }

        // run the current task and apply the result to the control data
        this.currentTask.update(this.controlData);
    }

    /**
     * Tell the operator component that operation is stopping
     */
    public void stop()
    {
        this.currentTask.cancel(this.controlData);
    }

    /**
     * Get a value indicating the desired drive train X Velocity 
     * @return value between -1.0 and 1.0 (percentage of max right turn velocity)
     */
    public double getDriveTrainXVelocity()
    {
        double xVelocity = this.controlData.getDriveTrainXVelocity();

        SmartDashboardLogger.putNumber(AutonomousDriver.DRIVETRAIN_X_VELOCITY_LOG_KEY, xVelocity);

        return xVelocity;
    }

    /**
     * Get a value indicating the desired drive train Y velocity (turn amount) 
     * @return value between -1.0 and 1.0 (percentage of max forward velocity)
     */
    public double getDriveTrainYVelocity()
    {
        double yVelocity = this.controlData.getDriveTrainYVelocity();

        SmartDashboardLogger.putNumber(AutonomousDriver.DRIVETRAIN_Y_VELOCITY_LOG_KEY, yVelocity);

        return yVelocity;
    }

    /**
     * Get a value indicating whether we should be using the drive train in simple mode 
     * @return true if we should be in simple mode, otherwise false
     */
    public boolean getDriveTrainSimpleMode()
    {
        boolean simpleMode = this.controlData.getDriveTrainSimpleMode();

        SmartDashboardLogger.putBoolean(AutonomousDriver.DRIVETRAIN_SIMPLE_MODE_LOG_KEY, simpleMode);

        return simpleMode;
    }

    /**
     * Gets a value indicating whether the shifter state should change 
     * @return true for state should change, false for no change 
     */
    public boolean getDriveTrainShifterMode()
    {

        boolean shifterMode = this.controlData.getDriveTrainShifterMode();

        SmartDashboardLogger.putBoolean(AutonomousDriver.DRIVETRAIN_SHIFTER_MODE_LOG_KEY, shifterMode);

        return shifterMode;
    }

    /**
     * Get a value indicating the desired drive train left position for positional mode
     * @return position
     */
    public double getDriveTrainLeftPosition()
    {
        double leftPosition = this.controlData.getDriveTrainLeftPosition();

        SmartDashboardLogger.putNumber(AutonomousDriver.DRIVETRAIN_LEFT_POSITION_LOG_KEY, leftPosition);

        return leftPosition;
    }

    /**
     * Get a value indicating the desired drive train right position for positional mode
     * @return position
     */
    public double getDriveTrainRightPosition()
    {
        double rightPosition = this.controlData.getDriveTrainRightPosition();

        SmartDashboardLogger.putNumber(AutonomousDriver.DRIVETRAIN_RIGHT_POSITION_LOG_KEY, rightPosition);

        return rightPosition;
    }

    /**
     * Get a value indicating whether the drive train should be in position or velocity mode
     * @return true if position mode, false if velocity mode
     */
    public boolean getDriveTrainPositionMode()
    {
        boolean positionMode = this.controlData.getDriveTrainPositionMode();

        SmartDashboardLogger.putBoolean(AutonomousDriver.DRIVETRAIN_POSITION_MODE_LOG_KEY, positionMode);

        return positionMode;
    }

}
