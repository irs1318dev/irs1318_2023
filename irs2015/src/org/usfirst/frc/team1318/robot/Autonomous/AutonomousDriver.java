package org.usfirst.frc.team1318.robot.Autonomous;

import org.usfirst.frc.team1318.robot.Common.IDriver;

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
    private final IAutonomousTask autonomousTask;
    private final AutonomousControlData controlData;

    private boolean hasBegun;
    private boolean hasEnded;

    /**
     * Initializes a new AutonomousDriver
     * @param autonomousTask to execute as a part of this driver
     */
    public AutonomousDriver(IAutonomousTask autonomousTask)
    {
        this.autonomousTask = autonomousTask;
        this.controlData = new AutonomousControlData();

        this.hasBegun = false;
        this.hasEnded = false;
    }

    /**
     * Tell the driver that some time has passed
     */
    public void update()
    {
        if (!this.hasEnded)
        {
            if (!this.hasBegun)
            {
                // if we haven't begun, begin
                this.autonomousTask.begin();
                this.hasBegun = true;
            }

            if (this.autonomousTask.hasCompleted())
            {
                // if we shouldn't continue, end the task
                this.autonomousTask.end(this.controlData);
                this.hasEnded = true;
            }
            else
            {
                // run the current task and apply the result to the control data
                this.autonomousTask.update(this.controlData);
            }
        }
    }

    /**
     * Tell the operator component that operation is stopping
     */
    public void stop()
    {
        if (this.autonomousTask != null)
        {
            this.autonomousTask.cancel(this.controlData);
            this.hasEnded = true;
        }
    }

    //================================================== DriveTrain ==============================================================

    /**
     * Get a value indicating the desired drive train X Velocity 
     * @return value between -1.0 and 1.0 (percentage of max right turn velocity)
     */
    public double getDriveTrainXVelocity()
    {
        double xVelocity = this.controlData.getDriveTrainXVelocity();

        //        SmartDashboardLogger.putNumber(AutonomousDriver.DRIVETRAIN_X_VELOCITY_LOG_KEY, xVelocity);

        return xVelocity;
    }

    /**
     * Get a value indicating the desired drive train Y velocity (turn amount) 
     * @return value between -1.0 and 1.0 (percentage of max forward velocity)
     */
    public double getDriveTrainYVelocity()
    {
        double yVelocity = this.controlData.getDriveTrainYVelocity();

        //        SmartDashboardLogger.putNumber(AutonomousDriver.DRIVETRAIN_Y_VELOCITY_LOG_KEY, yVelocity);

        return yVelocity;
    }

    /**
     * Get a value indicating whether we should be using the drive train in simple mode 
     * @return true if we should be in simple mode, otherwise false
     */
    public boolean getDriveTrainSimpleMode()
    {
        boolean simpleMode = this.controlData.getDriveTrainSimpleMode();

        //        SmartDashboardLogger.putBoolean(AutonomousDriver.DRIVETRAIN_SIMPLE_MODE_LOG_KEY, simpleMode);

        return simpleMode;
    }

    //do not use in autonomous 
    public boolean getDriveTrainCollectCansFromStepMacro()
    {
        return false;
    }

    /**
     * Get a value indicating the desired drive train left position for positional mode
     * @return position
     */
    public double getDriveTrainLeftPosition()
    {
        double leftPosition = this.controlData.getDriveTrainLeftPosition();

        //        SmartDashboardLogger.putNumber(AutonomousDriver.DRIVETRAIN_LEFT_POSITION_LOG_KEY, leftPosition);

        return leftPosition;
    }

    /**
     * Get a value indicating the desired drive train right position for positional mode
     * @return position
     */
    public double getDriveTrainRightPosition()
    {
        double rightPosition = this.controlData.getDriveTrainRightPosition();

        //        SmartDashboardLogger.putNumber(AutonomousDriver.DRIVETRAIN_RIGHT_POSITION_LOG_KEY, rightPosition);

        return rightPosition;
    }

    /**
     * Get a value indicating whether the drive train should be in position or velocity mode
     * @return true if position mode, false if velocity mode
     */
    public boolean getDriveTrainPositionMode()
    {
        boolean positionMode = this.controlData.getDriveTrainPositionMode();

        //        SmartDashboardLogger.putBoolean(AutonomousDriver.DRIVETRAIN_POSITION_MODE_LOG_KEY, positionMode);

        return positionMode;
    }
}
