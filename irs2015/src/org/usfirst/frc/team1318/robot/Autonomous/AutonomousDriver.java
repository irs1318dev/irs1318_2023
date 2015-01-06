package org.usfirst.frc.team1318.robot.Autonomous;

import org.usfirst.frc.team1318.robot.Common.IDriver;
import org.usfirst.frc.team1318.robot.Common.SmartDashboardLogger;

/**
 * Operator for autonomous mode.  Autonomous operator acts as the driver of the robot,
 * telling it what actions to perform as determined by the current task and tasks that have
 * come before it that intentionally don't reset their state.
 * 
 * @author Will
 *
 */
public class AutonomousDriver implements IDriver
{
    // logging constants
    private static final String COLLECTOR_EXTEND_LOG_KEY = "a.cxt";
    private static final String COLLECTOR_RETRACT_LOG_KEY = "a.cr";
    private static final String COLLECTOR_COLLECT_LOG_KEY = "a.cc";
    private static final String COLLECTOR_EXPEL_LOG_KEY = "a.cxp";
    private static final String SHOOTER_ANGLE_LOG_KEY = "a.sa";
    private static final String SHOOTER_MODE_LOG_KEY = "a.sm";
    private static final String SHOOTER_SHOOT_LOG_KEY = "a.ss";
    private static final String DRIVETRAIN_X_VELOCITY_LOG_KEY = "a.dxv";
    private static final String DRIVETRAIN_Y_VELOCITY_LOG_KEY = "a.dyv";
    private static final String DRIVETRAIN_SIMPLE_MODE_LOG_KEY = "a.dsm";
    private static final String DRIVETRAIN_LEFT_POSITION_LOG_KEY = "a.dlp";
    private static final String DRIVETRAIN_RIGHT_POSITION_LOG_KEY = "a.drp";
    private static final String DRIVETRAIN_POSITION_MODE_LOG_KEY = "a.dpm";

    private IAutonomousTask[] autonomousTasks;
    private int currentTaskPosition;
    private IAutonomousTask currentTask;
    private AutonomousControlData controlData;

    /**
     * Initializes a new AutonomousOperator
     * @param autonomousTasks to execute as a part of this operator
     */
    public AutonomousDriver(IAutonomousTask[] autonomousTasks)
    {
        // switch to Queue when available to get rid of currentTaskPosition...
        this.autonomousTasks = autonomousTasks;
        this.currentTaskPosition = 0;
        this.currentTask = null;
        this.controlData = new AutonomousControlData(); 

        this.validateAutonomousTasks();
    }

    /**
     * Tell the operator component that some time has passed
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
                this.currentTaskPosition++;
            }
        }

        // if there's no current task, find the next one and start it (if any)
        if (this.currentTask == null)
        {
            if (this.currentTaskPosition >= this.autonomousTasks.length)
            {
                return;
            }

            this.currentTask = this.autonomousTasks[this.currentTaskPosition];
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
     * Get a value indicating whether we should extend the collector 
     * @return true if we should extend, otherwise false
     */
    public boolean getCollectorExtendButton()
    {
        boolean collectorExtend = this.controlData.getCollectorExtend();

        SmartDashboardLogger.putBoolean(AutonomousDriver.COLLECTOR_EXTEND_LOG_KEY, collectorExtend);

        return collectorExtend;
    }

    /**
     * Get a value indicating whether we should retract the collector 
     * @return true if we should retract, otherwise false
     */
    public boolean getCollectorRetractButton()
    {
        boolean collectorRetract = this.controlData.getCollectorRetract();

        SmartDashboardLogger.putBoolean(AutonomousDriver.COLLECTOR_RETRACT_LOG_KEY, collectorRetract);

        return collectorRetract;
    }

    /**
     * Get a value indicating whether we should collect a ball using the collector 
     * @return true if we should collect, otherwise false
     */
    public boolean getCollectorCollectButton()
    {
        boolean collectorCollect = this.controlData.getCollectorCollect();

        SmartDashboardLogger.putBoolean(AutonomousDriver.COLLECTOR_COLLECT_LOG_KEY, collectorCollect);

        return collectorCollect;
    }

    /**
     * Get a value indicating whether we should expel a ball using the collector 
     * @return true if we should expel, otherwise false
     */
    public boolean getCollectorExpelButton()
    {
        boolean collectorExpel = this.controlData.getCollectorExpel();

        SmartDashboardLogger.putBoolean(AutonomousDriver.COLLECTOR_EXPEL_LOG_KEY, collectorExpel);

        return collectorExpel;
    }

    /**
     * Get a value indicating whether we should adjust the shooter angle 
     * @return true if we should move in, otherwise false
     */
    public boolean getShooterAngle()
    {
        boolean shooterAngle = this.controlData.getShooterAngle();

        SmartDashboardLogger.putBoolean(AutonomousDriver.SHOOTER_ANGLE_LOG_KEY, shooterAngle);

        return shooterAngle;
    }

    /**
     * Get a value indicating the shooter's current mode 
     * @return a value indicating the number of pistons to use in the shot
     */
    public int getShooterMode()
    {
        int shooterMode = this.controlData.getShooterMode();

        SmartDashboardLogger.putNumber(AutonomousDriver.SHOOTER_MODE_LOG_KEY, shooterMode);

        return shooterMode;
    }

    /**
     * Get a value indicating whether we should attempt to shoot
     * @return true if we should be shooting, otherwise false
     */
    public boolean getShooterShoot()
    {
        boolean shooterShoot = this.controlData.getShooterShoot();

        SmartDashboardLogger.putBoolean(AutonomousDriver.SHOOTER_SHOOT_LOG_KEY, shooterShoot);

        return shooterShoot;
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
    public boolean getDriveTrainSimpleModeButton()
    {
        boolean simpleMode = this.controlData.getDriveTrainSimpleMode();

        SmartDashboardLogger.putBoolean(AutonomousDriver.DRIVETRAIN_SIMPLE_MODE_LOG_KEY, simpleMode);

        return simpleMode;
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

    private void validateAutonomousTasks()
    {
        if (this.autonomousTasks == null)
        {
            throw new RuntimeException("autonomous tasks are null!");
        }

        for (int i = 0; i < this.autonomousTasks.length; i++)
        {
            if (this.autonomousTasks[i] == null)
            {
                throw new RuntimeException("null entry in autonomous tasks list!");
            }
        }
    }
}
