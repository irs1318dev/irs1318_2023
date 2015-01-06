package org.usfirst.frc.team1318.robot.Common;

/**
 * The "driver" describes the currently requested actions that the robot should be performing.  The driver could be either
 * a user controlling the robot with a joystick, or autonomous mode controlling the robot based on a list of pre-determined
 * task which each have their own lifecycle. 
 * 
 * @author Will
 *
 */
public interface IDriver
{
    /**
     * Tell the driver that some time has passed
     */
    public void update();

    /**
     * Tell the driver that operation is stopping
     */
    public void stop();

    /**
     * Get a value indicating whether we should extend the collector 
     * @return true if we should extend, otherwise false
     */
    public boolean getCollectorExtendButton();

    /**
     * Get a value indicating whether we should retract the collector 
     * @return true if we should retract, otherwise false
     */
    public boolean getCollectorRetractButton();

    /**
     * Get a value indicating whether we should collect a ball using the collector 
     * @return true if we should collect, otherwise false
     */
    public boolean getCollectorCollectButton();

    /**
     * Get a value indicating whether we should expel a ball using the collector 
     * @return true if we should expel, otherwise false
     */
    public boolean getCollectorExpelButton();

    /**
     * Get a value indicating whether we should adjust the shooter angle 
     * @return true if we should move in, otherwise false
     */
    public boolean getShooterAngle();

    /**
     * Get a value indicating the shooter's current mode 
     * @return a value indicating the number of pistons to use in the shot
     */
    public int getShooterMode();

    /**
     * Get a value indicating whether we should attempt to shoot
     * @return true if we should be shooting, otherwise false
     */
    public boolean getShooterShoot();

    /**
     * Get a value indicating the desired drive train X Velocity 
     * @return value between -1.0 and 1.0 (percentage of max right turn velocity)
     */
    public double getDriveTrainXVelocity();

    /**
     * Get a value indicating the desired drive train Y velocity (turn amount) 
     * @return value between -1.0 and 1.0 (percentage of max forward velocity)
     */
    public double getDriveTrainYVelocity();

    /**
     * Get a value indicating whether we should be using the drive train in simple mode 
     * @return true if we should be in simple mode, otherwise false
     */
    public boolean getDriveTrainSimpleModeButton();

    /**
     * Get a value indicating the desired drive train left position for positional mode
     * @return position
     */
    public double getDriveTrainLeftPosition();

    /**
     * Get a value indicating the desired drive train right position for positional mode
     * @return position
     */
    public double getDriveTrainRightPosition();

    /**
     * Get a value indicating whether the drive train should be in position or velocity mode
     * @return true if position mode, false if velocity mode
     */
    public boolean getDriveTrainPositionMode();
}
