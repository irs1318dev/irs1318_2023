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
     * Gets a value indicating whether the shifter state should change 
     * @return true for state should change, false for no change 
     */
    public boolean getDriveTrainShifterButton();
    
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
