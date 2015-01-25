package org.usfirst.frc.team1318.robot.Common;

/**
 * The "driver" describes the currently requested actions that the robot should be performing.  The driver could be either
 * a user controlling the robot with a joystick, or autonomous mode controlling the robot based on a queue of predetermined
 * tasks which each have their own lifecycle.
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

    //================================================== DriveTrain ==============================================================
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
    public boolean getDriveTrainSimpleMode();

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

    //=================================================== Elevator ===============================================================
    /**
     * Get a value indicating whether the robot should start the elevator macro 
     * @return true if start macro, otherwise false  
     */
    public boolean getElevatorMacroButton();

    /**
     * Get a value indicating whether the elevator should move to the 0 height setting 
     * @return true if move to 0 height setting, otherwise false
     */
    public boolean getElevatorHeight0Button();

    /**
     * Get a value indicating whether the elevator should move to the 1 height setting 
     * @return true if move to 1 height setting, otherwise false
     */
    public boolean getElevatorHeight1Button();

    /**
     * Get a value indicating whether the elevator should move to the 2 height setting 
     * @return true if move to 2 height setting, otherwise false
     */
    public boolean getElevatorHeight2Button();

    /**
     * Get a value indicating whether the elevator should move to the 3 height setting 
     * @return true if move to 3 height setting, otherwise false
     */
    public boolean getElevatorHeight3Button();

    /**
     * Get a value indicating whether the elevator should move to the 4 height setting 
     * @return true if move to 4 height setting, otherwise false
     */
    public boolean getElevatorHeight4Button();

    /**
     * Get a value indicating whether the elevator should move to the 5 height setting 
     * @return true if move to 5 height setting, otherwise false
     */
    public boolean getElevatorHeight5Button();

    /**
     * Get a value indicating whether the elevator should move to the 6 height setting 
     * @return true if move to 6 height setting, otherwise false
     */
    public boolean getElevatorHeight6Button();

    /**
     * Get a value indicating whether the elevator should move to the 7 height setting 
     * @return true if move to 7 height setting, otherwise false
     */
    public boolean getElevatorHeight7Button();

    /**
     * Get a value indicating the desired speed of the elevator 
     * @return the desired speed of the elevator 
     */
    public double getElevatorOverride();

}
