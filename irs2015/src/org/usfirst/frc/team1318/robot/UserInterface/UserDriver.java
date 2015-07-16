package org.usfirst.frc.team1318.robot.UserInterface;

import org.usfirst.frc.team1318.robot.JoystickButtonConstants;
import org.usfirst.frc.team1318.robot.Common.IDriver;
import org.usfirst.frc.team1318.robot.Common.ToggleButtons.SimpleToggleButton;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Driver for teleop mode.  User driver translates current toggle state and joystick state into
 * the specific actions that should be taken by the robot.
 * 
 * @author Will
 *
 */
public class UserDriver implements IDriver
{
    private Joystick joystickDriver;
    private Joystick joystickCoDriver;

    // DriveTrain toggles
    private final SimpleToggleButton simpleDriveModeButton;

    /**
     * Initializes a new UserDriver
     */
    public UserDriver()
    {
        this.joystickDriver = new Joystick(JoystickButtonConstants.JOYSTICK_DRIVER_PORT);
        this.joystickCoDriver = new Joystick(JoystickButtonConstants.JOYSTICK_CO_DRIVER_PORT);

        // initialize DriveTrain toggles
        this.simpleDriveModeButton = new SimpleToggleButton();
    }

    /**
     * Tell the driver component that some time has passed
     */
    public void update()
    {
        // update the state of the various toggle buttons
        updateDriveTrain();
    }

    private void updateDriveTrain()
    {
        //        this.simpleDriveModeButton.updateState(this.joystickDriver.getRawButton(JoystickButtonConstants.DRIVETRAIN_SIMPLE_BUTTON));
    }

    /**
     * Tell the driver that operation is stopping
     */
    public void stop()
    {
    }

    //================================================== DriveTrain ==============================================================

    /**
     * Get a value indicating the desired drive train X Velocity 
     * @return value between -1.0 and 1.0 (percentage of max right turn velocity)
     */
    public double getDriveTrainXVelocity()
    {
        double xVelocity = this.joystickDriver.getX();

        if (JoystickButtonConstants.INVERT_X_AXIS)
        {
            xVelocity = -xVelocity;
        }

        //        SmartDashboardLogger.putNumber(UserDriver.DRIVETRAIN_X_VELOCITY_LOG_KEY, xVelocity);

        return xVelocity;
    }

    /**
     * Get a value indicating the desired drive train Y velocity (turn amount) 
     * @return value between -1.0 and 1.0 (percentage of max forward velocity)
     */
    public double getDriveTrainYVelocity()
    {
        double yVelocity = this.joystickDriver.getY();

        if (JoystickButtonConstants.INVERT_Y_AXIS)
        {
            yVelocity = -yVelocity;
        }

        //        SmartDashboardLogger.putNumber(UserDriver.DRIVETRAIN_Y_VELOCITY_LOG_KEY, yVelocity);

        return yVelocity;
    }

    /**
     * Get a value indicating whether we should be using the drive train in simple mode 
     * @return true if we should be in simple mode, otherwise false
     */
    public boolean getDriveTrainSimpleMode()
    {
        boolean simpleMode = this.simpleDriveModeButton.isToggled();

        //        SmartDashboardLogger.putBoolean(UserDriver.DRIVETRAIN_SIMPLE_MODE_LOG_KEY, simpleMode);

        return simpleMode;
    }

    /**
     * Get a value indicating the desired drive train left position for positional mode
     * @return position
     */
    public double getDriveTrainLeftPosition()
    {
        // position mode is only used for autonomous
        return 0.0;
    }

    /**
     * Get a value indicating the desired drive train right position for positional mode
     * @return position
     */
    public double getDriveTrainRightPosition()
    {
        // position mode is only used for autonomous
        return 0.0;
    }

    /**
     * Get a value indicating whether the drive train should be in position or velocity mode
     * @return true if position mode, false if velocity mode
     */
    public boolean getDriveTrainPositionMode()
    {
        // position mode is only used for autonomous
        return false;
    }
}
