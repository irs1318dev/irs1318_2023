package org.usfirst.frc.team1318.robot.UserInterface;

import org.usfirst.frc.team1318.robot.JoystickButtonConstants;
import org.usfirst.frc.team1318.robot.Common.IDriver;
import org.usfirst.frc.team1318.robot.Common.SmartDashboardLogger;
import edu.wpi.first.wpilibj.*;

/**
 * Driver for teleop mode.  User driver translates current toggle state and joystick state into
 * the specific actions that should be taken by the robot.
 * 
 * @author Will
 *
 */
public class UserDriver implements IDriver
{
    // logging constants
    private static final String DRIVETRAIN_X_VELOCITY_LOG_KEY = "u.dxv";
    private static final String DRIVETRAIN_Y_VELOCITY_LOG_KEY = "u.dyv";
    private static final String DRIVETRAIN_SIMPLE_MODE_LOG_KEY = "u.dsm";

    private Joystick joystick;

    /**
     * Initializes a new UserOperator
     */
    public UserDriver()
    {
        this.joystick = new Joystick(JoystickButtonConstants.JOYSTICK_PORT);
        
        // instantiate toggles here
    }

    /**
     * Tell the operator component that some time has passed
     */
    public void update()
    {
        // check and update the toggles
    }
    
    /**
     * Tell the operator component that operation is stopping
     */
    public void stop()
    {
    }

    /**
     * Get a value indicating the desired drive train X Velocity 
     * @return value between -1.0 and 1.0 (percentage of max right turn velocity)
     */
    public double getDriveTrainXVelocity()
    {
        double xVelocity = this.joystick.getX();

        SmartDashboardLogger.putNumber(UserDriver.DRIVETRAIN_X_VELOCITY_LOG_KEY, xVelocity);

        return xVelocity;
    }

    /**
     * Get a value indicating the desired drive train Y velocity (turn amount) 
     * @return value between -1.0 and 1.0 (percentage of max forward velocity)
     */
    public double getDriveTrainYVelocity()
    {
        double yVelocity = this.joystick.getY();

        SmartDashboardLogger.putNumber(UserDriver.DRIVETRAIN_Y_VELOCITY_LOG_KEY, yVelocity);

        return yVelocity;
    }

    /**
     * Get a value indicating whether we should be using the drive train in simple mode 
     * @return true if we should be in simple mode, otherwise false
     */
    public boolean getDriveTrainSimpleModeButton()
    {
        boolean simpleMode = this.joystick.getRawButton(JoystickButtonConstants.DRIVETRAIN_SIMPLE_BUTTON);

        SmartDashboardLogger.putBoolean(UserDriver.DRIVETRAIN_SIMPLE_MODE_LOG_KEY, simpleMode);

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
