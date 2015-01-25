package org.usfirst.frc.team1318.robot.UserInterface;

import org.usfirst.frc.team1318.robot.JoystickButtonConstants;
import org.usfirst.frc.team1318.robot.Common.IDriver;
import org.usfirst.frc.team1318.robot.Common.SmartDashboardLogger;
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
    // logging constants
    private static final String DRIVETRAIN_X_VELOCITY_LOG_KEY = "u.driveXVelocity";
    private static final String DRIVETRAIN_Y_VELOCITY_LOG_KEY = "u.driveYVelocity";
    private static final String DRIVETRAIN_SIMPLE_MODE_LOG_KEY = "u.driveSimpleMode";

    private static final String ELEVATOR_MACRO_STATE_LOG_KEY = "u.elevatorMacroState";
    private static final String ELEVATOR_HEIGHT_0_STATE = "u.elevatorHeight0";
    private static final String ELEVATOR_HEIGHT_1_STATE = "u.elevatorHeight1";
    private static final String ELEVATOR_HEIGHT_2_STATE = "u.elevatorHeight2";
    private static final String ELEVATOR_HEIGHT_3_STATE = "u.elevatorHeight3";
    private static final String ELEVATOR_HEIGHT_4_STATE = "u.elevatorHeight4";
    private static final String ELEVATOR_HEIGHT_5_STATE = "u.elevatorHeight5";
    private static final String ELEVATOR_HEIGHT_6_STATE = "u.elevatorHeight6";
    private static final String ELEVATOR_HEIGHT_7_STATE = "u.elevatorHeight7";
    private static final String ELEVATOR_OVERRIDE_STATE = "u.elevatorOverride";

    private Joystick joystick;

    private SimpleToggleButton simpleDriveModeButton;

    /**
     * Initializes a new UserDriver
     */
    public UserDriver()
    {
        this.joystick = new Joystick(JoystickButtonConstants.JOYSTICK_PORT);

        // initialize various toggle buttons
        this.simpleDriveModeButton = new SimpleToggleButton();
    }

    /**
     * Tell the driver component that some time has passed
     */
    public void update()
    {
        // update the state of the various toggle buttons
        this.simpleDriveModeButton.updateState(this.joystick.getRawButton(JoystickButtonConstants.DRIVETRAIN_SIMPLE_BUTTON));
    }

    /**
     * Tell the driver that operation is stopping
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

        if (JoystickButtonConstants.INVERT_X_AXIS)
        {
            xVelocity = -xVelocity;
        }

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

        if (JoystickButtonConstants.INVERT_Y_AXIS)
        {
            yVelocity = -yVelocity;
        }

        SmartDashboardLogger.putNumber(UserDriver.DRIVETRAIN_Y_VELOCITY_LOG_KEY, yVelocity);

        return yVelocity;
    }

    /**
     * Get a value indicating whether we should be using the drive train in simple mode 
     * @return true if we should be in simple mode, otherwise false
     */
    public boolean getDriveTrainSimpleMode()
    {
        boolean simpleMode = this.simpleDriveModeButton.isToggled();

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

    @Override
    public boolean getElevatorMacroButton()
    {
        boolean state = this.joystick.getRawButton(JoystickButtonConstants.ELEVATOR_MACRO_BUTTON);
        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_MACRO_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorHeight0Button()
    {
        boolean state = this.joystick.getRawButton(JoystickButtonConstants.ELEVATOR_HEIGHT_0_BUTTON);
        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_HEIGHT_0_STATE, state);
        return state;
    }

    @Override
    public boolean getElevatorHeight1Button()
    {
        boolean state = this.joystick.getRawButton(JoystickButtonConstants.ELEVATOR_HEIGHT_1_BUTTON);
        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_HEIGHT_1_STATE, state);
        return state;
    }

    @Override
    public boolean getElevatorHeight2Button()
    {
        boolean state = this.joystick.getRawButton(JoystickButtonConstants.ELEVATOR_HEIGHT_2_BUTTON);
        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_HEIGHT_2_STATE, state);
        return state;
    }

    @Override
    public boolean getElevatorHeight3Button()
    {
        boolean state = this.joystick.getRawButton(JoystickButtonConstants.ELEVATOR_HEIGHT_3_BUTTON);
        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_HEIGHT_3_STATE, state);
        return state;
    }

    @Override
    public boolean getElevatorHeight4Button()
    {
        boolean state = this.joystick.getRawButton(JoystickButtonConstants.ELEVATOR_HEIGHT_4_BUTTON);
        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_HEIGHT_4_STATE, state);
        return state;
    }

    @Override
    public boolean getElevatorHeight5Button()
    {
        boolean state = this.joystick.getRawButton(JoystickButtonConstants.ELEVATOR_HEIGHT_5_BUTTON);
        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_HEIGHT_5_STATE, state);
        return state;
    }

    @Override
    public boolean getElevatorHeight6Button()
    {
        boolean state = this.joystick.getRawButton(JoystickButtonConstants.ELEVATOR_HEIGHT_6_BUTTON);
        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_HEIGHT_6_STATE, state);
        return state;
    }

    @Override
    public boolean getElevatorHeight7Button()
    {
        boolean state = this.joystick.getRawButton(JoystickButtonConstants.ELEVATOR_HEIGHT_7_BUTTON);
        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_HEIGHT_7_STATE, state);
        return state;
    }

    @Override
    public double getElevatorOverride()
    {
        double speed = this.joystick.getZ();
        SmartDashboardLogger.putNumber(UserDriver.ELEVATOR_OVERRIDE_STATE, speed);
        return speed;
    }
}
