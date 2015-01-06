package org.usfirst.frc.team1318.robot.UserInterface;

import org.usfirst.frc.team1318.robot.JoystickButtonConstants;
import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.Common.IDriver;
import org.usfirst.frc.team1318.robot.Common.SmartDashboardLogger;
import org.usfirst.frc.team1318.robot.Common.ToggleButtons.MultiToggleButton;
import org.usfirst.frc.team1318.robot.Common.ToggleButtons.SimpleTimedToggleButton;
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
    private static final String COLLECTOR_EXTEND_LOG_KEY = "u.cxt";
    private static final String COLLECTOR_RETRACT_LOG_KEY = "u.cr";
    private static final String COLLECTOR_COLLECT_LOG_KEY = "u.cc";
    private static final String COLLECTOR_EXPEL_LOG_KEY = "u.cxp";
    private static final String SHOOTER_ANGLE_LOG_KEY = "u.sa";
    private static final String SHOOTER_MODE_LOG_KEY = "u.sm";
    private static final String SHOOTER_SHOOT_LOG_KEY = "u.ss";
    private static final String DRIVETRAIN_X_VELOCITY_LOG_KEY = "u.dxv";
    private static final String DRIVETRAIN_Y_VELOCITY_LOG_KEY = "u.dyv";
    private static final String DRIVETRAIN_SIMPLE_MODE_LOG_KEY = "u.dsm";

    private Joystick joystick;
    private MultiToggleButton shooterMode;
    private SimpleTimedToggleButton shootButton;

    /**
     * Initializes a new UserOperator
     */
    public UserDriver()
    {
        this.joystick = new Joystick(JoystickButtonConstants.JOYSTICK_PORT);
        this.shooterMode = new MultiToggleButton(new int[] { 3, 4, 5 });
        this.shootButton = new SimpleTimedToggleButton(TuningConstants.SHOOTER_TOGGLE_DURATION);
    }

    /**
     * Tell the operator component that some time has passed
     */
    public void update()
    {
        // check the toggles
        if (this.joystick.getRawButton(JoystickButtonConstants.SHOOTER_MODE_TOGGLE_BUTTON))
        {
            this.shooterMode.toggle();
        }

        this.shootButton.tick();
        if (this.shootButton.canToggle() && this.joystick.getRawButton(JoystickButtonConstants.SHOOTER_SHOOT_BUTTON))
        {
            this.shootButton.toggle();
        }
    }
    
    /**
     * Tell the operator component that operation is stopping
     */
    public void stop()
    {
        this.shootButton.cancel();
    }

    /**
     * Get a value indicating whether we should extend the collector 
     * @return true if we should extend, otherwise false
     */
    public boolean getCollectorExtendButton()
    {
        boolean collectorExtend = this.joystick.getRawButton(JoystickButtonConstants.COLLECTOR_EXTEND_BUTTON);

        SmartDashboardLogger.putBoolean(UserDriver.COLLECTOR_EXTEND_LOG_KEY, collectorExtend);

        return collectorExtend;
    }

    /**
     * Get a value indicating whether we should retract the collector 
     * @return true if we should retract, otherwise false
     */
    public boolean getCollectorRetractButton()
    {
        boolean collectorRetract = this.joystick.getRawButton(JoystickButtonConstants.COLLECTOR_RETRACT_BUTTON);

        SmartDashboardLogger.putBoolean(UserDriver.COLLECTOR_RETRACT_LOG_KEY, collectorRetract);

        return collectorRetract;
    }

    /**
     * Get a value indicating whether we should collect a ball using the collector 
     * @return true if we should collect, otherwise false
     */
    public boolean getCollectorCollectButton()
    {
        boolean collectorCollect = this.joystick.getRawButton(JoystickButtonConstants.COLLECTOR_COLLECT_BUTTON);

        SmartDashboardLogger.putBoolean(UserDriver.COLLECTOR_COLLECT_LOG_KEY, collectorCollect);

        return collectorCollect;
    }

    /**
     * Get a value indicating whether we should expel a ball using the collector 
     * @return true if we should expel, otherwise false
     */
    public boolean getCollectorExpelButton()
    {
        boolean collectorExpel = this.joystick.getRawButton(JoystickButtonConstants.COLLECTOR_EXPEL_BUTTON);

        SmartDashboardLogger.putBoolean(UserDriver.COLLECTOR_EXPEL_LOG_KEY, collectorExpel);

        return collectorExpel;
    }

    /**
     * Get a value indicating whether we should adjust the shooter angle 
     * @return true if we should move in, otherwise false
     */
    public boolean getShooterAngle()
    {
        boolean shooterAngle = this.joystick.getRawButton(JoystickButtonConstants.SHOOTER_EXTEND_BUTTON);

        SmartDashboardLogger.putBoolean(UserDriver.SHOOTER_ANGLE_LOG_KEY, shooterAngle);

        return shooterAngle;
    }

    /**
     * Get a value indicating the shooter's current mode 
     * @return a value indicating the number of pistons to use in the shot
     */
    public int getShooterMode()
    {
        int shooterMode = this.shooterMode.getToggledState();

        SmartDashboardLogger.putNumber(UserDriver.SHOOTER_MODE_LOG_KEY, shooterMode);

        return shooterMode;
    }

    /**
     * Get a value indicating whether we should attempt to shoot
     * @return true if we should be shooting, otherwise false
     */
    public boolean getShooterShoot()
    {
        boolean shooterShoot = this.shootButton.isToggled();

        SmartDashboardLogger.putBoolean(UserDriver.SHOOTER_SHOOT_LOG_KEY, shooterShoot);

        return shooterShoot;
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
