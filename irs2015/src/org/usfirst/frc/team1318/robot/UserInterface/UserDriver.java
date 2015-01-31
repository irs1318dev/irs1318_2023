package org.usfirst.frc.team1318.robot.UserInterface;

import org.usfirst.frc.team1318.robot.JoystickButtonConstants;
import org.usfirst.frc.team1318.robot.Common.IDriver;
import org.usfirst.frc.team1318.robot.Common.SimpleButton;
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

    //Drive Train 
    private static final String DRIVETRAIN_X_VELOCITY_LOG_KEY = "u.driveXVelocity";
    private static final String DRIVETRAIN_Y_VELOCITY_LOG_KEY = "u.driveYVelocity";
    private static final String DRIVETRAIN_SIMPLE_MODE_LOG_KEY = "u.driveSimpleMode";

    //Elevator 
    private static final String ELEVATOR_MACRO_STATE_LOG_KEY = "u.elevatorMacroState";
    private static final String ELEVATOR_HEIGHT_0_STATE_LOG_KEY = "u.elevatorHeight0";
    private static final String ELEVATOR_HEIGHT_1_STATE_LOG_KEY = "u.elevatorHeight1";
    private static final String ELEVATOR_HEIGHT_2_STATE_LOG_KEY = "u.elevatorHeight2";
    private static final String ELEVATOR_HEIGHT_3_STATE_LOG_KEY = "u.elevatorHeight3";
    private static final String ELEVATOR_HEIGHT_4_STATE_LOG_KEY = "u.elevatorHeight4";
    private static final String ELEVATOR_HEIGHT_5_STATE_LOG_KEY = "u.elevatorHeight5";
    private static final String ELEVATOR_HEIGHT_6_STATE_LOG_KEY = "u.elevatorHeight6";
    private static final String ELEVATOR_HEIGHT_7_STATE_LOG_KEY = "u.elevatorHeight7";
    private static final String ELEVATOR_OVERRIDE_STATE_LOG_KEY = "u.elevatorOverride";

    //Arm 
    private static final String ARM_MACRO_EXTEND_STATE_LOG_KEY = "u.armMacroExtendState";
    private static final String ARM_MACRO_RETRACT_STATE_LOG_KEY = "u.armMacroRetractState";
    private static final String ARM_TILT_OVERRIDE_LOG_KEY = "u.armTiltOverride";
    private static final String ARM_EXTENDER_OVERRIDE_LOG_KEY = "u.armExtenderOverride";
    private static final String ARM_TROMBONE_OVERRIDE_LOG_KEY = "u.armTromboneOverride";

    //Intake 
    private static final String INTAKE_UP_STATE_KEY = "u.intakeUpState";
    private static final String INTAKE_DOWN_STATE_KEY = "u.intakeDownState";
    private static final String INTAKE_RIGHT_TOGGLE_OVERRIDE_STATE_KEY = "u.intakeRightToggleOverrideState";
    private static final String INTAKE_LEFT_TOGGLE_OVERRIDE_STATE_KEY = "u.intakeLeftToggleOverrideState";
    private static final String INTAKE_FORWARD_STATE_KEY = "u.intakeForwardStateKey";
    private static final String INTAKE_BACKWARD_STATE_KEY = "u.intakeBackwardStateKey";

    private Joystick joystick;

    private SimpleToggleButton simpleDriveModeButton;

    //Arm
    private SimpleButton armMacroExtendButton;
    private SimpleButton armMacroRetractButton;
    private SimpleToggleButton armExtenderToggleOverride;
    private SimpleToggleButton armTiltToggleOverride;
    private SimpleToggleButton armTromboneToggleOverride;

    //Intake
    private SimpleButton intakeUpButton;
    private SimpleButton intakeDownButton;
    private SimpleToggleButton intakeRightToggleOverride;
    private SimpleToggleButton intakeLeftToggleOverride;

    /**
     * Initializes a new UserDriver
     */
    public UserDriver()
    {
        this.joystick = new Joystick(JoystickButtonConstants.JOYSTICK_PORT);

        // initialize various toggle buttons
        this.simpleDriveModeButton = new SimpleToggleButton();

        //Arm
        this.armMacroExtendButton = new SimpleButton();
        this.armMacroRetractButton = new SimpleButton();
        this.armExtenderToggleOverride = new SimpleToggleButton();
        this.armTiltToggleOverride = new SimpleToggleButton();
        this.armTromboneToggleOverride = new SimpleToggleButton();

        //Intake
        this.intakeUpButton = new SimpleButton();
        this.intakeDownButton = new SimpleButton();
        this.intakeRightToggleOverride = new SimpleToggleButton();
        this.intakeLeftToggleOverride = new SimpleToggleButton();
    }

    /**
     * Tell the driver component that some time has passed
     */
    public void update()
    {
        // update the state of the various toggle buttons
        this.simpleDriveModeButton.updateState(this.joystick.getRawButton(JoystickButtonConstants.DRIVETRAIN_SIMPLE_BUTTON));

        //Arm 
        this.armMacroExtendButton.updateState(this.joystick.getRawButton(JoystickButtonConstants.ARM_MACRO_EXTEND_BUTTON));
        this.armMacroRetractButton.updateState(this.joystick.getRawButton(JoystickButtonConstants.ARM_MACRO_RETRACT_BUTTON));
        this.armExtenderToggleOverride.updateState(this.joystick.getRawButton(JoystickButtonConstants.ARM_EXTENDER_BUTTON));
        this.armTiltToggleOverride.updateState(this.joystick.getRawButton(JoystickButtonConstants.ARM_TILT_BUTTON));
        this.armTromboneToggleOverride.updateState(this.joystick.getRawButton(JoystickButtonConstants.ARM_TROMBONE_BUTTON));

        //Intake
        this.intakeUpButton.updateState(this.joystick.getRawButton(JoystickButtonConstants.INTAKE_UP_BUTTON));
        this.intakeDownButton.updateState(this.joystick.getRawButton(JoystickButtonConstants.INTAKE_DOWN_BUTTON));
        this.intakeRightToggleOverride.updateState(this.joystick.getRawButton(JoystickButtonConstants.INTAKE_RIGHT_TOGGLE_OVERRIDE));
        this.intakeLeftToggleOverride.updateState(this.joystick.getRawButton(JoystickButtonConstants.INTAKE_LEFT_TOGGLE_OVERRIDE));

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

    //=================================================== Elevator ===============================================================

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
        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_HEIGHT_0_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorHeight1Button()
    {
        boolean state = this.joystick.getRawButton(JoystickButtonConstants.ELEVATOR_HEIGHT_1_BUTTON);
        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_HEIGHT_1_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorHeight2Button()
    {
        boolean state = this.joystick.getRawButton(JoystickButtonConstants.ELEVATOR_HEIGHT_2_BUTTON);
        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_HEIGHT_2_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorHeight3Button()
    {
        boolean state = this.joystick.getRawButton(JoystickButtonConstants.ELEVATOR_HEIGHT_3_BUTTON);
        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_HEIGHT_3_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorHeight4Button()
    {
        boolean state = this.joystick.getRawButton(JoystickButtonConstants.ELEVATOR_HEIGHT_4_BUTTON);
        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_HEIGHT_4_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorHeight5Button()
    {
        boolean state = this.joystick.getRawButton(JoystickButtonConstants.ELEVATOR_HEIGHT_5_BUTTON);
        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_HEIGHT_5_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorHeight6Button()
    {
        boolean state = this.joystick.getRawButton(JoystickButtonConstants.ELEVATOR_HEIGHT_6_BUTTON);
        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_HEIGHT_6_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorHeight7Button()
    {
        boolean state = this.joystick.getRawButton(JoystickButtonConstants.ELEVATOR_HEIGHT_7_BUTTON);
        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_HEIGHT_7_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public double getElevatorOverride()
    {
        double speed = this.joystick.getZ();
        SmartDashboardLogger.putNumber(UserDriver.ELEVATOR_OVERRIDE_STATE_LOG_KEY, speed);
        return speed;
    }

    //===================================================== Arm =================================================================

    @Override
    public boolean getArmMacroExtendButton()
    {
        boolean mode = this.armMacroExtendButton.isActivated();
        SmartDashboardLogger.putBoolean(UserDriver.ARM_MACRO_EXTEND_STATE_LOG_KEY, mode);
        return mode;
    }

    @Override
    public boolean getArmMacroRetractButton()
    {
        boolean mode = this.armMacroRetractButton.isActivated();
        SmartDashboardLogger.putBoolean(UserDriver.ARM_MACRO_RETRACT_STATE_LOG_KEY, mode);
        return mode;
    }

    @Override
    public boolean getArmExtenderToggleOverride()
    {
        boolean mode = this.armExtenderToggleOverride.isToggled();
        SmartDashboardLogger.putBoolean(UserDriver.ARM_EXTENDER_OVERRIDE_LOG_KEY, mode);
        return mode;
    }

    @Override
    public boolean getArmTiltToggleOverride()
    {
        boolean mode = this.armTiltToggleOverride.isToggled();
        SmartDashboardLogger.putBoolean(UserDriver.ARM_TILT_OVERRIDE_LOG_KEY, mode);
        return mode;
    }

    @Override
    public boolean getArmTromboneToggleOverride()
    {
        boolean mode = this.armTromboneToggleOverride.isToggled();
        SmartDashboardLogger.putBoolean(UserDriver.ARM_TROMBONE_OVERRIDE_LOG_KEY, mode);
        return mode;
    }

    //=================================================== Intake ================================================================

    @Override
    public boolean getIntakeUpButton()
    {
        boolean mode = this.intakeUpButton.isActivated();
        SmartDashboardLogger.putBoolean(UserDriver.INTAKE_UP_STATE_KEY, mode);
        return mode;
    }

    @Override
    public boolean getIntakeDownButton()
    {
        boolean mode = this.intakeDownButton.isActivated();
        SmartDashboardLogger.putBoolean(UserDriver.INTAKE_DOWN_STATE_KEY, mode);
        return mode;
    }

    @Override
    public boolean getIntakeRightToggleOverride()
    {
        boolean mode = this.intakeRightToggleOverride.isToggled();
        SmartDashboardLogger.putBoolean(UserDriver.INTAKE_RIGHT_TOGGLE_OVERRIDE_STATE_KEY, mode);
        return mode;
    }

    @Override
    public boolean getIntakeLeftToggleOverride()
    {
        boolean mode = this.intakeLeftToggleOverride.isToggled();
        SmartDashboardLogger.putBoolean(UserDriver.INTAKE_LEFT_TOGGLE_OVERRIDE_STATE_KEY, mode);
        return mode;
    }

    @Override
    public boolean getIntakeForwardButton()
    {
        boolean mode = this.joystick.getRawButton(JoystickButtonConstants.INTAKE_FORWARD_BUTTON);
        SmartDashboardLogger.putBoolean(UserDriver.INTAKE_FORWARD_STATE_KEY, mode);
        return mode;
    }

    @Override
    public boolean getIntakeBackwardButton()
    {
        boolean mode = this.joystick.getRawButton(JoystickButtonConstants.INTAKE_BACKWARD_BUTTON);
        SmartDashboardLogger.putBoolean(UserDriver.INTAKE_BACKWARD_STATE_KEY, mode);
        return mode;
    }

}
