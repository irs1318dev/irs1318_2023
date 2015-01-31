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
    private static final String ELEVATOR_CONTAINER_MACRO_STATE_LOG_KEY = "u.elevatorContainerMacroState";
    private static final String ELEVATOR_SET_STATE_TO_FLOOR_LOG_KEY = "u.elevatorSetStateToFloor";
    private static final String ELEVATOR_SET_STATE_TO_PLATFORM_LOG_KEY = "u.elevatorSetStateToPlatform";
    private static final String ELEVATOR_SET_STATE_TO_STEP_LOG_KEY = "u.elevatorSetStateToStep";
    private static final String ELEVATOR_MOVE_TO_0_TOTES_LOG_KEY = "u.elevatorMoveTo0Totes";
    private static final String ELEVATOR_MOVE_TO_1_TOTE_STATE_LOG_KEY = "u.elevatorMoveTo1Tote";
    private static final String ELEVATOR_MOVE_TO_2_TOTES_STATE_LOG_KEY = "u.elevatorMoveTo2Totes";
    private static final String ELEVATOR_MOVE_TO_3_TOTES_STATE_LOG_KEY = "u.elevatorMoveTo3Totes";
    private static final String ELEVATOR_PID_TOGGLE_STATE_LOG_KEY = "u.elevatorPIDToggleState";
    private static final String ELEVATOR_STOP_LOG_KEY = "u.elevatorStop";
    private static final String ELEVATOR_UP_STATE_LOG_KEY = "u.elevatorUp";
    private static final String ELEVATOR_DOWN_STATE_LOG_KEY = "u.elevatorDown";

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

    //Elevator 
    private SimpleButton elevatorContainerMacroButton;
    private SimpleButton elevatorSetStateToFloorButton;
    private SimpleButton elevatorSetStateToPlatformButton;
    private SimpleButton elevatorSetStateToStepButton;
    private SimpleButton elevatorMoveTo0Totes;
    private SimpleButton elevatorMoveTo1Tote;
    private SimpleButton elevatorMoveTo2Totes;
    private SimpleButton elevatorMoveTo3Totes;
    private SimpleToggleButton elevatorPIDToggle;
    private SimpleButton elevatorStop;
    private SimpleButton elevatorUp;
    private SimpleButton elevatorDown;

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

        //Elevator
        this.elevatorContainerMacroButton = new SimpleButton();
        this.elevatorSetStateToFloorButton = new SimpleButton();
        this.elevatorSetStateToPlatformButton = new SimpleButton();
        this.elevatorSetStateToStepButton = new SimpleButton();
        this.elevatorMoveTo0Totes = new SimpleButton();
        this.elevatorMoveTo1Tote = new SimpleButton();
        this.elevatorMoveTo2Totes = new SimpleButton();
        this.elevatorMoveTo3Totes = new SimpleButton();
        this.elevatorPIDToggle = new SimpleToggleButton();
        this.elevatorStop = new SimpleButton();
        this.elevatorUp = new SimpleButton();
        this.elevatorDown = new SimpleButton();
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

        //Elevator
        this.elevatorContainerMacroButton.updateState(this.joystick
            .getRawButton(JoystickButtonConstants.ELEVATOR_CONTAINER_MACRO_BUTTON));
        this.elevatorSetStateToFloorButton.updateState(this.joystick
            .getRawButton(JoystickButtonConstants.ELEVATOR_SET_STAE_TO_FLOOR_BUTTON));
        this.elevatorSetStateToPlatformButton.updateState(this.joystick
            .getRawButton(JoystickButtonConstants.ELEVATOR_SET_STATE_TO_PLATFORM_BUTTON));
        this.elevatorSetStateToStepButton.updateState(this.joystick
            .getRawButton(JoystickButtonConstants.ELEVATOR_SET_STATE_TO_STEP_BUTTON));
        this.elevatorMoveTo0Totes.updateState(this.joystick.getRawButton(JoystickButtonConstants.ELEVATOR_MOVE_TO_0_TOTES_BUTTON));
        this.elevatorMoveTo1Tote.updateState(this.joystick.getRawButton(JoystickButtonConstants.ELEVATOR_MOVE_TO_1_TOTE_BUTTON));
        this.elevatorMoveTo2Totes.updateState(this.joystick.getRawButton(JoystickButtonConstants.ELEVATOR_MOVE_TO_2_TOTES_BUTTON));
        this.elevatorMoveTo3Totes.updateState(this.joystick.getRawButton(JoystickButtonConstants.ELEVATOR_MOVE_TO_3_TOTES_BUTTON));
        this.elevatorPIDToggle.updateState(this.joystick.getRawButton(JoystickButtonConstants.ELEVATOR_PID_TOGGLE));
        this.elevatorStop.updateState(this.joystick.getRawButton(JoystickButtonConstants.ELEVATOR_STOP_BUTTON));
        this.elevatorUp.updateState(this.joystick.getRawButton(JoystickButtonConstants.ELEVATOR_UP_BUTTON));
        this.elevatorDown.updateState(this.joystick.getRawButton(JoystickButtonConstants.ELEVATOR_DOWN_BUTTON));

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
    public boolean getElevatorContainerMacroButton()
    {
        boolean state = this.elevatorContainerMacroButton.isActivated();
        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_CONTAINER_MACRO_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorSetStateToFloorButton()
    {
        boolean state = this.elevatorSetStateToFloorButton.isActivated();
        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_SET_STATE_TO_FLOOR_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorSetStateToPlatformButton()
    {
        boolean state = this.elevatorSetStateToPlatformButton.isActivated();
        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_SET_STATE_TO_PLATFORM_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorSetStateToStepButton()
    {
        boolean state = this.elevatorSetStateToStepButton.isActivated();
        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_SET_STATE_TO_STEP_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorMoveTo0TotesButton()
    {
        boolean state = this.elevatorMoveTo0Totes.isActivated();
        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_MOVE_TO_0_TOTES_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorMoveTo1ToteButton()
    {
        boolean state = this.elevatorMoveTo1Tote.isActivated();
        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_MOVE_TO_1_TOTE_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorMoveTo2TotesButton()
    {
        boolean state = this.elevatorMoveTo2Totes.isActivated();
        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_MOVE_TO_2_TOTES_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorMoveTo3TotesButton()
    {
        boolean state = this.elevatorMoveTo3Totes.isActivated();
        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_MOVE_TO_3_TOTES_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorPIDToggle()
    {
        boolean state = this.elevatorPIDToggle.isToggled();
        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_PID_TOGGLE_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getStopElevatorButton()
    {
        boolean mode = this.elevatorStop.isActivated();
        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_STOP_LOG_KEY, mode);
        return mode;
    }

    @Override
    public boolean getElevatorUpButton()
    {
        boolean mode = this.elevatorUp.isActivated();
        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_UP_STATE_LOG_KEY, mode);
        return mode;
    }

    @Override
    public boolean getElevatorDownButton()
    {
        boolean mode = this.elevatorDown.isActivated();
        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_DOWN_STATE_LOG_KEY, mode);
        return mode;
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
