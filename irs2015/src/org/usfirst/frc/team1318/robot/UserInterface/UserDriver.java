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

    // Drive Train 
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
    private static final String ELEVATOR_PID_ON_STATE_LOG_KEY = "u.elevatorPIDOnState";
    private static final String ELEVATOR_PID_OFF_STATE_LOG_KEY = "u.elevatorPIDOffState";
    private static final String ELEVATOR_STOP_LOG_KEY = "u.elevatorStop";
    private static final String ELEVATOR_UP_STATE_LOG_KEY = "u.elevatorUp";
    private static final String ELEVATOR_DOWN_STATE_LOG_KEY = "u.elevatorDown";
    private static final String ELEVATOR_MOVE_TO_BOTTOM_LOG_KEY = "u.elevatorMoveToBottom";
    private static final String ELEVATOR_VELOCITY_OVERRIDE_LOG_KEY = "u.elevatorVelocityOverride";
    private static final String ELEVATOR_IGNORE_SENSORS_LOG_KEY = "u.elevatorIgnoreSensors";
    private static final String ELEVATOR_USE_SENSORS_LOG_KEY = "u.elevatorUseSensors";
    private static final String ELEVATOR_ZERO_ENCODERS_LOG_KEY = "u.elevatorZeroEncoders";

    //Arm 
    private static final String ARM_MACRO_EXTEND_STATE_LOG_KEY = "u.armMacroExtendState";
    private static final String ARM_MACRO_RETRACT_STATE_LOG_KEY = "u.armMacroRetractState";
    private static final String ARM_TILT_EXTEND_OVERRIDE_LOG_KEY = "u.armTiltExtendOverride";
    private static final String ARM_TILT_RETRACT_OVERRIDE_LOG_KEY = "u.armTiltRetractOverride";
    private static final String ARM_EXTENDER_EXTEND_OVERRIDE_LOG_KEY = "u.armExtenderExtendOverride";
    private static final String ARM_EXTENDER_RETRACT_OVERRIDE_LOG_KEY = "u.armExtenderRetractOverride";
    private static final String ARM_TROMBONE_EXTEND_OVERRIDE_LOG_KEY = "u.armTromboneExtendOverride";
    private static final String ARM_TROMBONE_RETRACT_OVERRIDE_LOG_KEY = "u.armTromboneRetractOverride";

    //Intake 
    private static final String INTAKE_UP_STATE_KEY = "u.intakeUpState";
    private static final String INTAKE_DOWN_STATE_KEY = "u.intakeDownState";
    private static final String INTAKE_RIGHT_EXTEND_OVERRIDE_STATE_KEY = "u.intakeRightExtendOverrideState";
    private static final String INTAKE_RIGHT_RETRACT_OVERRIDE_STATE_KEY = "u.intakeRightRetractOverrideState";
    private static final String INTAKE_LEFT_EXTEND_OVERRIDE_STATE_KEY = "u.intakeLeftExtendOverrideState";
    private static final String INTAKE_LEFT_RETRACT_OVERRIDE_STATE_KEY = "u.intakeLeftRetractOverrideState";
    private static final String INTAKE_FORWARD_STATE_KEY = "u.intakeForwardStateKey";
    private static final String INTAKE_BACKWARD_STATE_KEY = "u.intakeBackwardStateKey";

    private Joystick joystickDriver;
    private Joystick joystickCoDriver;

    // DriveTrain toggles
    private final SimpleToggleButton simpleDriveModeButton;

    //Arm
    private SimpleButton armMacroExtendButton;
    private SimpleButton armMacroRetractButton;
    private SimpleButton armExtenderExtendOverride;
    private SimpleButton armExtenderRetractOverride;
    private SimpleButton armTiltExtendOverride;
    private SimpleButton armTiltRetractOverride;
    private SimpleButton armTromboneExtendOverride;
    private SimpleButton armTromboneRetractOverride;

    //Intake
    private SimpleButton intakeUpButton;
    private SimpleButton intakeDownButton;
    private SimpleButton intakeRightExtendOverride;
    private SimpleButton intakeRightRetractOverride;
    private SimpleButton intakeLeftExtendOverride;
    private SimpleButton intakeLeftRetractOverride;

    //Elevator 
    private SimpleButton elevatorContainerMacroButton;
    private SimpleButton elevatorSetStateToFloorButton;
    private SimpleButton elevatorSetStateToPlatformButton;
    private SimpleButton elevatorSetStateToStepButton;
    private SimpleButton elevatorMoveTo0Totes;
    private SimpleButton elevatorMoveTo1Tote;
    private SimpleButton elevatorMoveTo2Totes;
    private SimpleButton elevatorMoveTo3Totes;
    private SimpleButton elevatorPIDOn;
    private SimpleButton elevatorPIDOff;
    private SimpleButton elevatorStop;
    private SimpleButton elevatorMoveToBottom;
    private SimpleButton elevatorIgnoreSensors;
    private SimpleButton elevatorUseSensors;
    private SimpleButton elevatorZeroEncoders;

    /**
     * Initializes a new UserDriver
     */
    public UserDriver()
    {
        this.joystickDriver = new Joystick(JoystickButtonConstants.JOYSTICK_DRIVER_PORT);
        this.joystickCoDriver = new Joystick(JoystickButtonConstants.JOYSTICK_CO_DRIVER_PORT);

        // initialize DriveTrain toggles
        this.simpleDriveModeButton = new SimpleToggleButton();

        //Arm
        this.armMacroExtendButton = new SimpleButton();
        this.armMacroRetractButton = new SimpleButton();
        this.armExtenderExtendOverride = new SimpleButton();
        this.armExtenderRetractOverride = new SimpleButton();
        this.armTiltExtendOverride = new SimpleButton();
        this.armTiltRetractOverride = new SimpleButton();
        this.armTromboneExtendOverride = new SimpleButton();
        this.armTromboneRetractOverride = new SimpleButton();

        //Intake
        this.intakeUpButton = new SimpleButton();
        this.intakeDownButton = new SimpleButton();
        this.intakeRightExtendOverride = new SimpleButton();
        this.intakeRightRetractOverride = new SimpleButton();
        this.intakeLeftExtendOverride = new SimpleButton();
        this.intakeLeftRetractOverride = new SimpleButton();

        //Elevator
        this.elevatorContainerMacroButton = new SimpleButton();
        this.elevatorSetStateToFloorButton = new SimpleButton();
        this.elevatorSetStateToPlatformButton = new SimpleButton();
        this.elevatorSetStateToStepButton = new SimpleButton();
        this.elevatorMoveTo0Totes = new SimpleButton();
        this.elevatorMoveTo1Tote = new SimpleButton();
        this.elevatorMoveTo2Totes = new SimpleButton();
        this.elevatorMoveTo3Totes = new SimpleButton();
        this.elevatorPIDOn = new SimpleButton();
        this.elevatorPIDOff = new SimpleButton();
        this.elevatorStop = new SimpleButton();
        this.elevatorIgnoreSensors = new SimpleButton();
        this.elevatorUseSensors = new SimpleButton();
        this.elevatorMoveToBottom = new SimpleButton();
        this.elevatorZeroEncoders = new SimpleButton();
    }

    /**
     * Tell the driver component that some time has passed
     */
    public void update()
    {
        // update the state of the various toggle buttons

        // DriveTrain
        this.simpleDriveModeButton.updateState(this.joystickDriver.getRawButton(JoystickButtonConstants.DRIVETRAIN_SIMPLE_BUTTON));

        //Arm 
        this.armMacroExtendButton.updateState(this.joystickDriver.getRawButton(JoystickButtonConstants.ARM_MACRO_EXTEND_BUTTON));
        this.armMacroRetractButton.updateState(this.joystickDriver.getRawButton(JoystickButtonConstants.ARM_MACRO_RETRACT_BUTTON));
        //        this.armExtenderExtendOverride.updateState(this.joystickCoDriver.getRawButton(JoystickButtonConstants.ARM_EXTENDER_EXTEND_BUTTON));
        //        this.armExtenderRetractOverride
        //            .updateState(this.joystickCoDriver.getRawButton(JoystickButtonConstants.ARM_EXTENDER_RETRACT_BUTTON));
        //        this.armTiltExtendOverride.updateState(this.joystickCoDriver.getRawButton(JoystickButtonConstants.ARM_TILT_EXTEND_BUTTON));
        //        this.armTiltRetractOverride.updateState(this.joystickCoDriver.getRawButton(JoystickButtonConstants.ARM_TILT_RETRACT_BUTTON));
        //        this.armTromboneExtendOverride.updateState(this.joystickCoDriver.getRawButton(JoystickButtonConstants.ARM_TROMBONE_EXTEND_BUTTON));
        //        this.armTromboneRetractOverride
        //            .updateState(this.joystickCoDriver.getRawButton(JoystickButtonConstants.ARM_TROMBONE_RETRACT_BUTTON));

        //Intake
        this.intakeUpButton.updateState(this.joystickDriver.getRawButton(JoystickButtonConstants.INTAKE_UP_BUTTON));
        this.intakeDownButton.updateState(this.joystickDriver.getRawButton(JoystickButtonConstants.INTAKE_DOWN_BUTTON));
        //        this.intakeRightExtendOverride
        //            .updateState(this.joystickCoDriver.getRawButton(JoystickButtonConstants.INTAKE_RIGHT_EXTEND_OVERRIDE));
        //        this.intakeRightRetractOverride
        //            .updateState(this.joystickCoDriver.getRawButton(JoystickButtonConstants.INTAKE_RIGHT_RETRACT_OVERRIDE));
        //        this.intakeLeftExtendOverride.updateState(this.joystickCoDriver.getRawButton(JoystickButtonConstants.INTAKE_LEFT_EXTEND_OVERRIDE));
        //        this.intakeLeftRetractOverride
        //            .updateState(this.joystickCoDriver.getRawButton(JoystickButtonConstants.INTAKE_LEFT_RETRACT_OVERRIDE));

        //Elevator  TODO: change CoDriver buttons back to CoDriver 
        this.elevatorContainerMacroButton.updateState(this.joystickDriver
            .getRawButton(JoystickButtonConstants.ELEVATOR_CONTAINER_MACRO_BUTTON));
        this.elevatorSetStateToFloorButton.updateState(this.joystickDriver
            .getRawButton(JoystickButtonConstants.ELEVATOR_SET_STAE_TO_FLOOR_BUTTON));
        this.elevatorSetStateToPlatformButton.updateState(this.joystickDriver
            .getRawButton(JoystickButtonConstants.ELEVATOR_SET_STATE_TO_PLATFORM_BUTTON));
        this.elevatorSetStateToStepButton.updateState(this.joystickDriver
            .getRawButton(JoystickButtonConstants.ELEVATOR_SET_STATE_TO_STEP_BUTTON));
        this.elevatorMoveTo0Totes.updateState(this.joystickDriver.getRawButton(JoystickButtonConstants.ELEVATOR_MOVE_TO_0_TOTES_BUTTON));
        this.elevatorMoveTo1Tote.updateState(this.joystickDriver.getRawButton(JoystickButtonConstants.ELEVATOR_MOVE_TO_1_TOTE_BUTTON));
        this.elevatorMoveTo2Totes.updateState(this.joystickDriver.getRawButton(JoystickButtonConstants.ELEVATOR_MOVE_TO_2_TOTES_BUTTON));
        this.elevatorMoveTo3Totes.updateState(this.joystickDriver.getRawButton(JoystickButtonConstants.ELEVATOR_MOVE_TO_3_TOTES_BUTTON));
        this.elevatorPIDOn.updateState(this.joystickCoDriver.getRawButton(JoystickButtonConstants.ELEVATOR_PID_ON));//TODO: change to CoDriver
        this.elevatorPIDOff.updateState(this.joystickCoDriver.getRawButton(JoystickButtonConstants.ELEVATOR_PID_OFF));//TODO: change to CoDriver
        this.elevatorStop.updateState(this.joystickCoDriver.getRawButton(JoystickButtonConstants.ELEVATOR_STOP_BUTTON));
        this.elevatorIgnoreSensors.updateState(this.joystickCoDriver.getRawButton(JoystickButtonConstants.ELEVATOR_IGNORE_SENSORS_BUTTON));
        this.elevatorUseSensors.updateState(this.joystickCoDriver.getRawButton(JoystickButtonConstants.ELEVATOR_USE_SENSORS_BUTTON));
        this.elevatorZeroEncoders.updateState(this.joystickCoDriver.getRawButton(JoystickButtonConstants.ELEVATOR_ZERO_ENCODERS));
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

        SmartDashboardLogger.putNumber(UserDriver.DRIVETRAIN_X_VELOCITY_LOG_KEY, xVelocity);

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

    //=================================================== Elevator ===============================================================

    @Override
    public boolean getElevatorContainerMacroButton()
    {
        boolean state = this.elevatorContainerMacroButton.isActivated();
        //        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_CONTAINER_MACRO_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorSetStateToFloorButton()
    {
        boolean state = this.elevatorSetStateToFloorButton.isActivated();
        //        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_SET_STATE_TO_FLOOR_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorSetStateToPlatformButton()
    {
        boolean state = this.elevatorSetStateToPlatformButton.isActivated();
        //        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_SET_STATE_TO_PLATFORM_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorSetStateToStepButton()
    {
        boolean state = this.elevatorSetStateToStepButton.isActivated();
        //        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_SET_STATE_TO_STEP_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorMoveTo0TotesButton()
    {
        boolean state = this.elevatorMoveTo0Totes.isActivated();
        //        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_MOVE_TO_0_TOTES_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorMoveTo1ToteButton()
    {
        boolean state = this.elevatorMoveTo1Tote.isActivated();
        //        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_MOVE_TO_1_TOTE_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorMoveTo2TotesButton()
    {
        boolean state = this.elevatorMoveTo2Totes.isActivated();
        //        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_MOVE_TO_2_TOTES_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorMoveTo3TotesButton()
    {
        boolean state = this.elevatorMoveTo3Totes.isActivated();
        //        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_MOVE_TO_3_TOTES_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorPIDOn()
    {
        boolean state = this.elevatorPIDOn.isActivated();
        //        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_PID_ON_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorPIDOff()
    {
        boolean state = this.elevatorPIDOff.isActivated();
        //        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_PID_OFF_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getStopElevatorButton()
    {
        boolean mode = this.elevatorStop.isActivated();
        //        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_STOP_LOG_KEY, mode);
        return mode;
    }

    @Override
    public boolean getElevatorUpButton()
    {
        boolean mode = this.joystickDriver.getRawButton(JoystickButtonConstants.ELEVATOR_UP_BUTTON);
        //        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_UP_STATE_LOG_KEY, mode);
        return mode;
    }

    @Override
    public boolean getElevatorDownButton()
    {
        boolean mode = this.joystickDriver.getRawButton(JoystickButtonConstants.ELEVATOR_DOWN_BUTTON);
        //        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_DOWN_STATE_LOG_KEY, mode);
        return mode;
    }

    @Override
    public boolean getElevatorMoveToBottom()
    {
        boolean mode = this.elevatorMoveToBottom.isActivated();
        //        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_MOVE_TO_BOTTOM_LOG_KEY, mode);
        return mode;
    }

    @Override
    public double getElevatorVelocityOverride()
    {
        double value = 0;//this.joystickCoDriver.getY();
        //        SmartDashboardLogger.putNumber(UserDriver.ELEVATOR_VELOCITY_OVERRIDE_LOG_KEY, value);
        return value;
    }

    @Override
    public boolean getIgnoreElevatorSensors()
    {
        boolean mode = this.elevatorIgnoreSensors.isActivated();
        //        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_IGNORE_SENSORS_LOG_KEY, mode);
        return mode;
    }

    @Override
    public boolean getUseElevatorSensors()
    {
        boolean mode = this.elevatorUseSensors.isActivated();
        //        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_USE_SENSORS_LOG_KEY, mode);
        return mode;
    }

    @Override
    public boolean getZeroElevatorEncoder()
    {
        boolean mode = this.elevatorZeroEncoders.isActivated();
        //        SmartDashboardLogger.putBoolean(UserDriver.ELEVATOR_ZERO_ENCODERS_LOG_KEY, mode);
        return mode;
    }

    //===================================================== Arm =================================================================

    @Override
    public boolean getArmMacroExtendButton()
    {
        boolean mode = this.armMacroExtendButton.isActivated();
        //        SmartDashboardLogger.putBoolean(UserDriver.ARM_MACRO_EXTEND_STATE_LOG_KEY, mode);
        return mode;
    }

    @Override
    public boolean getArmMacroRetractButton()
    {
        boolean mode = this.armMacroRetractButton.isActivated();
        //        SmartDashboardLogger.putBoolean(UserDriver.ARM_MACRO_RETRACT_STATE_LOG_KEY, mode);
        return mode;
    }

    @Override
    public boolean getArmExtenderExtendOverride()
    {
        boolean mode = this.armExtenderExtendOverride.isActivated();
        //        SmartDashboardLogger.putBoolean(UserDriver.ARM_EXTENDER_EXTEND_OVERRIDE_LOG_KEY, mode);
        return mode;
    }

    @Override
    public boolean getArmExtenderRetractOverride()
    {
        boolean mode = this.armExtenderRetractOverride.isActivated();
        //        SmartDashboardLogger.putBoolean(UserDriver.ARM_EXTENDER_RETRACT_OVERRIDE_LOG_KEY, mode);
        return mode;
    }

    @Override
    public boolean getArmTiltExtendOverride()
    {
        boolean mode = this.armTiltExtendOverride.isActivated();
        //        SmartDashboardLogger.putBoolean(UserDriver.ARM_TILT_EXTEND_OVERRIDE_LOG_KEY, mode);
        return mode;
    }

    @Override
    public boolean getArmTiltRetractOverride()
    {
        boolean mode = this.armTiltRetractOverride.isActivated();
        //        SmartDashboardLogger.putBoolean(UserDriver.ARM_TILT_RETRACT_OVERRIDE_LOG_KEY, mode);
        return mode;
    }

    @Override
    public boolean getArmTromboneExtendOverride()
    {
        boolean mode = this.armTromboneExtendOverride.isActivated();
        //        SmartDashboardLogger.putBoolean(UserDriver.ARM_TROMBONE_EXTEND_OVERRIDE_LOG_KEY, mode);
        return mode;
    }

    @Override
    public boolean getArmTromboneRetractOverride()
    {
        boolean mode = this.armTromboneRetractOverride.isActivated();
        //        SmartDashboardLogger.putBoolean(UserDriver.ARM_TROMBONE_RETRACT_OVERRIDE_LOG_KEY, mode);
        return mode;
    }

    //=================================================== Intake ================================================================

    @Override
    public boolean getIntakeUpButton()
    {
        boolean mode = this.intakeUpButton.isActivated();
        //        SmartDashboardLogger.putBoolean(UserDriver.INTAKE_UP_STATE_KEY, mode);
        return mode;
    }

    @Override
    public boolean getIntakeDownButton()
    {
        boolean mode = this.intakeDownButton.isActivated();
        //        SmartDashboardLogger.putBoolean(UserDriver.INTAKE_DOWN_STATE_KEY, mode);
        return mode;
    }

    @Override
    public boolean getIntakeRightExtendOverride()
    {
        boolean mode = this.intakeRightExtendOverride.isActivated();
        //        SmartDashboardLogger.putBoolean(UserDriver.INTAKE_RIGHT_EXTEND_OVERRIDE_STATE_KEY, mode);
        return mode;
    }

    @Override
    public boolean getIntakeRightRetractOverride()
    {
        boolean mode = this.intakeRightRetractOverride.isActivated();
        //        SmartDashboardLogger.putBoolean(UserDriver.INTAKE_RIGHT_RETRACT_OVERRIDE_STATE_KEY, mode);
        return mode;
    }

    @Override
    public boolean getIntakeLeftExtendOverride()
    {
        boolean mode = this.intakeLeftExtendOverride.isActivated();
        //        SmartDashboardLogger.putBoolean(UserDriver.INTAKE_LEFT_EXTEND_OVERRIDE_STATE_KEY, mode);
        return mode;
    }

    @Override
    public boolean getIntakeLeftRetractOverride()
    {
        boolean mode = this.intakeLeftRetractOverride.isActivated();
        //        SmartDashboardLogger.putBoolean(UserDriver.INTAKE_LEFT_RETRACT_OVERRIDE_STATE_KEY, mode);
        return mode;
    }

    @Override
    public boolean getIntakeForwardButton()
    {
        boolean mode = this.joystickDriver.getRawButton(JoystickButtonConstants.INTAKE_FORWARD_BUTTON);
        //        SmartDashboardLogger.putBoolean(UserDriver.INTAKE_FORWARD_STATE_KEY, mode);
        return mode;
    }

    @Override
    public boolean getIntakeBackwardButton()
    {
        boolean mode = this.joystickDriver.getRawButton(JoystickButtonConstants.INTAKE_BACKWARD_BUTTON);
        //        SmartDashboardLogger.putBoolean(UserDriver.INTAKE_BACKWARD_STATE_KEY, mode);
        return mode;
    }
}
