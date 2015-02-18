package org.usfirst.frc.team1318.robot.Autonomous;

import org.usfirst.frc.team1318.robot.Autonomous.Tasks.SequentialTask;
import org.usfirst.frc.team1318.robot.Common.IDriver;
import org.usfirst.frc.team1318.robot.Common.SmartDashboardLogger;

/**
 * Driver for autonomous mode.  Autonomous driver acts as the operator of the robot,
 * telling it what actions to perform as determined by the current task and tasks that have
 * come before it that intentionally don't reset their state.
 * 
 * @author Will
 *
 */
public class AutonomousDriver implements IDriver
{
    // logging constants
    //Drive Train 
    private static final String DRIVETRAIN_X_VELOCITY_LOG_KEY = "a.driveXVelocity";
    private static final String DRIVETRAIN_Y_VELOCITY_LOG_KEY = "a.driveYVelocity";
    private static final String DRIVETRAIN_SIMPLE_MODE_LOG_KEY = "a.driveSimpleMode";
    private static final String DRIVETRAIN_LEFT_POSITION_LOG_KEY = "a.driveLeftPosition";
    private static final String DRIVETRAIN_RIGHT_POSITION_LOG_KEY = "a.driveRightPosition";
    private static final String DRIVETRAIN_POSITION_MODE_LOG_KEY = "a.drivePositionMode";

    //Elevator 
    private static final String ELEVATOR_CONTAINER_MACRO_STATE_LOG_KEY = "a.elevatorContainerMacroState";
    private static final String ELEVATOR_SET_STATE_TO_FLOOR_LOG_KEY = "a.elevatorSetStateToFloor";
    private static final String ELEVATOR_SET_STATE_TO_PLATFORM_LOG_KEY = "a.elevatorSetStateToPlatform";
    private static final String ELEVATOR_SET_STATE_TO_STEP_LOG_KEY = "a.elevatorSetStateToStep";
    private static final String ELEVATOR_MOVE_TO_0_TOTES_LOG_KEY = "a.elevatorHeight3";
    private static final String ELEVATOR_MOVE_TO_1_TOTE_LOG_KEY = "a.elevatorHeight4";
    private static final String ELEVATOR_MOVE_TO_2_TOTES_LOG_KEY = "a.elevatorHeight5";
    private static final String ELEVATOR_MOVE_TO_3_TOTES_LOG_KEY = "a.elevatorHeight6";
    private static final String ELEVATOR_PICK_UP_MACRO_LOG_KEY = "a.elevatorPickUpMacro";
    private static final String ELEVATOR_PID_ON_STATE_LOG_KEY = "a.elevatorPIDOnState";
    private static final String ELEVATOR_PID_OFF_STATE_LOG_KEY = "a.elevatorPIDOffState";
    private static final String ELEVATOR_STOP_STATE_LOG_KEY = "a.elevatorStop";
    private static final String ELEVATOR_UP_STATE_LOG_KEY = "a.elevatorUp";
    private static final String ELEVATOR_DOWN_STATE_LOG_KEY = "a.elevatorDown";
    private static final String ELEVATOR_MOVE_TO_BOTTOM_STATE_LOG_KEY = "a.elevatorMoveToBottom";
    private static final String ELEVATOR_VELOCITY_OVERRIDE_LOG_KEY = "a.elevatorVelocityOverride";
    private static final String ELEVATOR_IGNORE_SENSORS_LOG_KEY = "a.elevatorIgnoreSensors";
    private static final String ELEVATOR_USE_SENSORS_LOG_KEY = "a.elevatorUseSensors";
    private static final String ELEVATOR_ZERO_ENCODERS_LOG_KEY = "a.elevatorZeroEncoders";

    //Arm 
    private static final String ARM_MACRO_EXTEND_STATE_LOG_KEY = "a.armMacroExtendState";
    private static final String ARM_MACRO_RETRACT_STATE_LOG_KEY = "a.armMacroRetractState";
    private static final String ARM_TILT_EXTEND_OVERRIDE_LOG_KEY = "a.armTiltExtendOverride";
    private static final String ARM_TILT_RETRACT_OVERRIDE_LOG_KEY = "a.armTiltRetractOverride";
    private static final String ARM_EXTENDER_EXTEND_OVERRIDE_LOG_KEY = "a.armExtenderExtendOverride";
    private static final String ARM_EXTENDER_RETRACT_OVERRIDE_LOG_KEY = "a.armExtenderRetractOverride";
    private static final String ARM_TROMBONE_EXTEND_OVERRIDE_LOG_KEY = "a.armTromboneExtendOverride";
    private static final String ARM_TROMBONE_RETRACT_OVERRIDE_LOG_KEY = "a.armTromboneRetractOverride";

    //Intake 
    private static final String INTAKE_UP_STATE_KEY = "a.intakeUpState";
    private static final String INTAKE_DOWN_STATE_KEY = "a.intakeDownState";
    private static final String INTAKE_RIGHT_EXTEND_OVERRIDE_STATE_KEY = "a.intakeRightExtendOverrideState";
    private static final String INTAKE_RIGHT_RETRACT_OVERRIDE_STATE_KEY = "a.intakeRightRetractOverrideState";
    private static final String INTAKE_LEFT_EXTEND_OVERRIDE_STATE_KEY = "a.intakeLeftExtendOverrideState";
    private static final String INTAKE_LEFT_RETRACT_OVERRIDE_STATE_KEY = "a.intakeLeftRetractOverrideState";
    private static final String INTAKE_FORWARD_STATE_KEY = "a.intakeForwardStateKey";
    private static final String INTAKE_BACKWARD_STATE_KEY = "a.intakeBackwardStateKey";

    private final IAutonomousTask autonomousTask;
    private final AutonomousControlData controlData;

    private boolean hasBegun;
    private boolean hasEnded;

    /**
     * Initializes a new AutonomousDriver
     * @param autonomousTask to execute as a part of this driver
     */
    public AutonomousDriver(IAutonomousTask autonomousTask)
    {
        this.autonomousTask = autonomousTask;
        this.controlData = new AutonomousControlData();

        this.hasBegun = false;
        this.hasEnded = false;
    }

    /**
     * Initializes a new AutonomousDriver
     * @param autonomousTasks to execute as a part of this driver
     */
    public AutonomousDriver(IAutonomousTask[] autonomousTasks)
    {
        IAutonomousTask singleTask = null;
        if (autonomousTasks != null)
        {
            if (autonomousTasks.length > 1)
            {
                singleTask = new SequentialTask(autonomousTasks);
            }
            else
            {
                singleTask = autonomousTasks[0];
            }
        }

        this.autonomousTask = singleTask;
        this.controlData = new AutonomousControlData();

        this.hasBegun = false;
        this.hasEnded = false;
    }

    /**
     * Tell the driver that some time has passed
     */
    public void update()
    {
        if (!this.hasEnded)
        {
            if (!this.hasBegun)
            {
                // if we haven't begun, begin
                this.autonomousTask.begin();
                this.hasBegun = true;
            }

            if (this.autonomousTask.hasCompleted())
            {
                // if we shouldn't continue, end the task
                this.autonomousTask.end(this.controlData);
                this.hasEnded = true;
            }
            else
            {
                // run the current task and apply the result to the control data
                this.autonomousTask.update(this.controlData);
            }
        }
    }

    /**
     * Tell the operator component that operation is stopping
     */
    public void stop()
    {
        if (this.autonomousTask != null)
        {
            this.autonomousTask.cancel(this.controlData);
            this.hasEnded = true;
        }
    }

    //================================================== DriveTrain ==============================================================

    /**
     * Get a value indicating the desired drive train X Velocity 
     * @return value between -1.0 and 1.0 (percentage of max right turn velocity)
     */
    public double getDriveTrainXVelocity()
    {
        double xVelocity = this.controlData.getDriveTrainXVelocity();

        //        SmartDashboardLogger.putNumber(AutonomousDriver.DRIVETRAIN_X_VELOCITY_LOG_KEY, xVelocity);

        return xVelocity;
    }

    /**
     * Get a value indicating the desired drive train Y velocity (turn amount) 
     * @return value between -1.0 and 1.0 (percentage of max forward velocity)
     */
    public double getDriveTrainYVelocity()
    {
        double yVelocity = this.controlData.getDriveTrainYVelocity();

        //        SmartDashboardLogger.putNumber(AutonomousDriver.DRIVETRAIN_Y_VELOCITY_LOG_KEY, yVelocity);

        return yVelocity;
    }

    /**
     * Get a value indicating whether we should be using the drive train in simple mode 
     * @return true if we should be in simple mode, otherwise false
     */
    public boolean getDriveTrainSimpleMode()
    {
        boolean simpleMode = this.controlData.getDriveTrainSimpleMode();

        //        SmartDashboardLogger.putBoolean(AutonomousDriver.DRIVETRAIN_SIMPLE_MODE_LOG_KEY, simpleMode);

        return simpleMode;
    }

    /**
     * Get a value indicating the desired drive train left position for positional mode
     * @return position
     */
    public double getDriveTrainLeftPosition()
    {
        double leftPosition = this.controlData.getDriveTrainLeftPosition();

        //        SmartDashboardLogger.putNumber(AutonomousDriver.DRIVETRAIN_LEFT_POSITION_LOG_KEY, leftPosition);

        return leftPosition;
    }

    /**
     * Get a value indicating the desired drive train right position for positional mode
     * @return position
     */
    public double getDriveTrainRightPosition()
    {
        double rightPosition = this.controlData.getDriveTrainRightPosition();

        //        SmartDashboardLogger.putNumber(AutonomousDriver.DRIVETRAIN_RIGHT_POSITION_LOG_KEY, rightPosition);

        return rightPosition;
    }

    /**
     * Get a value indicating whether the drive train should be in position or velocity mode
     * @return true if position mode, false if velocity mode
     */
    public boolean getDriveTrainPositionMode()
    {
        boolean positionMode = this.controlData.getDriveTrainPositionMode();

        //        SmartDashboardLogger.putBoolean(AutonomousDriver.DRIVETRAIN_POSITION_MODE_LOG_KEY, positionMode);

        return positionMode;
    }

    //=================================================== Elevator ===============================================================

    @Override
    public boolean getElevatorContainerMacroButton()
    {
        boolean mode = this.controlData.getElevatorContainerMacroState();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.ELEVATOR_CONTAINER_MACRO_STATE_LOG_KEY, mode);
        return mode;
    }

    @Override
    public boolean getElevatorSetStateToFloorButton()
    {
        boolean state = this.controlData.getElevatorSetStateToFloor();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.ELEVATOR_SET_STATE_TO_FLOOR_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorSetStateToPlatformButton()
    {
        boolean state = this.controlData.getElevatorSetStateToPlatform();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.ELEVATOR_SET_STATE_TO_PLATFORM_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorSetStateToStepButton()
    {
        boolean state = this.controlData.getElevatorSetStateToStep();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.ELEVATOR_SET_STATE_TO_STEP_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorMoveTo0TotesButton()
    {
        boolean state = this.controlData.getElevatorMoveTo0Totes();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.ELEVATOR_MOVE_TO_0_TOTES_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorMoveTo1ToteButton()
    {
        boolean state = this.controlData.getElevatorMoveTo1Tote();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.ELEVATOR_MOVE_TO_1_TOTE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorMoveTo2TotesButton()
    {
        boolean state = this.controlData.getElevatorMoveTo2Totes();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.ELEVATOR_MOVE_TO_2_TOTES_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorMoveTo3TotesButton()
    {
        boolean state = this.controlData.getElevatorMoveTo3Totes();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.ELEVATOR_MOVE_TO_3_TOTES_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorPickUpMacro()
    {
        boolean state = this.controlData.getElevatorTotePickUpMacroState();
        SmartDashboardLogger.putBoolean(AutonomousDriver.ELEVATOR_PICK_UP_MACRO_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorPIDOn()
    {
        boolean state = this.controlData.getElevatorPIDOnState();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.ELEVATOR_PID_ON_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorPIDOff()
    {
        boolean state = this.controlData.getElevatorPIDOffState();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.ELEVATOR_PID_OFF_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getStopElevatorButton()
    {
        boolean mode = this.controlData.getElevatorStopState();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.ELEVATOR_STOP_STATE_LOG_KEY, mode);
        return mode;
    }

    @Override
    public boolean getElevatorUpButton()
    {
        boolean mode = this.controlData.getElevatorUpState();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.ELEVATOR_UP_STATE_LOG_KEY, mode);
        return mode;
    }

    @Override
    public boolean getElevatorDownButton()
    {
        boolean mode = this.controlData.getElevatorDownState();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.ELEVATOR_DOWN_STATE_LOG_KEY, mode);
        return mode;
    }

    @Override
    public boolean getElevatorMoveToBottom()
    {
        boolean mode = this.controlData.getElevatorMoveToBottomState();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.ELEVATOR_MOVE_TO_BOTTOM_STATE_LOG_KEY, mode);
        return mode;
    }

    public double getElevatorVelocityOverride()
    {
        double value = this.controlData.getElevatorVelocityOverrideState();
        //        SmartDashboardLogger.putNumber(AutonomousDriver.ELEVATOR_VELOCITY_OVERRIDE_LOG_KEY, value);
        return value;
    }

    @Override
    public boolean getIgnoreElevatorSensors()
    {
        boolean mode = this.controlData.getElevatorIgnoreSensorsState();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.ELEVATOR_IGNORE_SENSORS_LOG_KEY, mode);
        return mode;
    }

    @Override
    public boolean getUseElevatorSensors()
    {
        boolean mode = this.controlData.getElevatorUseSensorsState();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.ELEVATOR_USE_SENSORS_LOG_KEY, mode);
        return mode;
    }

    @Override
    public boolean getZeroElevatorEncoder()
    {
        boolean mode = this.controlData.getElevatorZeroEncoderState();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.ELEVATOR_ZERO_ENCODERS_LOG_KEY, mode);
        return mode;
    }

    @Override
    public boolean getSlowElevatorButton()
    {
        //TODO: add 
        return false;
    }

    @Override
    public boolean getFastElevatorButton()
    {
        //TODO: add 
        return false;
    }

    //===================================================== Arm =================================================================

    @Override
    public boolean getArmMacroExtendButton()
    {
        boolean state = this.controlData.getArmMacroExtendState();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.ARM_MACRO_EXTEND_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getArmMacroRetractButton()
    {
        boolean state = this.controlData.getArmMacroRetractState();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.ARM_MACRO_RETRACT_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getArmExtenderExtendOverride()
    {
        boolean state = this.controlData.getArmExtenderExtendOverrideState();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.ARM_EXTENDER_EXTEND_OVERRIDE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getArmExtenderRetractOverride()
    {
        boolean state = this.controlData.getArmExtenderRetractOverrideState();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.ARM_EXTENDER_RETRACT_OVERRIDE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getArmTiltExtendOverride()
    {
        boolean state = this.controlData.getArmTiltExtendOverrideState();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.ARM_TILT_EXTEND_OVERRIDE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getArmTiltRetractOverride()
    {
        boolean state = this.controlData.getArmTiltRetractOverrideState();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.ARM_TILT_RETRACT_OVERRIDE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getArmTiltRetractHoldButton()
    {
        //we probably shouldn't use this for autonomous  
        return false;
    }

    @Override
    public boolean getArmTromboneExtendOverride()
    {
        boolean state = this.controlData.getArmTromboneExtendOverrideState();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.ARM_TROMBONE_EXTEND_OVERRIDE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getArmTromboneRetractOverride()
    {
        boolean state = this.controlData.getArmTromboneRetractOverrideState();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.ARM_TROMBONE_RETRACT_OVERRIDE_LOG_KEY, state);
        return state;
    }

    //=================================================== Intake ================================================================

    @Override
    public boolean getIntakeUpButton()
    {
        boolean state = this.controlData.getIntakeUpState();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.INTAKE_UP_STATE_KEY, state);
        return state;
    }

    @Override
    public boolean getIntakeDownButton()
    {
        boolean state = this.controlData.getIntakeDownState();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.INTAKE_DOWN_STATE_KEY, state);
        return state;
    }

    @Override
    public boolean getIntakeDownHoldButton()
    {
        //we probably shouldn't use this method in autonomous 
        return false;
    }

    @Override
    public boolean getIntakeRightExtendOverride()
    {
        boolean state = this.controlData.getIntakeRightExtendOverrideState();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.INTAKE_RIGHT_EXTEND_OVERRIDE_STATE_KEY, state);
        return state;
    }

    @Override
    public boolean getIntakeRightRetractOverride()
    {
        boolean state = this.controlData.getIntakeRightRetractOverrideState();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.INTAKE_RIGHT_RETRACT_OVERRIDE_STATE_KEY, state);
        return state;
    }

    @Override
    public boolean getIntakeLeftExtendOverride()
    {
        boolean state = this.controlData.getIntakeLeftExtendOverrideState();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.INTAKE_LEFT_EXTEND_OVERRIDE_STATE_KEY, state);
        return state;
    }

    @Override
    public boolean getIntakeLeftRetractOverride()
    {
        boolean state = this.controlData.getIntakeLeftRetractOverrideState();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.INTAKE_LEFT_RETRACT_OVERRIDE_STATE_KEY, state);
        return state;
    }

    @Override
    public boolean getIntakeForwardButton()
    {
        boolean state = this.controlData.getIntakeForwardState();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.INTAKE_FORWARD_STATE_KEY, state);
        return state;
    }

    @Override
    public boolean getIntakeBackwardButton()
    {
        boolean state = this.controlData.getIntakeBackwardState();
        //        SmartDashboardLogger.putBoolean(AutonomousDriver.INTAKE_BACKWARD_STATE_KEY, state);
        return state;
    }

}
