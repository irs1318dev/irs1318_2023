package org.usfirst.frc.team1318.robot.Autonomous;

import java.util.Queue;

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
    private static final String ELEVATOR_MACRO_STATE_LOG_KEY = "a.elevatorMacroState";
    private static final String ELEVATOR_HEIGHT_0_STATE_LOG_KEY = "a.elevatorHeight0";
    private static final String ELEVATOR_HEIGHT_1_STATE_LOG_KEY = "a.elevatorHeight1";
    private static final String ELEVATOR_HEIGHT_2_STATE_LOG_KEY = "a.elevatorHeight2";
    private static final String ELEVATOR_HEIGHT_3_STATE_LOG_KEY = "a.elevatorHeight3";
    private static final String ELEVATOR_HEIGHT_4_STATE_LOG_KEY = "a.elevatorHeight4";
    private static final String ELEVATOR_HEIGHT_5_STATE_LOG_KEY = "a.elevatorHeight5";
    private static final String ELEVATOR_HEIGHT_6_STATE_LOG_KEY = "a.elevatorHeight6";
    private static final String ELEVATOR_HEIGHT_7_STATE_LOG_KEY = "a.elevatorHeight7";
    private static final String ELEVATOR_OVERRIDE_STATE_LOG_KEY = "a.elevatorOverride";

    //Arm 
    private static final String ARM_MACRO_STATE_LOG_KEY = "a.armMacroState";
    private static final String ARM_EXTENDER_STATE_LOG_KEY = "a.armExtenderOverride";
    private static final String ARM_TILT_STATE_LOG_KEY = "a.armTiltOverride";
    private static final String ARM_TROMBONE_STATE_LOG_KEY = "a.armTromboneOverride";

    private final Queue<IAutonomousTask> autonomousTasks;
    private IAutonomousTask currentTask;
    private final AutonomousControlData controlData;

    /**
     * Initializes a new AutonomousDriver
     * @param autonomousTasks to execute as a part of this driver
     */
    public AutonomousDriver(Queue<IAutonomousTask> autonomousTasks)
    {
        this.autonomousTasks = autonomousTasks;
        this.currentTask = null;
        this.controlData = new AutonomousControlData();
    }

    /**
     * Tell the driver that some time has passed
     */
    public void update()
    {
        // check whether we should continue with the current task
        if (this.currentTask != null)
        {
            if (!this.currentTask.shouldContinue())
            {
                this.currentTask.end(this.controlData);
                this.currentTask = null;
            }
        }

        // if there's no current task, find the next one and start it (if any)
        if (this.currentTask == null)
        {
            this.currentTask = this.autonomousTasks.poll();

            // if there's no next task to run, then we are done
            if (this.currentTask == null)
            {
                return;
            }

            this.currentTask.begin();
        }

        // run the current task and apply the result to the control data
        this.currentTask.update(this.controlData);
    }

    /**
     * Tell the operator component that operation is stopping
     */
    public void stop()
    {
        if (this.currentTask != null)
        {
            this.currentTask.cancel(this.controlData);
            this.currentTask = null;
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

        SmartDashboardLogger.putNumber(AutonomousDriver.DRIVETRAIN_X_VELOCITY_LOG_KEY, xVelocity);

        return xVelocity;
    }

    /**
     * Get a value indicating the desired drive train Y velocity (turn amount) 
     * @return value between -1.0 and 1.0 (percentage of max forward velocity)
     */
    public double getDriveTrainYVelocity()
    {
        double yVelocity = this.controlData.getDriveTrainYVelocity();

        SmartDashboardLogger.putNumber(AutonomousDriver.DRIVETRAIN_Y_VELOCITY_LOG_KEY, yVelocity);

        return yVelocity;
    }

    /**
     * Get a value indicating whether we should be using the drive train in simple mode 
     * @return true if we should be in simple mode, otherwise false
     */
    public boolean getDriveTrainSimpleMode()
    {
        boolean simpleMode = this.controlData.getDriveTrainSimpleMode();

        SmartDashboardLogger.putBoolean(AutonomousDriver.DRIVETRAIN_SIMPLE_MODE_LOG_KEY, simpleMode);

        return simpleMode;
    }

    /**
     * Get a value indicating the desired drive train left position for positional mode
     * @return position
     */
    public double getDriveTrainLeftPosition()
    {
        double leftPosition = this.controlData.getDriveTrainLeftPosition();

        SmartDashboardLogger.putNumber(AutonomousDriver.DRIVETRAIN_LEFT_POSITION_LOG_KEY, leftPosition);

        return leftPosition;
    }

    /**
     * Get a value indicating the desired drive train right position for positional mode
     * @return position
     */
    public double getDriveTrainRightPosition()
    {
        double rightPosition = this.controlData.getDriveTrainRightPosition();

        SmartDashboardLogger.putNumber(AutonomousDriver.DRIVETRAIN_RIGHT_POSITION_LOG_KEY, rightPosition);

        return rightPosition;
    }

    /**
     * Get a value indicating whether the drive train should be in position or velocity mode
     * @return true if position mode, false if velocity mode
     */
    public boolean getDriveTrainPositionMode()
    {
        boolean positionMode = this.controlData.getDriveTrainPositionMode();

        SmartDashboardLogger.putBoolean(AutonomousDriver.DRIVETRAIN_POSITION_MODE_LOG_KEY, positionMode);

        return positionMode;
    }

    //=================================================== Elevator ===============================================================

    @Override
    public boolean getElevatorMacroButton()
    {
        boolean mode = this.controlData.getElevatorMacroState();
        SmartDashboardLogger.putBoolean(AutonomousDriver.ELEVATOR_MACRO_STATE_LOG_KEY, mode);
        return mode;
    }

    @Override
    public boolean getElevatorHeight0Button()
    {
        boolean state = this.controlData.getElevatorHeight0State();
        SmartDashboardLogger.putBoolean(AutonomousDriver.ELEVATOR_HEIGHT_0_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorHeight1Button()
    {
        boolean state = this.controlData.getElevatorHeight1State();
        SmartDashboardLogger.putBoolean(AutonomousDriver.ELEVATOR_HEIGHT_1_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorHeight2Button()
    {
        boolean state = this.controlData.getElevatorHeight2State();
        SmartDashboardLogger.putBoolean(AutonomousDriver.ELEVATOR_HEIGHT_2_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorHeight3Button()
    {
        boolean state = this.controlData.getElevatorHeight3State();
        SmartDashboardLogger.putBoolean(AutonomousDriver.ELEVATOR_HEIGHT_3_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorHeight4Button()
    {
        boolean state = this.controlData.getElevatorHeight4State();
        SmartDashboardLogger.putBoolean(AutonomousDriver.ELEVATOR_HEIGHT_4_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorHeight5Button()
    {
        boolean state = this.controlData.getElevatorHeight5State();
        SmartDashboardLogger.putBoolean(AutonomousDriver.ELEVATOR_HEIGHT_5_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorHeight6Button()
    {
        boolean state = this.controlData.getElevatorHeight6State();
        SmartDashboardLogger.putBoolean(AutonomousDriver.ELEVATOR_HEIGHT_6_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getElevatorHeight7Button()
    {
        boolean state = this.controlData.getElevatorHeight7State();
        SmartDashboardLogger.putBoolean(AutonomousDriver.ELEVATOR_HEIGHT_7_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public double getElevatorOverride()
    {
        double speed = this.controlData.getElevatorOverrideState();
        SmartDashboardLogger.putNumber(AutonomousDriver.ELEVATOR_OVERRIDE_STATE_LOG_KEY, speed);
        return speed;
    }

    //===================================================== Arm =================================================================

    @Override
    public boolean getArmMacroToggle()
    {
        boolean state = this.controlData.getArmMacroState();
        SmartDashboardLogger.putBoolean(AutonomousDriver.ARM_MACRO_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getArmExtenderToggleOverride()
    {
        boolean state = this.controlData.getArmExtenderOverrideState();
        SmartDashboardLogger.putBoolean(AutonomousDriver.ARM_EXTENDER_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getArmTiltToggleOverride()
    {
        boolean state = this.controlData.getArmTiltOverrideState();
        SmartDashboardLogger.putBoolean(AutonomousDriver.ARM_TILT_STATE_LOG_KEY, state);
        return state;
    }

    @Override
    public boolean getArmTromboneToggleOverride()
    {
        boolean state = this.controlData.getArmTromboneOverrideState();
        SmartDashboardLogger.putBoolean(AutonomousDriver.ARM_TROMBONE_STATE_LOG_KEY, state);
        return state;
    }

    //=================================================== Intake ================================================================

    @Override
    public boolean getIntakeUpButton()
    {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getIntakeDownButton()
    {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getIntakeRightToggleOverride()
    {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getIntakeLeftToggleOverride()
    {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getIntakeForwardButton()
    {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getIntakeBackwardButton()
    {
        // TODO Auto-generated method stub
        return false;
    }

}
