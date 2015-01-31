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
     * Get a value indicating whether the robot should start the macro to flip a container  
     * @return true if start macro, otherwise false  
     */
    public boolean getElevatorContainerMacroButton();

    /**
     * Get a value indicating whether the elevator state should be set to the floor
     * @return true if set to floor, otherwise false
     */
    public boolean getElevatorSetStateToFloorButton();

    /**
     * Get a value indicating whether the elevator state should be set to the platform
     * @return true if set to platform, otherwise false
     */
    public boolean getElevatorSetStateToPlatformButton();

    /**
     * Get a value indicating whether the elevator state should be set to the step
     * @return true if set to step, otherwise false
     */
    public boolean getElevatorSetStateToStepButton();

    /**
     * Get a value indicating whether the elevator should move to the height of 0 totes over the current setting
     * @return true if move, otherwise false
     */
    public boolean getElevatorMoveTo0TotesButton();

    /**
     * Get a value indicating whether the elevator should move to the height of 1 tote over the current setting
     * @return true if move, otherwise false
     */
    public boolean getElevatorMoveTo1ToteButton();

    /**
     * Get a value indicating whether the elevator should move to the height of 2 totes over the current setting
     * @return true if move, otherwise false
     */
    public boolean getElevatorMoveTo2TotesButton();

    /**
     * Get a value indicating whether the elevator should move to the height of 3 totes over the current setting
     * @return true if move, otherwise false
     */
    public boolean getElevatorMoveTo3TotesButton();

    /**
     * Get a value indicating whether to toggle the elevator PID
     * @return true if toggle, otherwise false
     */
    public boolean getElevatorPIDToggle();

    /**
     * Get a value indicating whether the elevator should stop 
     * @return true for stop elevator, otherwise false
     */
    public boolean getStopElevatorButton();

    /**
     * Get a value indicating whether to move the elevator up 
     * @return true for move elevator up, otherwise false 
     */
    public boolean getElevatorUpButton();

    /**
     * Get a value indicating whether to move the elevator down 
     * @return true for move elevator down, otherwise false 
     */
    public boolean getElevatorDownButton();

    //===================================================== Arm =================================================================
    /**
     * Get a value indicating whether to run the macro to extend the whole arm state 
     * @return true for run macro, otherwise false 
     */
    public boolean getArmMacroExtendButton();

    /**
     * Get a value indicating whether to run the macro to retract teh whole arm state 
     * @return true for run macro, otherwise false 
     */
    public boolean getArmMacroRetractButton();

    /**
     * Get a value indicating whether to toggle the arm extender state 
     * @return true for toggle state, otherwise false 
     */
    public boolean getArmExtenderToggleOverride();

    /**
     * Get a value indicating whether to toggle the arm tilt state 
     * @return true for toggle state, otherwise false 
     */
    public boolean getArmTiltToggleOverride();

    /**
     * Get a value indicating whether to toggle the arm trombone state 
     * @return true for toggle state, otherwise false 
     */
    public boolean getArmTromboneToggleOverride();

    //=================================================== Intake ================================================================
    /**
     * Get a value indicating whether to move both sides of the intake up 
     * @return true for move up, otherwise false 
     */
    public boolean getIntakeUpButton();

    /**
     * Get a value indicating whether to move both sides of the intake down 
     * @return true for move down, otherwise false 
     */
    public boolean getIntakeDownButton();

    /**
     * Get a value indicating whether to switch the state of the right side of the intake 
     * @return true for toggle, otherwise false 
     */
    public boolean getIntakeRightToggleOverride();

    /**
     * Get a value indicating whether to switch the state of the left side of the intake 
     * @return true for toggle, otherwise false 
     */
    public boolean getIntakeLeftToggleOverride();

    /**
     * Get a value indicating whether to move the intake wheels forward 
     * @return true for move forward, otherwise false 
     */
    public boolean getIntakeForwardButton();

    /**
     * Get a value indicating whether to move the intake wheels forward 
     * @return true for move forward, otherwise false 
     */
    public boolean getIntakeBackwardButton();

}
