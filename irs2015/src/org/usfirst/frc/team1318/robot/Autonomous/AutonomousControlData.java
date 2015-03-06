package org.usfirst.frc.team1318.robot.Autonomous;

/**
 * Class simply holding data about the current control settings for autonomous mode.  
 * 
 * @author Will
 *
 */
public class AutonomousControlData
{
    //Drive Train 
    private double driveTrainXVelocity;
    private double driveTrainYVelocity;
    private double driveTrainLeftPosition;
    private double driveTrainRightPosition;
    private boolean driveTrainPositionMode;
    private boolean driveTrainSimpleMode;

    //Elevator 
    private boolean elevatorContainerMacroState;
    private boolean elevatorTotePickUpMacroState;
    private boolean elevatorSetStateToFloor;
    private boolean elevatorSetStateToPlatform;
    private boolean elevatorSetStateToStep;
    private boolean elevatorMoveTo0Totes;
    private boolean elevatorMoveTo1Tote;
    private boolean elevatorMoveTo2Totes;
    private boolean elevatorMoveTo3Totes;
    private boolean elevatorPIDOnState;
    private boolean elevatorPIDOffState;
    private boolean elevatorStopState;
    private boolean elevatorUpState;
    private boolean elevatorDownState;
    private boolean elevatorMoveToBottomState;
    private double elevatorVelocityOverrideState;
    private boolean elevatorIgnoreSensorsState;
    private boolean elevatorUseSensorsState;
    private boolean elevatorZeroEncodersState;
    private boolean elevatorSlowState;
    private boolean elevatorRegularSpeedState;
    private boolean elevatorFastState;

    //Arm
    private boolean armMacroExtendState;
    private boolean armMacroRetractState;
    private boolean armExtenderExtendOverrideState;
    private boolean armExtenderRetractOverrideState;
    private boolean armTiltExtendOverrideState;
    private boolean armTiltRetractOverrideState;
    private boolean armTromboneExtendOverrideState;
    private boolean armTromboneRetractOverrideState;

    //Intake 
    private boolean intakeUpState;
    private boolean intakeDownState;
    private boolean intakeRightExtendOverrideState;
    private boolean intakeRightRetractOverrideState;
    private boolean intakeLeftExtendOverrideState;
    private boolean intakeLeftRetractOverrideState;
    private boolean intakeForwardState;
    private boolean intakeBackwardState;
    private boolean intakeJitterState;

    /**
     * Initializes a new AutonomousControlData
     */
    public AutonomousControlData()
    {
        //Drive Train 
        this.driveTrainXVelocity = 0.0;
        this.driveTrainYVelocity = 0.0;
        this.driveTrainSimpleMode = false;
        this.driveTrainLeftPosition = 0.0;
        this.driveTrainRightPosition = 0.0;
        this.driveTrainPositionMode = false;

        //Elevator 
        this.elevatorContainerMacroState = false;
        this.elevatorSetStateToFloor = false;
        this.elevatorSetStateToPlatform = false;
        this.elevatorSetStateToStep = false;
        this.elevatorMoveTo0Totes = false;
        this.elevatorMoveTo1Tote = false;
        this.elevatorMoveTo2Totes = false;
        this.elevatorMoveTo3Totes = false;
        this.elevatorPIDOnState = false;
        this.elevatorPIDOffState = false;
        this.elevatorStopState = false;
        this.elevatorUpState = false;
        this.elevatorDownState = false;
        this.elevatorVelocityOverrideState = 0;
        this.elevatorIgnoreSensorsState = false;
        this.elevatorUseSensorsState = true;
        this.elevatorZeroEncodersState = false;

        //Arm 
        this.armMacroExtendState = false;
        this.armMacroRetractState = false;
        this.armExtenderExtendOverrideState = false;
        this.armExtenderRetractOverrideState = false;
        this.armTiltExtendOverrideState = false;
        this.armTiltRetractOverrideState = false;
        this.armTromboneExtendOverrideState = false;
        this.armTromboneRetractOverrideState = false;

        //Intake
        this.intakeUpState = false;
        this.intakeDownState = false;
        this.intakeRightExtendOverrideState = false;
        this.intakeRightRetractOverrideState = false;
        this.intakeLeftExtendOverrideState = false;
        this.intakeLeftRetractOverrideState = false;
        this.intakeForwardState = false;
        this.intakeBackwardState = false;
        this.intakeJitterState = false;
    }

    public double getDriveTrainXVelocity()
    {
        return this.driveTrainXVelocity;
    }

    public void setDriveTrainXVelocity(double driveTrainXVelocity)
    {
        this.driveTrainXVelocity = driveTrainXVelocity;
    }

    public double getDriveTrainYVelocity()
    {
        return this.driveTrainYVelocity;
    }

    public void setDriveTrainYVelocity(double driveTrainYVelocity)
    {
        this.driveTrainYVelocity = driveTrainYVelocity;
    }

    public boolean getDriveTrainSimpleMode()
    {
        return this.driveTrainSimpleMode;
    }

    public void setDriveTrainSimpleMode(boolean driveTrainSimpleMode)
    {
        this.driveTrainSimpleMode = driveTrainSimpleMode;
    }

    public double getDriveTrainLeftPosition()
    {
        return this.driveTrainLeftPosition;
    }

    public void setDriveTrainLeftPosition(double driveTrainLeftPosition)
    {
        this.driveTrainLeftPosition = driveTrainLeftPosition;
    }

    public double getDriveTrainRightPosition()
    {
        return this.driveTrainRightPosition;
    }

    public void setDriveTrainRightPosition(double driveTrainRightPosition)
    {
        this.driveTrainRightPosition = driveTrainRightPosition;
    }

    public boolean getDriveTrainPositionMode()
    {
        return this.driveTrainPositionMode;
    }

    public void setDriveTrainPositionMode(boolean driveTrainPositionMode)
    {
        this.driveTrainPositionMode = driveTrainPositionMode;
    }

    public boolean getElevatorContainerMacroState()
    {
        return this.elevatorContainerMacroState;
    }

    public void setElevatorContainerMacroState(boolean elevatorContainerMacroState)
    {
        this.elevatorContainerMacroState = elevatorContainerMacroState;
    }

    public boolean getElevatorSetStateToFloor()
    {
        return this.elevatorSetStateToFloor;
    }

    public void setElevatorSetStateToFloor(boolean elevatorSetStateToFloor)
    {
        this.elevatorSetStateToFloor = elevatorSetStateToFloor;
    }

    public boolean getElevatorSetStateToPlatform()
    {
        return this.elevatorSetStateToPlatform;
    }

    public void setElevatorSetStateToPlatform(boolean elevatorSetStateToPlatform)
    {
        this.elevatorSetStateToPlatform = elevatorSetStateToPlatform;
    }

    public boolean getElevatorSetStateToStep()
    {
        return this.elevatorSetStateToStep;
    }

    public void setElevatorSetStateToStep(boolean elevatorSetStateToStep)
    {
        this.elevatorSetStateToStep = elevatorSetStateToStep;
    }

    public boolean getElevatorMoveTo0Totes()
    {
        return this.elevatorMoveTo0Totes;
    }

    public void setElevatorMoveTo0Totes(boolean elevatorMoveTo0Totes)
    {
        this.elevatorMoveTo0Totes = elevatorMoveTo0Totes;
    }

    public boolean getElevatorMoveTo1Tote()
    {
        return this.elevatorMoveTo1Tote;
    }

    public void setElevatorMoveTo1Tote(boolean elevatorMoveTo1Tote)
    {
        this.elevatorMoveTo1Tote = elevatorMoveTo1Tote;
    }

    public boolean getElevatorMoveTo2Totes()
    {
        return this.elevatorMoveTo2Totes;
    }

    public void setElevatorMoveTo2Totes(boolean elevatorMoveTo2Totes)
    {
        this.elevatorMoveTo2Totes = elevatorMoveTo2Totes;
    }

    public boolean getElevatorMoveTo3Totes()
    {
        return this.elevatorMoveTo3Totes;
    }

    public void setElevatorMoveTo3Totes(boolean elevatorMoveTo3Totes)
    {
        this.elevatorMoveTo3Totes = elevatorMoveTo3Totes;
    }

    public boolean getElevatorTotePickUpMacroState()
    {
        return elevatorTotePickUpMacroState;
    }

    public void setElevatorTotePickUpMacroState(boolean elevatorTotePickUpMacroState)
    {
        this.elevatorTotePickUpMacroState = elevatorTotePickUpMacroState;
    }

    public boolean getElevatorPIDOnState()
    {
        return this.elevatorPIDOnState;
    }

    public void setElevatorPIDOnState(boolean elevatorPIDOnState)
    {
        this.elevatorPIDOnState = elevatorPIDOnState;
    }

    public boolean getElevatorPIDOffState()
    {
        return this.elevatorPIDOffState;
    }

    public void setElevatorPIDOffState(boolean elevatorPIDOffState)
    {
        this.elevatorPIDOffState = elevatorPIDOffState;
    }

    public boolean getElevatorStopState()
    {
        return this.elevatorStopState;
    }

    public void setElevatorStopState(boolean elevatorStopState)
    {
        this.elevatorStopState = elevatorStopState;
    }

    public boolean getElevatorUpState()
    {
        return this.elevatorUpState;
    }

    public void setElevatorUpState(boolean elevatorUpState)
    {
        this.elevatorUpState = elevatorUpState;
    }

    public boolean getElevatorDownState()
    {
        return this.elevatorDownState;
    }

    public void setElevatorDownState(boolean elevatorDownState)
    {
        this.elevatorDownState = elevatorDownState;
    }

    public boolean getElevatorMoveToBottomState()
    {
        return elevatorMoveToBottomState;
    }

    public void setElevatorMoveToBottomState(boolean elevatorMoveToBottomState)
    {
        this.elevatorMoveToBottomState = elevatorMoveToBottomState;
    }

    public boolean getArmMacroExtendState()
    {
        return this.armMacroExtendState;
    }

    public void setElevatorVelocityOverrideState(double velocity)
    {
        this.elevatorVelocityOverrideState = velocity;
    }

    public double getElevatorVelocityOverrideState()
    {
        return this.elevatorVelocityOverrideState;
    }

    public void setElevatorIgnoreSensorsState(boolean elevatorIgnoreSensorsState)
    {
        this.elevatorIgnoreSensorsState = elevatorIgnoreSensorsState;
    }

    public boolean getElevatorIgnoreSensorsState()
    {
        return this.elevatorIgnoreSensorsState;
    }

    public void setElevatorUseSensorsState(boolean elevatorUseSensorsState)
    {
        this.elevatorUseSensorsState = elevatorUseSensorsState;
    }

    public boolean getElevatorUseSensorsState()
    {
        return this.elevatorUseSensorsState;
    }

    public void setElevatorZeroEncodersState(boolean elevatorZeroEncodersState)
    {
        this.elevatorZeroEncodersState = elevatorZeroEncodersState;
    }

    public boolean getElevatorZeroEncoderState()
    {
        return this.elevatorZeroEncodersState;
    }

    public void setElevatorSlowButton(boolean elevatorSlowState)
    {
        this.elevatorSlowState = elevatorSlowState;
    }

    public boolean getElevatorSlowButton()
    {
        return this.elevatorSlowState;
    }

    public void setElevatorRegularSpeedButton(boolean elevatorRegularSpeedState)
    {
        this.elevatorRegularSpeedState = elevatorRegularSpeedState;
    }

    public boolean getElevatorRegularSpeedButton()
    {
        return this.elevatorRegularSpeedState;
    }

    public void setElevatorFastButton(boolean elevatorFastState)
    {
        this.elevatorFastState = elevatorFastState;
    }

    public boolean getElevatorFastButton()
    {
        return this.elevatorFastState;
    }

    public void setArmMacroExtendState(boolean armMacroExtendState)
    {
        this.armMacroExtendState = armMacroExtendState;
    }

    public boolean getArmMacroRetractState()
    {
        return this.armMacroRetractState;
    }

    public void setArmMacroRetractState(boolean armMacroRetractState)
    {
        this.armMacroRetractState = armMacroRetractState;
    }

    public boolean getArmExtenderExtendOverrideState()
    {
        return this.armExtenderExtendOverrideState;
    }

    public void setArmExtenderExtendOverrideState(boolean armElevatorExtendOverrideState)
    {
        this.armExtenderExtendOverrideState = armElevatorExtendOverrideState;
    }

    public boolean getArmExtenderRetractOverrideState()
    {
        return this.armExtenderRetractOverrideState;
    }

    public void setArmExtenderRetractOverrideState(boolean armElevatorRetractOverrideState)
    {
        this.armExtenderRetractOverrideState = armElevatorRetractOverrideState;
    }

    public boolean getArmTiltExtendOverrideState()
    {
        return this.armTiltExtendOverrideState;
    }

    public void setArmTiltExtendOverrideState(boolean armTiltExtendOverrideState)
    {
        this.armTiltExtendOverrideState = armTiltExtendOverrideState;
    }

    public boolean getArmTiltRetractOverrideState()
    {
        return this.armTiltRetractOverrideState;
    }

    public void setArmTiltRetractOverrideState(boolean armTiltRetractOverrideState)
    {
        this.armTiltRetractOverrideState = armTiltRetractOverrideState;
    }

    public boolean getArmTromboneExtendOverrideState()
    {
        return this.armTromboneExtendOverrideState;
    }

    public void setArmTromboneExtendOverrideState(boolean armTromboneExtendOverrideState)
    {
        this.armTromboneExtendOverrideState = armTromboneExtendOverrideState;
    }

    public boolean getArmTromboneRetractOverrideState()
    {
        return this.armTromboneRetractOverrideState;
    }

    public void setArmTromboneRetractOverrideState(boolean armTromboneRetractOverrideState)
    {
        this.armTromboneRetractOverrideState = armTromboneRetractOverrideState;
    }

    public boolean getIntakeUpState()
    {
        return this.intakeUpState;
    }

    public void setIntakeUpState(boolean intakeUpState)
    {
        this.intakeUpState = intakeUpState;
    }

    public boolean getIntakeDownState()
    {
        return this.intakeDownState;
    }

    public void setIntakeDownState(boolean intakeDownState)
    {
        this.intakeDownState = intakeDownState;
    }

    public boolean getIntakeRightExtendOverrideState()
    {
        return this.intakeRightExtendOverrideState;
    }

    public void setIntakeRightExtendOverrideState(boolean intakeRightExtendOverrideState)
    {
        this.intakeRightExtendOverrideState = intakeRightExtendOverrideState;
    }

    public boolean getIntakeRightRetractOverrideState()
    {
        return this.intakeRightRetractOverrideState;
    }

    public void setIntakeRightRetractOverrideState(boolean intakeRightRetractOverrideState)
    {
        this.intakeRightRetractOverrideState = intakeRightRetractOverrideState;
    }

    public boolean getIntakeLeftExtendOverrideState()
    {
        return this.intakeLeftExtendOverrideState;
    }

    public void setIntakeLeftExtendOverrideState(boolean intakeLeftExtendOverrideState)
    {
        this.intakeLeftExtendOverrideState = intakeLeftExtendOverrideState;
    }

    public boolean getIntakeLeftRetractOverrideState()
    {
        return this.intakeLeftRetractOverrideState;
    }

    public void setIntakeLeftRetractOverrideState(boolean intakeLeftRetractOverrideState)
    {
        this.intakeLeftRetractOverrideState = intakeLeftRetractOverrideState;
    }

    public boolean getIntakeForwardState()
    {
        return this.intakeForwardState;
    }

    public void setIntakeForwardState(boolean intakeForwardState)
    {
        this.intakeForwardState = intakeForwardState;
    }

    public boolean getIntakeBackwardState()
    {
        return this.intakeBackwardState;
    }

    public void setIntakeBackwardState(boolean intakeBackwardState)
    {
        this.intakeBackwardState = intakeBackwardState;
    }

    public boolean getIntakeJitterState()
    {
        return this.intakeJitterState;
    }

    public void setIntakeJitterState(boolean intakeJitterState)
    {
        this.intakeJitterState = intakeJitterState;
    }

}
