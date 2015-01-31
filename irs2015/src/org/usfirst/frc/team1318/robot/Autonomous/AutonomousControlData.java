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
    private boolean elevatorSetStateToFloor;
    private boolean elevatorSetStateToPlatform;
    private boolean elevatorSetStateToStep;
    private boolean elevatorMoveTo0Totes;
    private boolean elevatorMoveTo1Tote;
    private boolean elevatorMoveTo2Totes;
    private boolean elevatorMoveTo3Totes;
    private boolean elevatorPIDToggleState;
    private boolean elevatorStopState;
    private boolean elevatorUpState;
    private boolean elevatorDownState;

    //Arm
    private boolean armMacroExtendState;
    private boolean armMacroRetractState;
    private boolean armExtenderOverrideState;
    private boolean armTiltOverrideState;
    private boolean armTromboneOverrideState;

    //Intake 
    private boolean intakeUpState;
    private boolean intakeDownState;
    private boolean intakeRightToggleOverrideState;
    private boolean intakeLeftToggleOverrideState;
    private boolean intakeForwardState;
    private boolean intakeBackwardState;

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
        this.elevatorPIDToggleState = false;
        this.elevatorStopState = false;
        this.elevatorUpState = false;
        this.elevatorDownState = false;

        //Arm 
        this.armMacroExtendState = false;
        this.armMacroRetractState = false;
        this.armExtenderOverrideState = false;
        this.armTiltOverrideState = false;
        this.armTromboneOverrideState = false;

        //Intake
        this.intakeUpState = false;
        this.intakeDownState = false;
        this.intakeRightToggleOverrideState = false;
        this.intakeLeftToggleOverrideState = false;
        this.intakeForwardState = false;
        this.intakeBackwardState = false;
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
        return elevatorContainerMacroState;
    }

    public void setElevatorContainerMacroState(boolean elevatorContainerMacroState)
    {
        this.elevatorContainerMacroState = elevatorContainerMacroState;
    }

    public boolean getElevatorSetStateToFloor()
    {
        return elevatorSetStateToFloor;
    }

    public void setElevatorSetStateToFloor(boolean elevatorSetStateToFloor)
    {
        this.elevatorSetStateToFloor = elevatorSetStateToFloor;
    }

    public boolean getElevatorSetStateToPlatform()
    {
        return elevatorSetStateToPlatform;
    }

    public void setElevatorSetStateToPlatform(boolean elevatorSetStateToPlatform)
    {
        this.elevatorSetStateToPlatform = elevatorSetStateToPlatform;
    }

    public boolean getElevatorSetStateToStep()
    {
        return elevatorSetStateToStep;
    }

    public void setElevatorSetStateToStep(boolean elevatorSetStateToStep)
    {
        this.elevatorSetStateToStep = elevatorSetStateToStep;
    }

    public boolean getElevatorMoveTo0Totes()
    {
        return elevatorMoveTo0Totes;
    }

    public void setElevatorMoveTo0Totes(boolean elevatorMoveTo0Totes)
    {
        this.elevatorMoveTo0Totes = elevatorMoveTo0Totes;
    }

    public boolean getElevatorMoveTo1Tote()
    {
        return elevatorMoveTo1Tote;
    }

    public void setElevatorMoveTo1Tote(boolean elevatorMoveTo1Tote)
    {
        this.elevatorMoveTo1Tote = elevatorMoveTo1Tote;
    }

    public boolean getElevatorMoveTo2Totes()
    {
        return elevatorMoveTo2Totes;
    }

    public void setElevatorMoveTo2Totes(boolean elevatorMoveTo2Totes)
    {
        this.elevatorMoveTo2Totes = elevatorMoveTo2Totes;
    }

    public boolean getElevatorMoveTo3Totes()
    {
        return elevatorMoveTo3Totes;
    }

    public void setElevatorMoveTo3Totes(boolean elevatorMoveTo3Totes)
    {
        this.elevatorMoveTo3Totes = elevatorMoveTo3Totes;
    }

    public boolean getElevatorPIDToggleState()
    {
        return elevatorPIDToggleState;
    }

    public void setElevatorPIDToggleState(boolean elevatorPIDToggleState)
    {
        this.elevatorPIDToggleState = elevatorPIDToggleState;
    }

    public boolean getElevatorStopState()
    {
        return elevatorStopState;
    }

    public void setElevatorStopState(boolean elevatorStopState)
    {
        this.elevatorStopState = elevatorStopState;
    }

    public boolean getElevatorUpState()
    {
        return elevatorUpState;
    }

    public void setElevatorUpState(boolean elevatorUpState)
    {
        this.elevatorUpState = elevatorUpState;
    }

    public boolean getElevatorDownState()
    {
        return elevatorDownState;
    }

    public void setElevatorDownState(boolean elevatorDownState)
    {
        this.elevatorDownState = elevatorDownState;
    }

    public boolean getArmMacroExtendState()
    {
        return armMacroExtendState;
    }

    public void setArmMacroExtendState(boolean armMacroExtendState)
    {
        this.armMacroExtendState = armMacroExtendState;
    }

    public boolean getArmMacroRetractState()
    {
        return armMacroRetractState;
    }

    public void setArmMacroRetractState(boolean armMacroRetractState)
    {
        this.armMacroRetractState = armMacroRetractState;
    }

    public boolean getArmExtenderOverrideState()
    {
        return armExtenderOverrideState;
    }

    public void setArmExtenderOverrideState(boolean armElevatorOverrideState)
    {
        this.armExtenderOverrideState = armElevatorOverrideState;
    }

    public boolean getArmTiltOverrideState()
    {
        return armTiltOverrideState;
    }

    public void setArmTiltOverrideState(boolean armTiltOverrideState)
    {
        this.armTiltOverrideState = armTiltOverrideState;
    }

    public boolean getArmTromboneOverrideState()
    {
        return armTromboneOverrideState;
    }

    public void setArmTromboneOverrideState(boolean armTromboneOverrideState)
    {
        this.armTromboneOverrideState = armTromboneOverrideState;
    }

    public boolean getIntakeUpState()
    {
        return intakeUpState;
    }

    public void setIntakeUpState(boolean intakeUpState)
    {
        this.intakeUpState = intakeUpState;
    }

    public boolean getIntakeDownState()
    {
        return intakeDownState;
    }

    public void setIntakeDownState(boolean intakeDownState)
    {
        this.intakeDownState = intakeDownState;
    }

    public boolean getIntakeRightToggleOverrideState()
    {
        return intakeRightToggleOverrideState;
    }

    public void setIntakeRightToggleOverrideState(boolean intakeRightToggleOverrideState)
    {
        this.intakeRightToggleOverrideState = intakeRightToggleOverrideState;
    }

    public boolean getIntakeLeftToggleOverrideState()
    {
        return intakeLeftToggleOverrideState;
    }

    public void setIntakeLeftToggleOverrideState(boolean intakeLeftToggleOverrideState)
    {
        this.intakeLeftToggleOverrideState = intakeLeftToggleOverrideState;
    }

    public boolean getIntakeForwardState()
    {
        return intakeForwardState;
    }

    public void setIntakeForwardState(boolean intakeForwardState)
    {
        this.intakeForwardState = intakeForwardState;
    }

    public boolean getIntakeBackwardState()
    {
        return intakeBackwardState;
    }

    public void setIntakeBackwardState(boolean intakeBackwardState)
    {
        this.intakeBackwardState = intakeBackwardState;
    }
}
