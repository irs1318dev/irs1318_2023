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
    private boolean elevatorMacroState;
    private boolean elevatorHeight0State;
    private boolean elevatorHeight1State;
    private boolean elevatorHeight2State;
    private boolean elevatorHeight3State;
    private boolean elevatorHeight4State;
    private boolean elevatorHeight5State;
    private boolean elevatorHeight6State;
    private boolean elevatorHeight7State;
    private double elevatorOverrideState;

    //Arm
    private boolean armMacroState;
    private boolean armExtenderOverrideState;
    private boolean armTiltOverrideState;
    private boolean armTromboneOverrideState;

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
        this.elevatorMacroState = false;
        this.elevatorHeight0State = false;
        this.elevatorHeight1State = false;
        this.elevatorHeight2State = false;
        this.elevatorHeight3State = false;
        this.elevatorHeight4State = false;
        this.elevatorHeight5State = false;
        this.elevatorHeight6State = false;
        this.elevatorHeight7State = false;
        this.elevatorOverrideState = 0.0;

        //Arm 
        this.armMacroState = false;
        this.armExtenderOverrideState = false;
        this.armTiltOverrideState = false;
        this.armTromboneOverrideState = false;
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

    public boolean getElevatorMacroState()
    {
        return elevatorMacroState;
    }

    public void setElevatorMacroState(boolean elevatorMacroState)
    {
        this.elevatorMacroState = elevatorMacroState;
    }

    public boolean getElevatorHeight0State()
    {
        return elevatorHeight0State;
    }

    public void setElevatorHeight0State(boolean elevatorHeight0State)
    {
        this.elevatorHeight0State = elevatorHeight0State;
    }

    public boolean getElevatorHeight1State()
    {
        return elevatorHeight1State;
    }

    public void setElevatorHeight1State(boolean elevatorHeight1State)
    {
        this.elevatorHeight1State = elevatorHeight1State;
    }

    public boolean getElevatorHeight2State()
    {
        return elevatorHeight2State;
    }

    public void setElevatorHeight2State(boolean elevatorHeight2State)
    {
        this.elevatorHeight2State = elevatorHeight2State;
    }

    public boolean getElevatorHeight3State()
    {
        return elevatorHeight3State;
    }

    public void setElevatorHeight3State(boolean elevatorHeight3State)
    {
        this.elevatorHeight3State = elevatorHeight3State;
    }

    public boolean getElevatorHeight4State()
    {
        return elevatorHeight4State;
    }

    public void setElevatorHeight4State(boolean elevatorHeight4State)
    {
        this.elevatorHeight4State = elevatorHeight4State;
    }

    public boolean getElevatorHeight5State()
    {
        return elevatorHeight5State;
    }

    public void setElevatorHeight5State(boolean elevatorHeight5State)
    {
        this.elevatorHeight5State = elevatorHeight5State;
    }

    public boolean getElevatorHeight6State()
    {
        return elevatorHeight6State;
    }

    public void setElevatorHeight6State(boolean elevatorHeight6State)
    {
        this.elevatorHeight6State = elevatorHeight6State;
    }

    public boolean getElevatorHeight7State()
    {
        return elevatorHeight7State;
    }

    public void setElevatorHeight7State(boolean elevatorHeight7State)
    {
        this.elevatorHeight7State = elevatorHeight7State;
    }

    public double getElevatorOverrideState()
    {
        return elevatorOverrideState;
    }

    public void setElevatorOverrideState(double elevatorOverrideState)
    {
        this.elevatorOverrideState = elevatorOverrideState;
    }

    public boolean getArmMacroState()
    {
        return armMacroState;
    }

    public void setArmMacroState(boolean armMacroOverrideState)
    {
        this.armMacroState = armMacroOverrideState;
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
}
