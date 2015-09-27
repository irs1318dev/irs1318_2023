package org.usfirst.frc.team1318.robot.Driver.Descriptions;

public abstract class OperationDescription
{
    private final DriverOperationType type;
    private final UserInputDevice userInputDevice;

    protected OperationDescription(DriverOperationType type, UserInputDevice userInputDevice)
    {
        this.type = type;
        this.userInputDevice = userInputDevice;
    }

    public DriverOperationType getType()
    {
        return this.type;
    }

    public UserInputDevice getUserInputDevice()
    {
        return this.userInputDevice;
    }
}
