package org.usfirst.frc.team1318.robot.Driver.Descriptions;

import org.usfirst.frc.team1318.robot.Driver.Buttons.AnalogAxis;

public class AnalogOperationDescription extends OperationDescription
{
    private final AnalogAxis userInputDeviceAxis;

    public AnalogOperationDescription(UserInputDevice userInputDevice, AnalogAxis userInputDeviceAxis)
    {
        super(DriverOperationType.Analog, userInputDevice);

        this.userInputDeviceAxis = userInputDeviceAxis;
    }

    public AnalogAxis getUserInputDeviceAxis()
    {
        return this.userInputDeviceAxis;
    }

    public double getDefaultValue()
    {
        return 0.0;
    }
}
