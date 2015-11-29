package org.usfirst.frc.team1318.robot.Driver.Descriptions;

import org.usfirst.frc.team1318.robot.Driver.Buttons.AnalogAxis;

/**
 * Describes an operation that will give a double value (typically between -1 and 1).
 * 
 */
public class AnalogOperationDescription extends OperationDescription
{
    private final AnalogAxis userInputDeviceAxis;

    public AnalogOperationDescription(UserInputDevice userInputDevice, AnalogAxis userInputDeviceAxis)
    {
        super(OperationType.Analog, userInputDevice);

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
