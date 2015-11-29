package org.usfirst.frc.team1318.robot.Driver.Descriptions;

import org.usfirst.frc.team1318.robot.Driver.Buttons.ButtonType;

/**
 * Describes an operation that will give a boolean (true or false) value.
 * 
 */
public class DigitalOperationDescription extends OperationDescription
{
    private final int userInputDeviceButton;
    private final ButtonType buttonType;

    public DigitalOperationDescription(UserInputDevice userInputDevice, int userInputDeviceButton, ButtonType buttonType)
    {
        super(OperationType.Digital, userInputDevice);

        this.userInputDeviceButton = userInputDeviceButton;
        this.buttonType = buttonType;
    }

    public int getUserInputDeviceButton()
    {
        return this.userInputDeviceButton;
    }

    public ButtonType getButtonType()
    {
        return this.buttonType;
    }

    public boolean getDefaultValue()
    {
        return false;
    }
}
