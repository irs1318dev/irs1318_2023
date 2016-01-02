package org.usfirst.frc.team1318.robot.Driver.Descriptions;

import org.usfirst.frc.team1318.robot.Driver.Buttons.ButtonType;

/**
 * Describes an operation that will give a boolean (true or false) value.
 * 
 */
public class DigitalOperationDescription extends OperationDescription
{
    private final int userInputDeviceButton;
    private final int userInputDevicePovValue;
    private final ButtonType buttonType;

    public DigitalOperationDescription(
        UserInputDevice userInputDevice,
        int userInputDeviceButton,
        int povValue,
        ButtonType buttonType)
    {
        super(OperationType.Digital, userInputDevice);

        this.userInputDeviceButton = userInputDeviceButton;
        this.userInputDevicePovValue = povValue;
        this.buttonType = buttonType;
    }

    public DigitalOperationDescription(
        UserInputDevice userInputDevice,
        int userInputDeviceButton,
        ButtonType buttonType)
    {
        this(userInputDevice, userInputDeviceButton, 0, buttonType);
    }

    public int getUserInputDeviceButton()
    {
        return this.userInputDeviceButton;
    }

    public int getUserInputDevicePovValue()
    {
        return this.userInputDevicePovValue;
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
