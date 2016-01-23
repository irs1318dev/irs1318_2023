package org.usfirst.frc.team1318.robot.Driver.Descriptions;

import org.usfirst.frc.team1318.robot.Driver.UserInputDeviceButton;
import org.usfirst.frc.team1318.robot.Driver.Buttons.ButtonType;

/**
 * Describes an operation that will give a boolean (true or false) value.
 * 
 */
public class DigitalOperationDescription extends OperationDescription
{
    private final UserInputDeviceButton userInputDeviceButton;
    private final int userInputDevicePovValue;
    private final ButtonType buttonType;
    private final DigitalSensor sensor;

    public DigitalOperationDescription(
        UserInputDevice userInputDevice,
        int povValue,
        ButtonType buttonType)
    {
        this(userInputDevice, UserInputDeviceButton.JOYSTICK_POV, povValue, DigitalSensor.None, buttonType);
    }

    public DigitalOperationDescription(
        UserInputDevice userInputDevice,
        UserInputDeviceButton userInputDeviceButton,
        ButtonType buttonType)
    {
        this(userInputDevice, userInputDeviceButton, 0, DigitalSensor.None, buttonType);
    }

    public DigitalOperationDescription(
        UserInputDevice userInputDevice,
        DigitalSensor sensor,
        ButtonType buttonType)
    {
        this(userInputDevice, UserInputDeviceButton.NONE, 0, sensor, buttonType);
    }

    private DigitalOperationDescription(
        UserInputDevice userInputDevice,
        UserInputDeviceButton userInputDeviceButton,
        int povValue,
        DigitalSensor sensor,
        ButtonType buttonType)
    {
        super(OperationType.Digital, userInputDevice);

        this.userInputDeviceButton = userInputDeviceButton;
        this.userInputDevicePovValue = povValue;
        this.sensor = sensor;
        this.buttonType = buttonType;
    }

    public UserInputDeviceButton getUserInputDeviceButton()
    {
        return this.userInputDeviceButton;
    }

    public int getUserInputDevicePovValue()
    {
        return this.userInputDevicePovValue;
    }

    public DigitalSensor getSensor()
    {
        return this.sensor;
    }

    public ButtonType getButtonType()
    {
        return this.buttonType;
    }
}
