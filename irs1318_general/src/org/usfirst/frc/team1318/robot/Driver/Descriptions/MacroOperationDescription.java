package org.usfirst.frc.team1318.robot.Driver.Descriptions;

import org.usfirst.frc.team1318.robot.Driver.IControlTask;
import org.usfirst.frc.team1318.robot.Driver.Operation;
import org.usfirst.frc.team1318.robot.Driver.UserInputDeviceButton;

public class MacroOperationDescription extends OperationDescription
{
    private final UserInputDeviceButton userInputDeviceButton;
    private final int userInputDevicePovValue;
    private final DigitalSensor sensor;
    private final Operation[] affectedOperations;
    private final IControlTask task;

    public MacroOperationDescription(
        UserInputDevice userInputDevice,
        int povValue,
        IControlTask task,
        Operation... affectedOperations)
    {
        this(userInputDevice, UserInputDeviceButton.JOYSTICK_POV, povValue, DigitalSensor.None, task, affectedOperations);
    }

    public MacroOperationDescription(
        UserInputDevice userInputDevice,
        UserInputDeviceButton userInputDeviceButton,
        IControlTask task,
        Operation... affectedOperations)
    {
        this(userInputDevice, userInputDeviceButton, 0, DigitalSensor.None, task, affectedOperations);
    }

    public MacroOperationDescription(
        DigitalSensor sensor,
        IControlTask task,
        Operation... affectedOperations)
    {
        this(UserInputDevice.Sensor, UserInputDeviceButton.NONE, 0, sensor, task, affectedOperations);
    }

    private MacroOperationDescription(
        UserInputDevice userInputDevice,
        UserInputDeviceButton userInputDeviceButton,
        int povValue,
        DigitalSensor sensor,
        IControlTask task,
        Operation... affectedOperations)
    {
        super(OperationType.None, userInputDevice);

        this.userInputDeviceButton = userInputDeviceButton;
        this.userInputDevicePovValue = povValue;
        this.sensor = sensor;
        this.affectedOperations = affectedOperations;
        this.task = task;
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

    public IControlTask getTask()
    {
        return this.task;
    }

    public Operation[] getAffectedOperations()
    {
        return this.affectedOperations;
    }
}
