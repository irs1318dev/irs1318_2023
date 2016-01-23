package org.usfirst.frc.team1318.robot.Driver.Descriptions;

import java.util.function.Supplier;

import org.usfirst.frc.team1318.robot.Driver.IControlTask;
import org.usfirst.frc.team1318.robot.Driver.Operation;
import org.usfirst.frc.team1318.robot.Driver.UserInputDeviceButton;

public class MacroOperationDescription extends OperationDescription
{
    private final UserInputDeviceButton userInputDeviceButton;
    private final int userInputDevicePovValue;
    private final DigitalSensor sensor;
    private final Operation[] affectedOperations;
    private final Supplier<IControlTask> taskSupplier;

    public MacroOperationDescription(
        UserInputDevice userInputDevice,
        int povValue,
        Supplier<IControlTask> taskSupplier,
        Operation... affectedOperations)
    {
        this(userInputDevice, UserInputDeviceButton.JOYSTICK_POV, povValue, DigitalSensor.None, taskSupplier, affectedOperations);
    }

    public MacroOperationDescription(
        UserInputDevice userInputDevice,
        UserInputDeviceButton userInputDeviceButton,
        Supplier<IControlTask> taskSupplier,
        Operation... affectedOperations)
    {
        this(userInputDevice, userInputDeviceButton, 0, DigitalSensor.None, taskSupplier, affectedOperations);
    }

    public MacroOperationDescription(
        DigitalSensor sensor,
        Supplier<IControlTask> taskSupplier,
        Operation... affectedOperations)
    {
        this(UserInputDevice.Sensor, UserInputDeviceButton.NONE, 0, sensor, taskSupplier, affectedOperations);
    }

    private MacroOperationDescription(
        UserInputDevice userInputDevice,
        UserInputDeviceButton userInputDeviceButton,
        int povValue,
        DigitalSensor sensor,
        Supplier<IControlTask> taskSupplier,
        Operation... affectedOperations)
    {
        super(OperationType.None, userInputDevice);

        this.userInputDeviceButton = userInputDeviceButton;
        this.userInputDevicePovValue = povValue;
        this.sensor = sensor;
        this.affectedOperations = affectedOperations;
        this.taskSupplier = taskSupplier;
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

    public IControlTask constructTask()
    {
        return this.taskSupplier.get();
    }

    public Operation[] getAffectedOperations()
    {
        return this.affectedOperations;
    }
}
