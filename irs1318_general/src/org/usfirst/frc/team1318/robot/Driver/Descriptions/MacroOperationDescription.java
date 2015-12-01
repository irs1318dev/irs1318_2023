package org.usfirst.frc.team1318.robot.Driver.Descriptions;

import java.util.function.Supplier;

import org.usfirst.frc.team1318.robot.Driver.IControlTask;
import org.usfirst.frc.team1318.robot.Driver.Operation;

public class MacroOperationDescription extends OperationDescription
{
    private final int userInputDeviceButton;
    private final int userInputDevicePovValue;
    private final Operation[] affectedOperations;
    private final Supplier<IControlTask> taskSupplier;

    public MacroOperationDescription(
        UserInputDevice userInputDevice,
        int userInputDeviceButton,
        int povValue,
        Supplier<IControlTask> taskSupplier,
        Operation... affectedOperations)
    {
        super(OperationType.None, userInputDevice);

        this.userInputDeviceButton = userInputDeviceButton;
        this.userInputDevicePovValue = povValue;
        this.affectedOperations = affectedOperations;
        this.taskSupplier = taskSupplier;
    }

    public MacroOperationDescription(
        UserInputDevice userInputDevice,
        int userInputDeviceButton,
        Supplier<IControlTask> taskSupplier,
        Operation... affectedOperations)
    {
        this(userInputDevice, userInputDeviceButton, 0, taskSupplier, affectedOperations);
    }

    public int getUserInputDeviceButton()
    {
        return this.userInputDeviceButton;
    }

    public int getUserInputDevicePovValue()
    {
        return this.userInputDevicePovValue;
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
