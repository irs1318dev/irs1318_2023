package org.usfirst.frc.team1318.robot.Driver.Descriptions;

import java.util.function.Supplier;

import org.usfirst.frc.team1318.robot.Driver.Operation;
import org.usfirst.frc.team1318.robot.Driver.Macros.MacroTask;

public class MacroOperationDescription extends OperationDescription
{
    private final int userInputDeviceButton;
    private final Operation[] affectedOperations;
    private final Supplier<MacroTask> macroSupplier;

    public MacroOperationDescription(
        UserInputDevice userInputDevice,
        int userInputDeviceButton,
        Supplier<MacroTask> macroSupplier,
        Operation... affectedOperations)
    {
        super(DriverOperationType.None, userInputDevice);

        this.userInputDeviceButton = userInputDeviceButton;
        this.affectedOperations = affectedOperations;
        this.macroSupplier = macroSupplier;
    }

    public int getUserInputDeviceButton()
    {
        return this.userInputDeviceButton;
    }

    public MacroTask constructMacroTask()
    {
        return this.macroSupplier.get();
    }

    public Operation[] getAffectedOperations()
    {
        return this.affectedOperations;
    }
}
