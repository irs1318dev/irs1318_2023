package frc.robot.driver.common.descriptions;

import java.util.function.Supplier;

import frc.robot.driver.Operation;
import frc.robot.driver.Shift;
import frc.robot.driver.common.IControlTask;
import frc.robot.driver.common.UserInputDeviceButton;
import frc.robot.driver.common.buttons.ButtonType;

public class MacroOperationDescription extends OperationDescription
{
    private final boolean clearInterrupt;
    private final UserInputDeviceButton userInputDeviceButton;
    private final int userInputDevicePovValue;
    private final DigitalSensor sensor;
    private final ButtonType buttonType;
    private final Supplier<IControlTask> taskSupplier;
    private final Operation[] affectedOperations;
    private final Operation[] macroCancelOperations;

    /**
     * Initializes a new MacroOperationDescription based on a user interaction
     * @param userInputDevice which device will perform the macro operation (driver or codriver joystick)
     * @param userInputDeviceButton the button on the device that performs the macro operation
     * @param buttonType the behavior type to use for the macro operation
     * @param taskSupplier the function that creates the tasks that should be performed by the macro
     * @param affectedOperations the list of operations that will be utilized by this macro
     */
    public MacroOperationDescription(
        UserInputDevice userInputDevice,
        UserInputDeviceButton userInputDeviceButton,
        ButtonType buttonType,
        Supplier<IControlTask> taskSupplier,
        Operation[] affectedOperations)
    {
        this(
            true,
            userInputDevice,
            userInputDeviceButton,
            0,
            DigitalSensor.None,
            Shift.Any,
            buttonType,
            taskSupplier,
            affectedOperations,
            null);
    }

    /**
     * Initializes a new MacroOperationDescription based on a user interaction
     * @param userInputDevice which device will perform the macro operation (driver or codriver joystick)
     * @param userInputDeviceButton the button on the device that performs the macro operation
     * @param requiredShift the shift button that must be applied to perform macro
     * @param buttonType the behavior type to use for the macro operation
     * @param taskSupplier the function that creates the tasks that should be performed by the macro
     * @param affectedOperations the list of operations that will be utilized by this macro
     * @param macroCancelOperations the list of operations that can be used to cancel this macro
     */
    public MacroOperationDescription(
        UserInputDevice userInputDevice,
        UserInputDeviceButton userInputDeviceButton,
        Shift requiredShift,
        ButtonType buttonType,
        Supplier<IControlTask> taskSupplier,
        Operation[] affectedOperations,
        Operation[] macroCancelOperations)
    {
        this(
            true,
            userInputDevice,
            userInputDeviceButton,
            0,
            DigitalSensor.None,
            requiredShift,
            buttonType,
            taskSupplier,
            affectedOperations,
            macroCancelOperations);
    }

    /**
     * Initializes a new MacroOperationDescription based on a user interaction on the POV
     * @param userInputDevice which device will perform the macro operation (driver or codriver joystick)
     * @param povValue the value of the POV (hat) used to perform the macro operation
     * @param buttonType the behavior type to use for the macro operation
     * @param taskSupplier the function that creates the tasks that should be performed by the macro
     * @param affectedOperations the list of operations that will be utilized by this macro
     */
    public MacroOperationDescription(
        UserInputDevice userInputDevice,
        int povValue,
        ButtonType buttonType,
        Supplier<IControlTask> taskSupplier,
        Operation[] affectedOperations)
    {
        this(
            true,
            userInputDevice,
            UserInputDeviceButton.JOYSTICK_POV,
            povValue,
            DigitalSensor.None,
            Shift.Any,
            buttonType,
            taskSupplier,
            affectedOperations,
            null);
    }

    /**
     * Initializes a new MacroOperationDescription based on a user interaction on the POV
     * @param userInputDevice which device will perform the macro operation (driver or codriver joystick)
     * @param povValue the value of the POV (hat) used to perform the macro operation
     * @param requiredShift the shift button that must be applied to perform macro
     * @param buttonType the behavior type to use for the macro operation
     * @param taskSupplier the function that creates the tasks that should be performed by the macro
     * @param affectedOperations the list of operations that will be utilized by this macro
     * @param macroCancelOperations the list of operations that can be used to cancel this macro
     */
    public MacroOperationDescription(
        UserInputDevice userInputDevice,
        int povValue,
        Shift requiredShift,
        ButtonType buttonType,
        Supplier<IControlTask> taskSupplier,
        Operation[] affectedOperations,
        Operation[] macroCancelOperations)
    {
        this(
            true,
            userInputDevice,
            UserInputDeviceButton.JOYSTICK_POV,
            povValue,
            DigitalSensor.None,
            requiredShift,
            buttonType,
            taskSupplier,
            affectedOperations,
            macroCancelOperations);
    }

    /**
     * Initializes a new MacroOperationDescription based on a sensor
     * @param sensor the sensor that triggers the macro operation
     * @param buttonType the behavior type to use for the macro operation
     * @param taskSupplier the function that creates the tasks that should be performed by the macro
     * @param affectedOperations the list of operations that will be utilized by this macro
     */
    public MacroOperationDescription(
        DigitalSensor sensor,
        ButtonType buttonType,
        Supplier<IControlTask> taskSupplier,
        Operation[] affectedOperations)
    {
        this(
            true,
            UserInputDevice.Sensor,
            UserInputDeviceButton.NONE,
            0,
            sensor,
            Shift.Any,
            buttonType,
            taskSupplier,
            affectedOperations,
            null);
    }

    /**
     * Initializes a new MacroOperationDescription based on a user interaction on the POV
     * @param clearInterrupt whether to clear the interruption of the operations when the macro completes
     * @param userInputDevice which device will perform the macro operation (driver or codriver joystick)
     * @param userInputDeviceButton the button on the device that performs the macro operation
     * @param povValue the value of the POV (hat) used to perform the macro operation
     * @param sensor the sensor that triggers the macro operation
     * @param requiredShift the shift button that must be applied to perform macro
     * @param buttonType the behavior type to use for the macro operation
     * @param taskSupplier the function that creates the tasks that should be performed by the macro
     * @param affectedOperations the list of operations that will be utilized by this macro
     * @param macroCancelOperations the list of operations that can be used to cancel this macro
     */
    private MacroOperationDescription(
        boolean clearInterrupt,
        UserInputDevice userInputDevice,
        UserInputDeviceButton userInputDeviceButton,
        int povValue,
        DigitalSensor sensor,
        Shift requiredShift,
        ButtonType buttonType,
        Supplier<IControlTask> taskSupplier,
        Operation[] affectedOperations,
        Operation[] macroCancelOperations)
    {
        super(OperationType.None, userInputDevice, requiredShift);

        this.clearInterrupt = clearInterrupt;
        this.userInputDeviceButton = userInputDeviceButton;
        this.userInputDevicePovValue = povValue;
        this.sensor = sensor;
        this.buttonType = buttonType;
        this.taskSupplier = taskSupplier;
        this.affectedOperations = affectedOperations;
        this.macroCancelOperations = macroCancelOperations;
    }

    public boolean shouldClearInterrupt()
    {
        return this.clearInterrupt;
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

    public IControlTask constructTask()
    {
        return this.taskSupplier.get();
    }

    public Operation[] getMacroCancelOperations()
    {
        if (this.macroCancelOperations != null)
        {
            return this.macroCancelOperations;
        }

        return this.affectedOperations;
    }

    public Operation[] getAffectedOperations()
    {
        return this.affectedOperations;
    }
}
