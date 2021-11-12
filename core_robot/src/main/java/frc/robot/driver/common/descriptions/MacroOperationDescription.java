package frc.robot.driver.common.descriptions;

import java.util.function.Supplier;

import frc.robot.driver.IOperation;
import frc.robot.driver.MacroOperation;
import frc.robot.driver.Shift;
import frc.robot.driver.common.AnalogAxis;
import frc.robot.driver.common.IControlTask;
import frc.robot.driver.common.UserInputDeviceButton;
import frc.robot.driver.common.buttons.ButtonType;

public class MacroOperationDescription extends OperationDescription
{
    private final boolean clearInterrupt;
    private final UserInputDeviceButton userInputDeviceButton;
    private final int userInputDevicePovValue;
    private final AnalogAxis userInputDeviceAxis;
    private final ButtonType buttonType;
    private final Supplier<IControlTask> taskSupplier;
    private final IOperation[] affectedOperations;
    private final IOperation[] macroCancelOperations;

    /**
     * Initializes a new MacroOperationDescription based on a user interaction
     * @param operation the macro operation being described
     * @param userInputDevice which device will perform the macro operation (driver or operator joystick)
     * @param userInputDeviceButton the button on the device that performs the macro operation
     * @param buttonType the behavior type to use for the macro operation
     * @param taskSupplier the function that creates the tasks that should be performed by the macro
     * @param affectedOperations the list of operations that will be utilized by this macro
     */
    public MacroOperationDescription(
        MacroOperation operation,
        UserInputDevice userInputDevice,
        UserInputDeviceButton userInputDeviceButton,
        ButtonType buttonType,
        Supplier<IControlTask> taskSupplier,
        IOperation[] affectedOperations)
    {
        this(
            true,
            operation,
            userInputDevice,
            userInputDeviceButton,
            -1,
            AnalogAxis.NONE,
            0.0,
            0.0,
            null,
            null,
            buttonType,
            taskSupplier,
            affectedOperations,
            null);
    }

    /**
     * Initializes a new MacroOperationDescription based on a user interaction
     * @param operation the macro operation being described
     * @param userInputDevice which device will perform the macro operation (driver or operator joystick)
     * @param userInputDeviceButton the button on the device that performs the macro operation
     * @param relevantShifts the shifts that should be considered when checking if we should perform the macro
     * @param requiredShifts the shift button(s) that must be applied to perform macro
     * @param buttonType the behavior type to use for the macro operation
     * @param taskSupplier the function that creates the tasks that should be performed by the macro
     * @param affectedOperations the list of operations that will be utilized by this macro
     */
    public MacroOperationDescription(
        MacroOperation operation,
        UserInputDevice userInputDevice,
        UserInputDeviceButton userInputDeviceButton,
        Shift relevantShifts,
        Shift requiredShifts,
        ButtonType buttonType,
        Supplier<IControlTask> taskSupplier,
        IOperation[] affectedOperations)
    {
        this(
            true,
            operation,
            userInputDevice,
            userInputDeviceButton,
            -1,
            AnalogAxis.NONE,
            0.0,
            0.0,
            relevantShifts,
            requiredShifts,
            buttonType,
            taskSupplier,
            affectedOperations,
            null);
    }

    /**
     * Initializes a new MacroOperationDescription based on a user interaction
     * @param operation the macro operation being described
     * @param userInputDevice which device will perform the macro operation (driver or operator joystick)
     * @param userInputDeviceButton the button on the device that performs the macro operation
     * @param buttonType the behavior type to use for the macro operation
     * @param taskSupplier the function that creates the tasks that should be performed by the macro
     * @param affectedOperations the list of operations that will be utilized by this macro
     * @param macroCancelOperations the list of operations that can be used to cancel this macro
     */
    public MacroOperationDescription(
        MacroOperation operation,
        UserInputDevice userInputDevice,
        UserInputDeviceButton userInputDeviceButton,
        ButtonType buttonType,
        Supplier<IControlTask> taskSupplier,
        IOperation[] affectedOperations,
        IOperation[] macroCancelOperations)
    {
        this(
            true,
            operation,
            userInputDevice,
            userInputDeviceButton,
            -1,
            AnalogAxis.NONE,
            0.0,
            0.0,
            null,
            null,
            buttonType,
            taskSupplier,
            affectedOperations,
            macroCancelOperations);
    }

    /**
     * Initializes a new MacroOperationDescription based on a user interaction
     * @param operation the macro operation being described
     * @param userInputDevice which device will perform the macro operation (driver or operator joystick)
     * @param userInputDeviceButton the button on the device that performs the macro operation
     * @param relevantShifts the shifts that should be considered when checking if we should perform the macro
     * @param requiredShifts the shift button(s) that must be applied to perform macro
     * @param buttonType the behavior type to use for the macro operation
     * @param taskSupplier the function that creates the tasks that should be performed by the macro
     * @param affectedOperations the list of operations that will be utilized by this macro
     * @param macroCancelOperations the list of operations that can be used to cancel this macro
     */
    public MacroOperationDescription(
        MacroOperation operation,
        UserInputDevice userInputDevice,
        UserInputDeviceButton userInputDeviceButton,
        Shift relevantShifts,
        Shift requiredShifts,
        ButtonType buttonType,
        Supplier<IControlTask> taskSupplier,
        IOperation[] affectedOperations,
        IOperation[] macroCancelOperations)
    {
        this(
            true,
            operation,
            userInputDevice,
            userInputDeviceButton,
            -1,
            AnalogAxis.NONE,
            0.0,
            0.0,
            relevantShifts,
            requiredShifts,
            buttonType,
            taskSupplier,
            affectedOperations,
            macroCancelOperations);
    }

    /**
     * Initializes a new MacroOperationDescription based on a user interaction on the POV
     * @param operation the macro operation being described
     * @param userInputDevice which device will perform the macro operation (driver or operator joystick)
     * @param povValue the value of the POV (hat) used to perform the macro operation
     * @param buttonType the behavior type to use for the macro operation
     * @param taskSupplier the function that creates the tasks that should be performed by the macro
     * @param affectedOperations the list of operations that will be utilized by this macro
     */
    public MacroOperationDescription(
        MacroOperation operation,
        UserInputDevice userInputDevice,
        int povValue,
        ButtonType buttonType,
        Supplier<IControlTask> taskSupplier,
        IOperation[] affectedOperations)
    {
        this(
            true,
            operation,
            userInputDevice,
            UserInputDeviceButton.POV,
            povValue,
            AnalogAxis.NONE,
            0.0,
            0.0,
            null,
            null,
            buttonType,
            taskSupplier,
            affectedOperations,
            null);
    }

    /**
     * Initializes a new MacroOperationDescription based on a user interaction on the POV
     * @param operation the macro operation being described
     * @param userInputDevice which device will perform the macro operation (driver or operator joystick)
     * @param povValue the value of the POV (hat) used to perform the macro operation
     * @param relevantShifts the shifts that should be considered when checking if we should perform the macro
     * @param requiredShifts the shift button(s) that must be applied to perform macro
     * @param buttonType the behavior type to use for the macro operation
     * @param taskSupplier the function that creates the tasks that should be performed by the macro
     * @param affectedOperations the list of operations that will be utilized by this macro
     */
    public MacroOperationDescription(
        MacroOperation operation,
        UserInputDevice userInputDevice,
        int povValue,
        Shift relevantShifts,
        Shift requiredShifts,
        ButtonType buttonType,
        Supplier<IControlTask> taskSupplier,
        IOperation[] affectedOperations)
    {
        this(
            true,
            operation,
            userInputDevice,
            UserInputDeviceButton.POV,
            povValue,
            AnalogAxis.NONE,
            0.0,
            0.0,
            relevantShifts,
            requiredShifts,
            buttonType,
            taskSupplier,
            affectedOperations,
            null);
    }

    /**
     * Initializes a new MacroOperationDescription based on a user interaction on the POV
     * @param operation the macro operation being described
     * @param userInputDevice which device will perform the macro operation (driver or operator joystick)
     * @param povValue the value of the POV (hat) used to perform the macro operation
     * @param relevantShifts the shifts that should be considered when checking if we should perform the macro
     * @param requiredShifts the shift button(s) that must be applied to perform macro
     * @param buttonType the behavior type to use for the macro operation
     * @param taskSupplier the function that creates the tasks that should be performed by the macro
     * @param affectedOperations the list of operations that will be utilized by this macro
     * @param macroCancelOperations the list of operations that can be used to cancel this macro
     */
    public MacroOperationDescription(
        MacroOperation operation,
        UserInputDevice userInputDevice,
        int povValue,
        Shift relevantShifts,
        Shift requiredShifts,
        ButtonType buttonType,
        Supplier<IControlTask> taskSupplier,
        IOperation[] affectedOperations,
        IOperation[] macroCancelOperations)
    {
        this(
            true,
            operation,
            userInputDevice,
            UserInputDeviceButton.POV,
            povValue,
            AnalogAxis.NONE,
            0.0,
            0.0,
            relevantShifts,
            requiredShifts,
            buttonType,
            taskSupplier,
            affectedOperations,
            macroCancelOperations);
    }

    /**
     * Initializes a new MacroOperationDescription based on a user interaction on an axis
     * @param operation the macro operation being described
     * @param userInputDevice which device will indicate the operation (driver or operator joystick)
     * @param analogAxis the analog axis used to perform the operation
     * @param axisRangeMinValue the min value of the range that triggers the operation
     * @param axisRangeMaxValue the max value of the range that triggers the operation
     * @param buttonType the behavior type to use for the operation
     * @param taskSupplier the function that creates the tasks that should be performed by the macro
     * @param affectedOperations the list of operations that will be utilized by this macro
     */
    public MacroOperationDescription(
        MacroOperation operation,
        UserInputDevice userInputDevice,
        AnalogAxis analogAxis,
        double axisRangeMinValue,
        double axisRangeMaxValue,
        ButtonType buttonType,
        Supplier<IControlTask> taskSupplier,
        IOperation[] affectedOperations)
    {
        this(
            true,
            operation,
            userInputDevice,
            UserInputDeviceButton.ANALOG_AXIS_RANGE,
            -1,
            analogAxis,
            axisRangeMinValue,
            axisRangeMaxValue,
            null,
            null,
            buttonType,
            taskSupplier,
            affectedOperations,
            null);
    }

    /**
     * Initializes a new MacroOperationDescription based on a user interaction on an axis
     * @param operation the macro operation being described
     * @param userInputDevice which device will indicate the operation (driver or operator joystick)
     * @param analogAxis the analog axis used to perform the operation
     * @param axisRangeMinValue the min value of the range that triggers the operation
     * @param axisRangeMaxValue the max value of the range that triggers the operation
     * @param buttonType the behavior type to use for the operation
     * @param taskSupplier the function that creates the tasks that should be performed by the macro
     * @param affectedOperations the list of operations that will be utilized by this macro
     * @param macroCancelOperations the list of operations that can be used to cancel this macro
     */
    public MacroOperationDescription(
        MacroOperation operation,
        UserInputDevice userInputDevice,
        AnalogAxis analogAxis,
        double axisRangeMinValue,
        double axisRangeMaxValue,
        ButtonType buttonType,
        Supplier<IControlTask> taskSupplier,
        IOperation[] affectedOperations,
        IOperation[] macroCancelOperations)
    {
        this(
            true,
            operation,
            userInputDevice,
            UserInputDeviceButton.ANALOG_AXIS_RANGE,
            -1,
            analogAxis,
            axisRangeMinValue,
            axisRangeMaxValue,
            null,
            null,
            buttonType,
            taskSupplier,
            affectedOperations,
            macroCancelOperations);
    }

    /**
     * Initializes a new MacroOperationDescription based on a user interaction on an axis
     * @param operation the macro operation being described
     * @param userInputDevice which device will indicate the operation (driver or operator joystick)
     * @param analogAxis the analog axis used to perform the operation
     * @param axisRangeMinValue the min value of the range that triggers the operation
     * @param axisRangeMaxValue the max value of the range that triggers the operation
     * @param relevantShifts the shifts that should be considered when checking if we should perform the macro
     * @param requiredShifts the shift button(s) that must be applied to perform macro
     * @param buttonType the behavior type to use for the operation
     * @param taskSupplier the function that creates the tasks that should be performed by the macro
     * @param affectedOperations the list of operations that will be utilized by this macro
     */
    public MacroOperationDescription(
        MacroOperation operation,
        UserInputDevice userInputDevice,
        AnalogAxis analogAxis,
        double axisRangeMinValue,
        double axisRangeMaxValue,
        Shift relevantShifts,
        Shift requiredShifts,
        ButtonType buttonType,
        Supplier<IControlTask> taskSupplier,
        IOperation[] affectedOperations)
    {
        this(
            true,
            operation,
            userInputDevice,
            UserInputDeviceButton.ANALOG_AXIS_RANGE,
            -1,
            analogAxis,
            axisRangeMinValue,
            axisRangeMaxValue,
            relevantShifts,
            requiredShifts,
            buttonType,
            taskSupplier,
            affectedOperations,
            null);
    }

    /**
     * Initializes a new MacroOperationDescription based on a user interaction on an axis
     * @param operation the macro operation being described
     * @param userInputDevice which device will indicate the operation (driver or operator joystick)
     * @param analogAxis the analog axis used to perform the operation
     * @param axisRangeMinValue the min value of the range that triggers the operation
     * @param axisRangeMaxValue the max value of the range that triggers the operation
     * @param relevantShifts the shifts that should be considered when checking if we should perform the macro
     * @param requiredShifts the shift button(s) that must be applied to perform macro
     * @param buttonType the behavior type to use for the operation
     * @param taskSupplier the function that creates the tasks that should be performed by the macro
     * @param affectedOperations the list of operations that will be utilized by this macro
     * @param macroCancelOperations the list of operations that can be used to cancel this macro
     */
    public MacroOperationDescription(
        MacroOperation operation,
        UserInputDevice userInputDevice,
        AnalogAxis analogAxis,
        double axisRangeMinValue,
        double axisRangeMaxValue,
        Shift relevantShifts,
        Shift requiredShifts,
        ButtonType buttonType,
        Supplier<IControlTask> taskSupplier,
        IOperation[] affectedOperations,
        IOperation[] macroCancelOperations)
    {
        this(
            true,
            operation,
            userInputDevice,
            UserInputDeviceButton.ANALOG_AXIS_RANGE,
            -1,
            analogAxis,
            axisRangeMinValue,
            axisRangeMaxValue,
            relevantShifts,
            requiredShifts,
            buttonType,
            taskSupplier,
            affectedOperations,
            macroCancelOperations);
    }

    /**
     * Initializes a new MacroOperationDescription based on a user interaction on the POV
     * @param clearInterrupt whether to clear the interruption of the operations when the macro completes
     * @param operation the macro operation being described
     * @param userInputDevice which device will perform the macro operation (driver or operator joystick)
     * @param userInputDeviceButton the button on the device that performs the macro operation
     * @param povValue the value of the POV (hat) used to perform the macro operation
     * @param relevantShifts the shifts that should be considered when checking if we should perform the macro
     * @param requiredShifts the shift button(s) that must be applied to perform macro
     * @param buttonType the behavior type to use for the macro operation
     * @param taskSupplier the function that creates the tasks that should be performed by the macro
     * @param affectedOperations the list of operations that will be utilized by this macro
     * @param macroCancelOperations the list of operations that can be used to cancel this macro
     */
    private MacroOperationDescription(
        boolean clearInterrupt,
        MacroOperation operation,
        UserInputDevice userInputDevice,
        UserInputDeviceButton userInputDeviceButton,
        int povValue,
        AnalogAxis analogAxis,
        double axisRangeMinValue,
        double axisRangeMaxValue,
        Shift relevantShifts,
        Shift requiredShifts,
        ButtonType buttonType,
        Supplier<IControlTask> taskSupplier,
        IOperation[] affectedOperations,
        IOperation[] macroCancelOperations)
    {
        super(operation, OperationType.None, userInputDevice, axisRangeMinValue, axisRangeMaxValue, relevantShifts, requiredShifts);

        this.clearInterrupt = clearInterrupt;
        this.userInputDeviceButton = userInputDeviceButton;
        this.userInputDevicePovValue = povValue;
        this.userInputDeviceAxis = analogAxis;
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

    public AnalogAxis getUserInputDeviceAxis()
    {
        return this.userInputDeviceAxis;
    }

    public ButtonType getButtonType()
    {
        return this.buttonType;
    }

    public IControlTask constructTask()
    {
        return this.taskSupplier.get();
    }

    public IOperation[] getMacroCancelOperations()
    {
        if (this.macroCancelOperations != null)
        {
            return this.macroCancelOperations;
        }

        return this.affectedOperations;
    }

    public IOperation[] getAffectedOperations()
    {
        return this.affectedOperations;
    }
}
