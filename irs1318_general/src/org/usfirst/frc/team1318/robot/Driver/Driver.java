package org.usfirst.frc.team1318.robot.Driver;

import java.util.HashMap;
import java.util.Map;

import org.usfirst.frc.team1318.robot.Driver.Buttons.AnalogAxis;
import org.usfirst.frc.team1318.robot.Driver.Buttons.ButtonType;
import org.usfirst.frc.team1318.robot.Driver.ControlTasks.DriveTimedTask;
import org.usfirst.frc.team1318.robot.Driver.Descriptions.AnalogOperationDescription;
import org.usfirst.frc.team1318.robot.Driver.Descriptions.DigitalOperationDescription;
import org.usfirst.frc.team1318.robot.Driver.Descriptions.MacroOperationDescription;
import org.usfirst.frc.team1318.robot.Driver.Descriptions.OperationDescription;
import org.usfirst.frc.team1318.robot.Driver.Descriptions.UserInputDevice;
import org.usfirst.frc.team1318.robot.Driver.States.AnalogOperationState;
import org.usfirst.frc.team1318.robot.Driver.States.DigitalOperationState;
import org.usfirst.frc.team1318.robot.Driver.States.OperationState;

/**
 * Driver that represents something that operates the robot.  This is either autonomous or teleop/user driver.
 *
 */
public abstract class Driver
{
    @SuppressWarnings("serial")
    protected Map<Operation, OperationDescription> operationSchema = new HashMap<Operation, OperationDescription>()
    {
        {
            put(
                Operation.DriveTrainMoveForward,
                new AnalogOperationDescription(
                    UserInputDevice.Driver,
                    AnalogAxis.Y));
            put(
                Operation.DriveTrainTurn,
                new AnalogOperationDescription(
                    UserInputDevice.Driver,
                    AnalogAxis.X));
            put(
                Operation.DriveTrainShiftGearUp,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_STICK_TOP_LEFT_BUTTON,
                    ButtonType.Click));
            put(
                Operation.DriveTrainShiftGearDown,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_STICK_BOTTOM_LEFT_BUTTON,
                    ButtonType.Click));
            put(
                Operation.DriveTrainSimpleMode,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_STICK_THUMB_BUTTON,
                    ButtonType.Toggle));
            put(
                Operation.DriveTrainUsePositionalMode,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Toggle));
            put(
                Operation.DriveTrainLeftPosition,
                new AnalogOperationDescription(
                    UserInputDevice.None,
                    AnalogAxis.None));
            put(
                Operation.DriveTrainRightPosition,
                new AnalogOperationDescription(
                    UserInputDevice.None,
                    AnalogAxis.None));
        }
    };

    @SuppressWarnings("serial")
    protected Map<MacroOperation, MacroOperationDescription> macroSchema = new HashMap<MacroOperation, MacroOperationDescription>()
    {
        {
            put(
                MacroOperation.DriveDistance,
                new MacroOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_BASE_BOTTOM_RIGHT_BUTTON,
                    () -> ((IControlTask)new DriveTimedTask(20.0, 0.05, 0.05)),
                    new Operation[]
                        { Operation.DriveTrainMoveForward, Operation.DriveTrainTurn, Operation.DriveTrainUsePositionalMode }));
        }
    };

    protected final Map<Operation, OperationState> operationStateMap;

    /**
     * Initializes a new Driver
     */
    protected Driver()
    {
        this.operationStateMap = new HashMap<Operation, OperationState>(this.operationSchema.size());
        for (Operation operation : this.operationSchema.keySet())
        {
            this.operationStateMap.put(operation, OperationState.createFromDescription(this.operationSchema.get(operation)));
        }
    }

    /**
     * Tell the driver that some time has passed
     */
    public abstract void update();

    /**
     * Tell the driver that operation is stopping
     */
    public abstract void stop();

    /**
     * Get a boolean indicating whether the current digital operation is enabled
     * @param digitalOperation to get
     * @return the current value of the digital operation
     */
    public boolean getDigital(Operation digitalOperation)
    {
        OperationState state = this.operationStateMap.get(digitalOperation);
        if (!(state instanceof DigitalOperationState))
        {
            throw new RuntimeException("not a digital operation!");
        }

        DigitalOperationState digitalState = (DigitalOperationState)state;
        return digitalState.getState();
    }

    /**
     * Get a double between -1.0 and 1.0 indicating the current value of the analog operation
     * @param analogOperation to get
     * @return the current value of the analog operation
     */
    public double getAnalog(Operation analogOperation)
    {
        OperationState state = this.operationStateMap.get(analogOperation);
        if (!(state instanceof AnalogOperationState))
        {
            throw new RuntimeException("not an analog operation!");
        }

        AnalogOperationState analogState = (AnalogOperationState)state;
        return analogState.getState();
    }
}
