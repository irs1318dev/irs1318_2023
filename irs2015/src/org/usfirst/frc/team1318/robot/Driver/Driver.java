package org.usfirst.frc.team1318.robot.Driver;

import java.util.HashMap;
import java.util.Map;

import org.usfirst.frc.team1318.robot.Driver.Buttons.AnalogAxis;
import org.usfirst.frc.team1318.robot.Driver.Buttons.ButtonType;
import org.usfirst.frc.team1318.robot.Driver.Descriptions.AnalogOperationDescription;
import org.usfirst.frc.team1318.robot.Driver.Descriptions.DigitalOperationDescription;
import org.usfirst.frc.team1318.robot.Driver.Descriptions.MacroOperationDescription;
import org.usfirst.frc.team1318.robot.Driver.Descriptions.OperationDescription;
import org.usfirst.frc.team1318.robot.Driver.Descriptions.UserInputDevice;

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
                    JoystickButtonConstants.JOYSTICK_STICK_TOP_LEFT_BUTTON,
                    ButtonType.Click));
            put(
                Operation.DriveTrainShiftGearDown,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    JoystickButtonConstants.JOYSTICK_STICK_BOTTOM_LEFT_BUTTON,
                    ButtonType.Click));
            put(
                Operation.DriveTrainSimpleMode,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    JoystickButtonConstants.JOYSTICK_STICK_THUMB_BUTTON,
                    ButtonType.Toggle));
            put(
                Operation.DriveTrainUsePositionalMode,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    JoystickButtonConstants.NONE,
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
            put(
                Operation.Foo,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    JoystickButtonConstants.JOYSTICK_STICK_TRIGGER_BUTTON,
                    ButtonType.Simple));
            put(
                Operation.Bar,
                new AnalogOperationDescription(
                    UserInputDevice.CoDriver,
                    AnalogAxis.Throttle));
            put(
                Operation.Qux,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    JoystickButtonConstants.JOYSTICK_BASE_BOTTOM_LEFT_BUTTON,
                    ButtonType.Toggle));
        }
    };

    @SuppressWarnings("serial")
    protected Map<MacroOperation, MacroOperationDescription> macroSchema = new HashMap<MacroOperation, MacroOperationDescription>()
    {
        {
            //            put(
            //                MacroOperation.FooBarQux,
            //                new MacroOperationDescription(
            //                    UserInputDevice.Driver,
            //                    JoystickButtonConstants.JOYSTICK_BASE_BOTTOM_RIGHT_BUTTON,
            //                    (Supplier),
            //                    new Operation[] { Operation.Foo, Operation.Bar, Operation.Qux });
        }
    };

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
    public abstract boolean getDigital(Operation digitalOperation);

    /**
     * Get a double between -1.0 and 1.0 indicating the current value of the analog operation
     * @param analogOperation to get
     * @return the current value of the analog operation
     */
    public abstract double getAnalog(Operation analogOperation);
}
