package org.usfirst.frc.team1318.robot.driver;

import java.util.HashMap;
import java.util.Map;

import javax.inject.Singleton;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;
import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.driver.common.IButtonMap;
import org.usfirst.frc.team1318.robot.driver.common.UserInputDeviceButton;
import org.usfirst.frc.team1318.robot.driver.common.buttons.AnalogAxis;
import org.usfirst.frc.team1318.robot.driver.common.buttons.ButtonType;
import org.usfirst.frc.team1318.robot.driver.common.descriptions.AnalogOperationDescription;
import org.usfirst.frc.team1318.robot.driver.common.descriptions.DigitalOperationDescription;
import org.usfirst.frc.team1318.robot.driver.common.descriptions.MacroOperationDescription;
import org.usfirst.frc.team1318.robot.driver.common.descriptions.OperationDescription;
import org.usfirst.frc.team1318.robot.driver.common.descriptions.UserInputDevice;
import org.usfirst.frc.team1318.robot.driver.controltasks.PIDBrakeTask;
import org.usfirst.frc.team1318.robot.driver.controltasks.VisionAdvanceAndCenterTask;
import org.usfirst.frc.team1318.robot.driver.controltasks.VisionCenteringTask;

@Singleton
public class ButtonMap implements IButtonMap
{
    @SuppressWarnings("serial")
    public static Map<Operation, OperationDescription> OperationSchema = new HashMap<Operation, OperationDescription>()
    {
        {
            // Operations for vision
            put(
                Operation.EnableVision,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.JOYSTICK_STICK_TOP_RIGHT_BUTTON,
                    ButtonType.Toggle));

            // Operations for the drive train
            put(
                Operation.DriveTrainDisablePID,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_11,
                    ButtonType.Click));
            put(
                Operation.DriveTrainEnablePID,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_12,
                    ButtonType.Click));
            put(
                Operation.DriveTrainMoveForward,
                new AnalogOperationDescription(
                    UserInputDevice.Driver,
                    AnalogAxis.Y,
                    ElectronicsConstants.INVERT_Y_AXIS,
                    TuningConstants.DRIVETRAIN_Y_DEAD_ZONE));
            put(
                Operation.DriveTrainTurn,
                new AnalogOperationDescription(
                    UserInputDevice.Driver,
                    AnalogAxis.X,
                    ElectronicsConstants.INVERT_X_AXIS,
                    TuningConstants.DRIVETRAIN_X_DEAD_ZONE));
            put(
                Operation.DriveTrainSimpleMode,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
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
                    AnalogAxis.None,
                    false,
                    0.0));
            put(
                Operation.DriveTrainRightPosition,
                new AnalogOperationDescription(
                    UserInputDevice.None,
                    AnalogAxis.None,
                    false,
                    0.0));
            put(
                Operation.DriveTrainSwapFrontOrientation,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Toggle));
        }
    };

    @SuppressWarnings("serial")
    public static Map<MacroOperation, MacroOperationDescription> MacroSchema = new HashMap<MacroOperation, MacroOperationDescription>()
    {
        {
            // Brake mode macro
            put(
                MacroOperation.PIDBrake,
                new MacroOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_STICK_THUMB_BUTTON,
                    ButtonType.Simple,
                    () -> new PIDBrakeTask(),
                    new Operation[]
                    {
                        Operation.DriveTrainUsePositionalMode,
                        Operation.DriveTrainLeftPosition,
                        Operation.DriveTrainRightPosition,
                    }));

            // Centering macro
            put(
                MacroOperation.VisionCenter,
                new MacroOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_BASE_MIDDLE_LEFT_BUTTON,
                    ButtonType.Toggle,
                    () -> new VisionCenteringTask(),
                    new Operation[]
                    {
                        Operation.EnableVision,
                        Operation.DriveTrainUsePositionalMode,
                        Operation.DriveTrainLeftPosition,
                        Operation.DriveTrainRightPosition,
                        Operation.DriveTrainTurn,
                        Operation.DriveTrainMoveForward,
                    }));
            put(
                MacroOperation.VisionCenterAndAdvance,
                new MacroOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_STICK_TOP_RIGHT_BUTTON,
                    ButtonType.Toggle,
                    () -> new VisionAdvanceAndCenterTask(),
                    new Operation[]
                    {
                        Operation.EnableVision,
                        Operation.DriveTrainUsePositionalMode,
                        Operation.DriveTrainLeftPosition,
                        Operation.DriveTrainRightPosition,
                        Operation.DriveTrainTurn,
                        Operation.DriveTrainMoveForward,
                    }));
        }
    };

    @Override
    public Map<Operation, OperationDescription> getOperationSchema()
    {
        return ButtonMap.OperationSchema;
    }

    @Override
    public Map<MacroOperation, MacroOperationDescription> getMacroOperationSchema()
    {
        return ButtonMap.MacroSchema;
    }
}
