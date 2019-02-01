package frc.robot.driver;

import java.util.HashMap;
import java.util.Map;

import javax.inject.Singleton;

import frc.robot.ElectronicsConstants;
import frc.robot.TuningConstants;
import frc.robot.common.robotprovider.AnalogAxis;
import frc.robot.driver.common.IButtonMap;
import frc.robot.driver.common.UserInputDeviceButton;
import frc.robot.driver.common.buttons.ButtonType;
import frc.robot.driver.common.descriptions.AnalogOperationDescription;
import frc.robot.driver.common.descriptions.DigitalOperationDescription;
import frc.robot.driver.common.descriptions.MacroOperationDescription;
import frc.robot.driver.common.descriptions.OperationDescription;
import frc.robot.driver.common.descriptions.ShiftDescription;
import frc.robot.driver.common.descriptions.UserInputDevice;
import frc.robot.driver.controltasks.*;

@Singleton
public class ButtonMap implements IButtonMap
{
    @SuppressWarnings("serial")
    private static Map<Shift, ShiftDescription> ShiftButtons = new HashMap<Shift, ShiftDescription>()
    {
        {
            put(
                Shift.Debug,
                new ShiftDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE));
        }
    };

    @SuppressWarnings("serial")
    public static Map<Operation, OperationDescription> OperationSchema = new HashMap<Operation, OperationDescription>()
    {
        {
            // Operations for vision
            put(
                Operation.EnableVision,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    90, // POV right
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
                Operation.DriveTrainUseBrakeMode,
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
                Operation.DriveTrainLeftVelocity,
                new AnalogOperationDescription(
                    UserInputDevice.None,
                    AnalogAxis.None,
                    false,
                    0.0));
            put(
                Operation.DriveTrainRightVelocity,
                new AnalogOperationDescription(
                    UserInputDevice.None,
                    AnalogAxis.None,
                    false,
                    0.0));
            put(
                Operation.DriveTrainHeading,
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
            put(
                Operation.DriveTrainUsePathMode,
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
                        Operation.DriveTrainUseBrakeMode,
                        Operation.DriveTrainLeftPosition,
                        Operation.DriveTrainRightPosition,
                    }));
            
            // turn in place macros
            put(
                MacroOperation.TurnInPlaceRight,
                new MacroOperationDescription(
                    UserInputDevice.Driver,
                    90,
                    Shift.Any,
                    ButtonType.Toggle,
                    () -> new NavxTurnTask(false, 180, true, true),
                    new Operation[]
                    {
                        Operation.DriveTrainUsePositionalMode,
                        Operation.DriveTrainUseBrakeMode,
                        Operation.DriveTrainLeftPosition,
                        Operation.DriveTrainRightPosition,
                        Operation.DriveTrainLeftVelocity,
                        Operation.DriveTrainRightVelocity,
                        Operation.DriveTrainTurn,
                        Operation.DriveTrainMoveForward,
                        Operation.DriveTrainSimpleMode,
                    },
                    new Operation[]
                    {
                        Operation.DriveTrainUsePositionalMode,
                        Operation.DriveTrainUseBrakeMode,
                        Operation.DriveTrainLeftPosition,
                        Operation.DriveTrainRightPosition,
                    }));
            put(
                MacroOperation.TurnInPlaceLeft,
                new MacroOperationDescription(
                    UserInputDevice.Driver,
                    270,
                    Shift.Any,
                    ButtonType.Toggle,
                    () -> new NavxTurnTask(false, -180, true, true),
                    new Operation[]
                    {
                        Operation.DriveTrainUsePositionalMode,
                        Operation.DriveTrainUseBrakeMode,
                        Operation.DriveTrainLeftPosition,
                        Operation.DriveTrainRightPosition,
                        Operation.DriveTrainTurn,
                        Operation.DriveTrainMoveForward,
                        Operation.DriveTrainSimpleMode,
                    },
                    new Operation[]
                    {
                        Operation.DriveTrainUsePositionalMode,
                        Operation.DriveTrainUseBrakeMode,
                        Operation.DriveTrainLeftPosition,
                        Operation.DriveTrainRightPosition,
                    }));
            put(
                MacroOperation.FollowSomePath,
                new MacroOperationDescription(
                    UserInputDevice.Driver,
                    0,
                    Shift.Any,
                    ButtonType.Toggle,
                    () -> new FollowPathTask(""),
                    new Operation[]
                    {
                        Operation.DriveTrainUsePositionalMode,
                        Operation.DriveTrainUseBrakeMode,
                        Operation.DriveTrainLeftPosition,
                        Operation.DriveTrainRightPosition,
                        Operation.DriveTrainLeftVelocity,
                        Operation.DriveTrainRightVelocity,
                        Operation.DriveTrainHeading,
                        Operation.DriveTrainUsePathMode,
                        Operation.DriveTrainTurn,
                        Operation.DriveTrainMoveForward,
                        Operation.DriveTrainSimpleMode,
                    },
                    new Operation[]
                    {
                        Operation.DriveTrainUsePositionalMode,
                        Operation.DriveTrainUseBrakeMode,
                        Operation.DriveTrainLeftPosition,
                        Operation.DriveTrainRightPosition,
                    }));

            // Centering macro
            put(
                MacroOperation.VisionCenter,
                new MacroOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
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
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
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
    public Map<Shift, ShiftDescription> getShiftMap()
    {
        return ButtonMap.ShiftButtons;
    }

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
