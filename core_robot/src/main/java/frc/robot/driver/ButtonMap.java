package frc.robot.driver;

import java.util.HashMap;
import java.util.Map;

import javax.inject.Singleton;

import frc.robot.*;
import frc.robot.driver.common.*;
import frc.robot.driver.common.buttons.*;
import frc.robot.driver.common.descriptions.*;
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
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_STICK_TRIGGER_BUTTON));
            put(
                Shift.ButtonPadDebug,
                new ShiftDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_16));
        }
    };

    @SuppressWarnings("serial")
    public static Map<AnalogOperation, AnalogOperationDescription> AnalogOperationSchema = new HashMap<AnalogOperation, AnalogOperationDescription>()
    {
        {
            // Operations for the drive train
            put(
                AnalogOperation.DriveTrainMoveForward,
                new AnalogOperationDescription(
                    UserInputDevice.Driver,
                    AnalogAxis.JOYSTICK_Y,
                    ElectronicsConstants.INVERT_Y_AXIS,
                    TuningConstants.DRIVETRAIN_Y_DEAD_ZONE));
            put(
                AnalogOperation.DriveTrainTurn,
                new AnalogOperationDescription(
                    UserInputDevice.Driver,
                    AnalogAxis.JOYSTICK_X,
                    ElectronicsConstants.INVERT_X_AXIS,
                    TuningConstants.DRIVETRAIN_X_DEAD_ZONE));
        }
    };

    @SuppressWarnings("serial")
    public static Map<DigitalOperation, DigitalOperationDescription> DigitalOperationSchema = new HashMap<DigitalOperation, DigitalOperationDescription>()
    {
        {
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
                    new IOperation[]
                    {
                        DigitalOperation.DriveTrainUsePositionalMode,
                        DigitalOperation.DriveTrainUseBrakeMode,
                        AnalogOperation.DriveTrainLeftPosition,
                        AnalogOperation.DriveTrainRightPosition,
                    }));

            // Driving Macros
            put(
                MacroOperation.TurnInPlaceRight,
                new MacroOperationDescription(
                    UserInputDevice.Driver,
                    90,
                    Shift.Debug,
                    ButtonType.Toggle,
                    () -> new NavxTurnTask(true, 180, 3.0, true, false),
                    new IOperation[]
                    {
                        DigitalOperation.DriveTrainUsePositionalMode,
                        DigitalOperation.DriveTrainUseBrakeMode,
                        AnalogOperation.DriveTrainLeftPosition,
                        AnalogOperation.DriveTrainRightPosition,
                        AnalogOperation.DriveTrainLeftVelocity,
                        AnalogOperation.DriveTrainRightVelocity,
                        AnalogOperation.DriveTrainTurn,
                        AnalogOperation.DriveTrainMoveForward,
                        DigitalOperation.DriveTrainSimpleMode,
                    },
                    new IOperation[]
                    {
                        DigitalOperation.DriveTrainUsePositionalMode,
                        DigitalOperation.DriveTrainUseBrakeMode,
                        AnalogOperation.DriveTrainLeftPosition,
                        AnalogOperation.DriveTrainRightPosition,
                    }));
            put(
                MacroOperation.TurnInPlaceLeft,
                new MacroOperationDescription(
                    UserInputDevice.Driver,
                    270,
                    Shift.Debug,
                    ButtonType.Toggle,
                    () -> new NavxTurnTask(true, -180, TuningConstants.NAVX_FAST_TURN_TIMEOUT, true, true),
                    new IOperation[]
                    {
                        DigitalOperation.DriveTrainUsePositionalMode,
                        DigitalOperation.DriveTrainUseBrakeMode,
                        AnalogOperation.DriveTrainLeftPosition,
                        AnalogOperation.DriveTrainRightPosition,
                        AnalogOperation.DriveTrainTurn,
                        AnalogOperation.DriveTrainMoveForward,
                        DigitalOperation.DriveTrainSimpleMode,
                    },
                    new IOperation[]
                    {
                        DigitalOperation.DriveTrainUsePositionalMode,
                        DigitalOperation.DriveTrainUseBrakeMode,
                        AnalogOperation.DriveTrainLeftPosition,
                        AnalogOperation.DriveTrainRightPosition,
                    }));
            put(
                MacroOperation.FollowSomePath,
                new MacroOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    Shift.Any,
                    ButtonType.Toggle,
                    () -> new FollowPathTask("/Paths/Circle 40 inch radius.csv"),
                    new IOperation[]
                    {
                        DigitalOperation.DriveTrainUsePositionalMode,
                        DigitalOperation.DriveTrainUseBrakeMode,
                        AnalogOperation.DriveTrainLeftPosition,
                        AnalogOperation.DriveTrainRightPosition,
                        AnalogOperation.DriveTrainLeftVelocity,
                        AnalogOperation.DriveTrainRightVelocity,
                        AnalogOperation.DriveTrainHeadingCorrection,
                        DigitalOperation.DriveTrainUsePathMode,
                        AnalogOperation.DriveTrainTurn,
                        AnalogOperation.DriveTrainMoveForward,
                        DigitalOperation.DriveTrainSimpleMode,
                    },
                    new IOperation[]
                    {
                        DigitalOperation.DriveTrainUsePositionalMode,
                        DigitalOperation.DriveTrainUseBrakeMode,
                        AnalogOperation.DriveTrainLeftPosition,
                        AnalogOperation.DriveTrainRightPosition,
                    }));

            // Vision Macros
            put(
                MacroOperation.VisionCenterAndAdvance,
                new MacroOperationDescription(
                    UserInputDevice.Driver,
                    180,
                    Shift.Debug,
                    ButtonType.Toggle,
                    () -> new VisionAdvanceAndCenterTask(DigitalOperation.VisionEnable),
                    new IOperation[]
                    {
                        DigitalOperation.VisionDisable,
                        DigitalOperation.VisionEnable,
                        DigitalOperation.DriveTrainUsePositionalMode,
                        AnalogOperation.DriveTrainLeftPosition,
                        AnalogOperation.DriveTrainRightPosition,
                        AnalogOperation.DriveTrainTurn,
                        AnalogOperation.DriveTrainMoveForward
                    }));
            put(
                MacroOperation.VisionFastCenterAndAdvance,
                new MacroOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_7,
                    Shift.None,
                    ButtonType.Toggle,
                    () -> new VisionFastAdvanceAndCenterTask(DigitalOperation.VisionEnable),
                    new IOperation[]
                    {
                        DigitalOperation.VisionDisable,
                        DigitalOperation.VisionEnable,
                        DigitalOperation.DriveTrainUsePositionalMode,
                        AnalogOperation.DriveTrainLeftPosition,
                        AnalogOperation.DriveTrainRightPosition,
                        AnalogOperation.DriveTrainTurn,
                        AnalogOperation.DriveTrainMoveForward
                    }));
        }
    };

    @Override
    public Map<Shift, ShiftDescription> getShiftMap()
    {
        return ButtonMap.ShiftButtons;
    }

    @Override
    public Map<AnalogOperation, AnalogOperationDescription> getAnalogOperationSchema()
    {
        return ButtonMap.AnalogOperationSchema;
    }

    @Override
    public Map<DigitalOperation, DigitalOperationDescription> getDigitalOperationSchema()
    {
        return ButtonMap.DigitalOperationSchema;
    }

    @Override
    public Map<MacroOperation, MacroOperationDescription> getMacroOperationSchema()
    {
        return ButtonMap.MacroSchema;
    }
}
