package frc.robot.driver;

import javax.inject.Singleton;

import frc.robot.*;
import frc.robot.driver.common.*;
import frc.robot.driver.common.buttons.*;
import frc.robot.driver.common.descriptions.*;
import frc.robot.driver.controltasks.*;

@Singleton
public class ButtonMap implements IButtonMap
{
    private static ShiftDescription[] ShiftButtonSchema = new ShiftDescription[]
    {
        new ShiftDescription(
            Shift.Debug,
            UserInputDevice.Driver,
            UserInputDeviceButton.JOYSTICK_STICK_TRIGGER_BUTTON),
/*        new ShiftDescription(
            Shift.ButtonPadDebug,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_16)),*/
    };

    public static AnalogOperationDescription[] AnalogOperationSchema = new AnalogOperationDescription[]
    {
        // Operations for the drive train
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainMoveForward,
            UserInputDevice.Driver,
            AnalogAxis.JOYSTICK_Y,
            ElectronicsConstants.INVERT_Y_AXIS,
            TuningConstants.DRIVETRAIN_Y_DEAD_ZONE),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainTurn,
            UserInputDevice.Driver,
            AnalogAxis.JOYSTICK_X,
            ElectronicsConstants.INVERT_X_AXIS,
            TuningConstants.DRIVETRAIN_X_DEAD_ZONE),
    };

    public static DigitalOperationDescription[] DigitalOperationSchema = new DigitalOperationDescription[]
    {
    };

    public static MacroOperationDescription[] MacroSchema = new MacroOperationDescription[]
    {
        // Brake mode macro
        new MacroOperationDescription(
            MacroOperation.PIDBrake,
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
            }),

        // Driving Macros
        new MacroOperationDescription(
            MacroOperation.TurnInPlaceRight,
            UserInputDevice.Driver,
            90,
            Shift.Debug,
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
            }),
        new MacroOperationDescription(
            MacroOperation.TurnInPlaceLeft,
            UserInputDevice.Driver,
            270,
            Shift.Debug,
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
            }),
        new MacroOperationDescription(
            MacroOperation.FollowSomePath,
            UserInputDevice.None,
            UserInputDeviceButton.NONE,
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
            }),

        // Vision Macros
        new MacroOperationDescription(
            MacroOperation.VisionCenterAndAdvance,
            UserInputDevice.Driver,
            180,
            Shift.Debug,
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
            }),
/*      new MacroOperationDescription(
            MacroOperation.VisionFastCenterAndAdvance,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_7,
            Shift.Debug,
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
            }),*/
    };

    @Override
    public ShiftDescription[] getShiftSchema()
    {
        return ButtonMap.ShiftButtonSchema;
    }

    @Override
    public AnalogOperationDescription[] getAnalogOperationSchema()
    {
        return ButtonMap.AnalogOperationSchema;
    }

    @Override
    public DigitalOperationDescription[] getDigitalOperationSchema()
    {
        return ButtonMap.DigitalOperationSchema;
    }

    @Override
    public MacroOperationDescription[] getMacroOperationSchema()
    {
        return ButtonMap.MacroSchema;
    }
}
