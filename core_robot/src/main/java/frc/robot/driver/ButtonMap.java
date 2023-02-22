package frc.robot.driver;

import javax.inject.Singleton;

import frc.robot.*;
import frc.robot.common.Helpers;
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
            Shift.DriverDebug,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_SELECT_BUTTON),
        new ShiftDescription(
            Shift.CodriverDebug,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_LEFT_BUTTON),
        new ShiftDescription(
            Shift.Test1Debug,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_LEFT_BUTTON),
        // new ShiftDescription(
        //     Shift.Test2Debug,
        //     UserInputDevice.Test2,
        //     UserInputDeviceButton.XBONE_LEFT_BUTTON),
    };

    public static AnalogOperationDescription[] AnalogOperationSchema = new AnalogOperationDescription[]
    {
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainMoveForward,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_LSY,
            ElectronicsConstants.INVERT_XBONE_LEFT_Y_AXIS,
            -TuningConstants.DRIVETRAIN_DEAD_ZONE_VELOCITY_Y,
            TuningConstants.DRIVETRAIN_DEAD_ZONE_VELOCITY_Y),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainMoveRight,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_LSX,
            ElectronicsConstants.INVERT_XBONE_LEFT_X_AXIS,
            -TuningConstants.DRIVETRAIN_DEAD_ZONE_VELOCITY_X,
            TuningConstants.DRIVETRAIN_DEAD_ZONE_VELOCITY_X),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainTurnAngleGoal,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_RSX,
            AnalogAxis.XBONE_RSY,
            Shift.None, // Shift.DriverDebug,
            Shift.None,
            !ElectronicsConstants.INVERT_XBONE_RIGHT_X_AXIS, // make left positive...
            ElectronicsConstants.INVERT_XBONE_RIGHT_Y_AXIS,
            0.0,
            TuningConstants.DRIVETRAIN_SKIP_OMEGA_ON_ZERO_DELTA,
            true,
            1.0,
            TuningConstants.MAGIC_NULL_VALUE,
            (x, y) -> Helpers.atan2d(x, y)),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainSpinLeft,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_LT,
            ElectronicsConstants.INVERT_XBONE_LEFT_TRIGGER, // turning left should be positive, as counter-clockwise is positive
            -TuningConstants.DRIVETRAIN_DEAD_ZONE_TURN,
            TuningConstants.DRIVETRAIN_DEAD_ZONE_TURN),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainSpinRight,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_RT,
            !ElectronicsConstants.INVERT_XBONE_RIGHT_TRIGGER, // make left positive, as counter-clockwise is positive
            -TuningConstants.DRIVETRAIN_DEAD_ZONE_TURN,
            TuningConstants.DRIVETRAIN_DEAD_ZONE_TURN),

        new AnalogOperationDescription(
            AnalogOperation.ArmIKZAdjustment,
            UserInputDevice.Codriver,
            AnalogAxis.XBONE_LSY,
            Shift.CodriverDebug,
            Shift.None,
            ElectronicsConstants.INVERT_XBONE_LEFT_Y_AXIS,
            -TuningConstants.ARM_LOWER_VELOCITY_DEAZONE,
            TuningConstants.ARM_LOWER_VELOCITY_DEAZONE,
            1.0),
        new AnalogOperationDescription(
            AnalogOperation.ArmIKXAdjustment,
            UserInputDevice.Codriver,
            AnalogAxis.XBONE_RSX,
            Shift.CodriverDebug,
            Shift.None,
            ElectronicsConstants.INVERT_XBONE_RIGHT_X_AXIS,
            -TuningConstants.ARM_UPPER_VELOCITY_DEAZONE,
            TuningConstants.ARM_UPPER_VELOCITY_DEAZONE,
            1.0),

        new AnalogOperationDescription(
            AnalogOperation.ArmLowerPositionAdjustment,
            UserInputDevice.Codriver,
            AnalogAxis.XBONE_LSY,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ElectronicsConstants.INVERT_XBONE_LEFT_Y_AXIS,
            -TuningConstants.ARM_LOWER_VELOCITY_DEAZONE,
            TuningConstants.ARM_LOWER_VELOCITY_DEAZONE,
            1.0),
        new AnalogOperationDescription(
            AnalogOperation.ArmUpperPositionAdjustment,
            UserInputDevice.Codriver,
            AnalogAxis.XBONE_RSY,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ElectronicsConstants.INVERT_XBONE_RIGHT_Y_AXIS,
            -TuningConstants.ARM_UPPER_VELOCITY_DEAZONE,
            TuningConstants.ARM_UPPER_VELOCITY_DEAZONE,
            1.0),

        new AnalogOperationDescription(
            AnalogOperation.ArmMMLowerPosition,
            TuningConstants.MAGIC_NULL_VALUE),
        new AnalogOperationDescription(
            AnalogOperation.ArmMMUpperPosition,
            TuningConstants.MAGIC_NULL_VALUE),
        new AnalogOperationDescription(
            AnalogOperation.ArmIKXPosition,
            TuningConstants.MAGIC_NULL_VALUE),
        new AnalogOperationDescription(
            AnalogOperation.ArmIKZPosition,
            TuningConstants.MAGIC_NULL_VALUE),
    };

    public static DigitalOperationDescription[] DigitalOperationSchema = new DigitalOperationDescription[]
    {
        // driving operations
        new DigitalOperationDescription(
            DigitalOperation.PositionResetFieldOrientation,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_Y_BUTTON,
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainReset,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_Y_BUTTON,
            Shift.DriverDebug,
            Shift.DriverDebug,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainEnableFieldOrientation,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_X_BUTTON,
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainDisableFieldOrientation,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_X_BUTTON,
            Shift.DriverDebug,
            Shift.DriverDebug,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainEnableMaintainDirectionMode,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_B_BUTTON,
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainDisableMaintainDirectionMode,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_B_BUTTON,
            Shift.DriverDebug,
            Shift.DriverDebug,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainSlowMode,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_A_BUTTON,
            ButtonType.Simple),

        new DigitalOperationDescription(
            DigitalOperation.ArmDisableSimpleMode,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_RIGHT_STICK_BUTTON,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.ArmEnableSimpleMode,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_RIGHT_STICK_BUTTON,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ButtonType.Click),

        new DigitalOperationDescription(
            DigitalOperation.IntakeGrab,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_LEFT_BUTTON,
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.IntakeRelease,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_LEFT_BUTTON,
            Shift.DriverDebug,
            Shift.DriverDebug,
            ButtonType.Click),
        // new DigitalOperationDescription(
        //     DigitalOperation.IntakeIn,
        //     UserInputDevice.Driver,
        //     UserInputDeviceButton.XBONE_RIGHT_BUTTON,
        //     Shift.DriverDebug,
        //     Shift.None,
        //     ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.IntakeOut,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_RIGHT_BUTTON,
            Shift.DriverDebug,
            Shift.DriverDebug,
            ButtonType.Simple),

        new DigitalOperationDescription(
            DigitalOperation.ArmForceReset,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_RIGHT_BUTTON,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ButtonType.Click),

        // Test operations:
        new DigitalOperationDescription(
            DigitalOperation.VisionEnableAprilTagProcessing,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_A_BUTTON,
            ButtonType.Toggle),
    };

    public static MacroOperationDescription[] MacroSchema = new MacroOperationDescription[]
    {
        // driving macros
        new MacroOperationDescription(
            MacroOperation.PIDLightBrake,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_LEFT_STICK_BUTTON,
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Simple,
            () -> new PIDBrakeTask(false),
            new IOperation[]
            {
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainTurnSpeed,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
            }),
        new MacroOperationDescription(
            MacroOperation.PIDHeavyBrake,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_LEFT_STICK_BUTTON,
            Shift.DriverDebug,
            Shift.DriverDebug,
            ButtonType.Simple,
            () -> new PIDBrakeTask(true),
            new IOperation[]
            {
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainTurnSpeed,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
            }),
        new MacroOperationDescription(
            MacroOperation.VisionCenterRetroReflective,
            UserInputDevice.Driver,
            0, // DPAD-up
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> new VisionTurningTask(VisionTurningTask.TurnType.RetroreflectiveCentering, true),
            new IOperation[]
            {
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainTurnSpeed,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionDisableStream,
                DigitalOperation.VisionEnableAprilTagProcessing,
                DigitalOperation.VisionEnableRetroreflectiveProcessing,
                DigitalOperation.VisionForceDisable,
            }),
        new MacroOperationDescription(
            MacroOperation.VisionCenterAprilTag,
            UserInputDevice.Driver,
            90, // DPAD-right
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> new VisionTurningTask(VisionTurningTask.TurnType.AprilTagCentering, true),
            new IOperation[]
            {
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainTurnSpeed,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionDisableStream,
                DigitalOperation.VisionEnableAprilTagProcessing,
                DigitalOperation.VisionEnableRetroreflectiveProcessing,
                DigitalOperation.VisionForceDisable,
            }),
        new MacroOperationDescription(
            MacroOperation.VisionParallelAprilTag,
            UserInputDevice.Driver,
            180, // DPAD-down
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> new VisionTurningTask(VisionTurningTask.TurnType.AprilTagParallelizing, true),
            new IOperation[]
            {
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainTurnSpeed,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionDisableStream,
                DigitalOperation.VisionEnableAprilTagProcessing,
                DigitalOperation.VisionEnableRetroreflectiveProcessing,
                DigitalOperation.VisionForceDisable,
            }),
        new MacroOperationDescription(
            MacroOperation.VisionResetPosition,
            UserInputDevice.Driver,
            270, //DPAD left
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> new VisionResetPositionTask(),
            new IOperation[]
            {
                AnalogOperation.DriveTrainStartingXPosition,
                AnalogOperation.DriveTrainStartingYPosition,
                DigitalOperation.DriveTrainResetXYPosition,
                DigitalOperation.VisionEnableRetroreflectiveProcessing,
                DigitalOperation.VisionEnableAprilTagProcessing,
            }),

        // Intake macros
        new MacroOperationDescription(
            MacroOperation.IntakeGamePiece,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_RIGHT_BUTTON,
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Simple,
            () -> new IntakeGamePieceTask(),
            new IOperation[]
            {
                DigitalOperation.IntakeIn,
                DigitalOperation.IntakeOut,
                DigitalOperation.IntakeRelease,
                DigitalOperation.IntakeGrab,
            }),

        // Cone Flipper macros
        new MacroOperationDescription(
            MacroOperation.ExtendLeftConeFlipper,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_SELECT_BUTTON,
            Shift.None,
            Shift.None,
            ButtonType.Simple,
            () -> SequentialTask.Sequence(
                new ArmMMPositionTask(
                    TuningConstants.ARM_LOWER_POSITION_STOWED,
                    TuningConstants.ARM_UPPER_POSITION_STOWED,
                    true),
                new ConeFlipperExtendTask(true)),
            new IOperation[]
            {
                AnalogOperation.ArmMMUpperPosition,
                AnalogOperation.ArmMMLowerPosition,
                DigitalOperation.ExtendLeftConeFlipper,
                DigitalOperation.ExtendRightConeFlipper,
            }),
        new MacroOperationDescription(
            MacroOperation.ExtendRightConeFlipper,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_START_BUTTON,
            Shift.None,
            Shift.None,
            ButtonType.Simple,
            () -> SequentialTask.Sequence(
                new ArmMMPositionTask(
                    TuningConstants.ARM_LOWER_POSITION_STOWED,
                    TuningConstants.ARM_UPPER_POSITION_STOWED,
                    true),
                new ConeFlipperExtendTask(false)),
            new IOperation[]
            {
                AnalogOperation.ArmMMUpperPosition,
                AnalogOperation.ArmMMLowerPosition,
                DigitalOperation.ExtendLeftConeFlipper,
                DigitalOperation.ExtendRightConeFlipper,
            }),

        // Arm position macros
        new MacroOperationDescription(
            MacroOperation.ArmResetToZero,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_RIGHT_BUTTON,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> new ArmZeroTask(),
            new IOperation[]
            {
                AnalogOperation.ArmMMUpperPosition,
                AnalogOperation.ArmMMLowerPosition,
                DigitalOperation.ArmForceReset,
            }),
        new MacroOperationDescription(
            MacroOperation.ArmGroundPickupPosition,
            UserInputDevice.Codriver,
            270, // POV-left
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> new ArmMMPositionTask(
                TuningConstants.ARM_LOWER_POSITION_GROUND_PICKUP,
                TuningConstants.ARM_UPPER_POSITION_GROUND_PICKUP),
            new IOperation[]
            {
                AnalogOperation.ArmMMLowerPosition,
                AnalogOperation.ArmMMUpperPosition,
            }),
        new MacroOperationDescription(
            MacroOperation.ArmGroundPlacePosition,
            UserInputDevice.Codriver,
            180, // POV-down
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> new ArmMMPositionTask(
                TuningConstants.ARM_LOWER_POSITION_GROUND_PLACING,
                TuningConstants.ARM_UPPER_POSITION_GROUND_PLACING),
            new IOperation[]
            {
                AnalogOperation.ArmMMLowerPosition,
                AnalogOperation.ArmMMUpperPosition,
            }),
        new MacroOperationDescription(
            MacroOperation.ArmMiddleConePosition,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_B_BUTTON,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> new ArmMMPositionTask(
                TuningConstants.ARM_LOWER_POSITION_MIDDLE_CONE,
                TuningConstants.ARM_UPPER_POSITION_MIDDLE_CONE),
            new IOperation[]
            {
                AnalogOperation.ArmMMLowerPosition,
                AnalogOperation.ArmMMUpperPosition,
            }),
        new MacroOperationDescription(
            MacroOperation.ArmMiddleCubePosition,
            UserInputDevice.Codriver,
            90, // POV-right
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> new ArmMMPositionTask(
                TuningConstants.ARM_LOWER_POSITION_MIDDLE_CUBE,
                TuningConstants.ARM_UPPER_POSITION_MIDDLE_CUBE),
            new IOperation[]
            {
                AnalogOperation.ArmMMLowerPosition,
                AnalogOperation.ArmMMUpperPosition,
            }),
        new MacroOperationDescription(
            MacroOperation.ArmHighConePosition,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_Y_BUTTON,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> new ArmMMPositionTask(
                TuningConstants.ARM_LOWER_POSITION_HIGH_CONE,
                TuningConstants.ARM_UPPER_POSITION_HIGH_CONE),
            new IOperation[]
            {
                AnalogOperation.ArmMMLowerPosition,
                AnalogOperation.ArmMMUpperPosition,
            }),
        new MacroOperationDescription(
            MacroOperation.ArmHighCubePosition,
            UserInputDevice.Codriver,
            0, // POV-up
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> new ArmMMPositionTask(
                TuningConstants.ARM_LOWER_POSITION_HIGH_CUBE,
                TuningConstants.ARM_UPPER_POSITION_HIGH_CUBE),
            new IOperation[]
            {
                AnalogOperation.ArmMMLowerPosition,
                AnalogOperation.ArmMMUpperPosition,
            }),
        new MacroOperationDescription(
            MacroOperation.ArmSubstationPickupPosition,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_X_BUTTON,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> new ArmMMPositionTask(
                TuningConstants.ARM_LOWER_POSITION_SUB_PICKUP,
                TuningConstants.ARM_UPPER_POSITION_SUB_PICKUP),
            new IOperation[]
            {
                AnalogOperation.ArmMMLowerPosition,
                AnalogOperation.ArmMMUpperPosition,
            }),
        new MacroOperationDescription(
            MacroOperation.ArmStowedPosition,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_A_BUTTON,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> new ArmMMPositionTask(
                TuningConstants.ARM_LOWER_POSITION_STOWED,
                TuningConstants.ARM_UPPER_POSITION_STOWED),
            new IOperation[]
            {
                AnalogOperation.ArmMMLowerPosition,
                AnalogOperation.ArmMMUpperPosition,
            }),
        new MacroOperationDescription(
            MacroOperation.ArmApproachPosition,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_A_BUTTON,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ButtonType.Toggle,
            () -> new ArmMMPositionTask(
                TuningConstants.ARM_LOWER_POSITION_APPROACH,
                TuningConstants.ARM_UPPER_POSITION_APPROACH),
            new IOperation[]
            {
                AnalogOperation.ArmMMLowerPosition,
                AnalogOperation.ArmMMUpperPosition,
            }),

        new MacroOperationDescription(
            MacroOperation.ChargeStationBalance,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_SELECT_BUTTON, // Left menu button
            Shift.Test1Debug,
            Shift.None,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new PitchResetTask(), //calibration
                new ChargeStationTask(false), //false means charge station in front of robot
                ConcurrentTask.AllTasks(
                    new PIDBrakeTask(),
                    new WaitTask(0.5))),
            new IOperation[]
            {
                DigitalOperation.PositionResetRobotPitch,
                DigitalOperation.PositionResetFieldOrientation,
                AnalogOperation.PositionStartingAngle,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainTurnSpeed,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.DriveTrainEnableMaintainDirectionMode
            }),
//            new MacroOperationDescription(
//                MacroOperation.ChargeStationBalanceGyro,
//                UserInputDevice.Test1,
//                UserInputDeviceButton.XBONE_START_BUTTON, // Left menu button
//                Shift.Test1Debug,
//                Shift.None,
//                ButtonType.Toggle,
//                () -> SequentialTask.Sequence(
//                    new PitchResetTask(), //calibration
//                    new ChargeStationTaskGyro2(false), // false means charge station in front of robot
//                    ConcurrentTask.AllTasks(
//                        new PIDBrakeTask(),
//                        new WaitTask(0.5))),
//                new IOperation[]
//                {
//                    DigitalOperation.PositionResetRobotPitch,
//                    DigitalOperation.PositionResetFieldOrientation,
//                    AnalogOperation.PositionStartingAngle,
//                    AnalogOperation.DriveTrainMoveForward,
//                    AnalogOperation.DriveTrainMoveRight,
//                    AnalogOperation.DriveTrainTurnAngleGoal,
//                    AnalogOperation.DriveTrainTurnSpeed,
//                    AnalogOperation.DriveTrainRotationA,
//                    AnalogOperation.DriveTrainRotationB,
//                    AnalogOperation.DriveTrainPathXGoal,
//                    AnalogOperation.DriveTrainPathYGoal,
//                    AnalogOperation.DriveTrainPathXVelocityGoal,
//                    AnalogOperation.DriveTrainPathYVelocityGoal,
//                    AnalogOperation.DriveTrainPathAngleVelocityGoal,
//                    AnalogOperation.DriveTrainPositionDrive1,
//                    AnalogOperation.DriveTrainPositionDrive2,
//                    AnalogOperation.DriveTrainPositionDrive3,
//                    AnalogOperation.DriveTrainPositionDrive4,
//                    AnalogOperation.DriveTrainPositionSteer1,
//                    AnalogOperation.DriveTrainPositionSteer2,
//                    AnalogOperation.DriveTrainPositionSteer3,
//                    AnalogOperation.DriveTrainPositionSteer4,
//                    DigitalOperation.DriveTrainSteerMode,
//                    DigitalOperation.DriveTrainMaintainPositionMode,
//                    DigitalOperation.DriveTrainPathMode,
//                    DigitalOperation.DriveTrainReset,
//                    DigitalOperation.DriveTrainEnableFieldOrientation,
//                    DigitalOperation.DriveTrainDisableFieldOrientation,
//                    DigitalOperation.DriveTrainUseRobotOrientation,
//                    DigitalOperation.DriveTrainEnableMaintainDirectionMode
//                }),


        new MacroOperationDescription(
            MacroOperation.ChargeStationBalanceReverse,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_SELECT_BUTTON, // Left menu button
            Shift.Test1Debug,
            Shift.Test1Debug,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                // new PitchResetTask(), //calibration
                new ChargeStationTask(true), // true means charge station behind
                ConcurrentTask.AllTasks(
                    new PIDBrakeTask(),
                    new WaitTask(0.5))),
            new IOperation[]
            {
                DigitalOperation.PositionResetFieldOrientation,
                AnalogOperation.PositionStartingAngle,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainTurnSpeed,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.DriveTrainEnableMaintainDirectionMode
            }),

        new MacroOperationDescription(
            MacroOperation.FollowPathTest1,
            UserInputDevice.Test1,
            0,
            Shift.Test1Debug,
            Shift.None,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new PositionStartingTask(0.0, 0.0, 0.0),
                new FollowPathTask("goForward6ft2AndRotate", false, false)),
            new IOperation[]
            {
                DigitalOperation.PositionResetFieldOrientation,
                DigitalOperation.PositionResetRobotPitch,
                AnalogOperation.PositionStartingAngle,
                DigitalOperation.DriveTrainResetXYPosition,
                AnalogOperation.DriveTrainStartingXPosition,
                AnalogOperation.DriveTrainStartingYPosition,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainTurnSpeed,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                AnalogOperation.DriveTrainTurnAngleReference,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionDisableStream,
                DigitalOperation.VisionEnableAprilTagProcessing,
                DigitalOperation.VisionEnableRetroreflectiveProcessing,
                DigitalOperation.VisionForceDisable,
            }),
        new MacroOperationDescription(
            MacroOperation.FollowPath2,
            UserInputDevice.Test1,
            180,
            Shift.None,
            Shift.None,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                    new PositionStartingTask(-251.861, TuningConstants.StartOneGridY + 1.05, 180),
                    new FollowPathTask("LoadEdgeto1", false, false),
                    new FollowPathTask("1to14", false, false),
                    new WaitTask(2),
                    new FollowPathTask("14to10", false, false),
                    new FollowPathTask("10to2", false, false)
                    ),
            new IOperation[]
            {
                DigitalOperation.PositionResetFieldOrientation,
                DigitalOperation.PositionResetRobotPitch,
                AnalogOperation.PositionStartingAngle,
                DigitalOperation.DriveTrainResetXYPosition,
                AnalogOperation.DriveTrainStartingXPosition,
                AnalogOperation.DriveTrainStartingYPosition,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainTurnSpeed,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                AnalogOperation.DriveTrainTurnAngleReference,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionDisableStream,
                DigitalOperation.VisionEnableAprilTagProcessing,
                DigitalOperation.VisionEnableRetroreflectiveProcessing,
                DigitalOperation.VisionForceDisable,
            }),

    new MacroOperationDescription(
        MacroOperation.FollowPath2,
        UserInputDevice.Test1,
        270,
        Shift.None,
        Shift.None,
        ButtonType.Toggle,
        () -> SequentialTask.Sequence(
                new PositionStartingTask(-TuningConstants.StartGridX, 16.8, 180),
                new FollowPathTask("GuardEdgeto9", false, false),
                new FollowPathTask("9to17", false, false),
                new WaitTask(2),
                new FollowPathTask("17to12", false, false),
                new FollowPathTask("12to8", false, false)
                ),
        new IOperation[]
        {
            DigitalOperation.PositionResetFieldOrientation,
            DigitalOperation.PositionResetRobotPitch,
            AnalogOperation.PositionStartingAngle,
            DigitalOperation.DriveTrainResetXYPosition,
            AnalogOperation.DriveTrainStartingXPosition,
            AnalogOperation.DriveTrainStartingYPosition,
            AnalogOperation.DriveTrainMoveForward,
            AnalogOperation.DriveTrainMoveRight,
            AnalogOperation.DriveTrainTurnAngleGoal,
            AnalogOperation.DriveTrainTurnSpeed,
            AnalogOperation.DriveTrainRotationA,
            AnalogOperation.DriveTrainRotationB,
            AnalogOperation.DriveTrainPathXGoal,
            AnalogOperation.DriveTrainPathYGoal,
            AnalogOperation.DriveTrainPathXVelocityGoal,
            AnalogOperation.DriveTrainPathYVelocityGoal,
            AnalogOperation.DriveTrainPathAngleVelocityGoal,
            AnalogOperation.DriveTrainPositionDrive1,
            AnalogOperation.DriveTrainPositionDrive2,
            AnalogOperation.DriveTrainPositionDrive3,
            AnalogOperation.DriveTrainPositionDrive4,
            AnalogOperation.DriveTrainPositionSteer1,
            AnalogOperation.DriveTrainPositionSteer2,
            AnalogOperation.DriveTrainPositionSteer3,
            AnalogOperation.DriveTrainPositionSteer4,
            AnalogOperation.DriveTrainTurnAngleReference,
            DigitalOperation.DriveTrainSteerMode,
            DigitalOperation.DriveTrainMaintainPositionMode,
            DigitalOperation.DriveTrainPathMode,
            DigitalOperation.DriveTrainReset,
            DigitalOperation.DriveTrainEnableFieldOrientation,
            DigitalOperation.DriveTrainDisableFieldOrientation,
            DigitalOperation.DriveTrainUseRobotOrientation,
            DigitalOperation.VisionDisableStream,
            DigitalOperation.VisionEnableAprilTagProcessing,
            DigitalOperation.VisionEnableRetroreflectiveProcessing,
            DigitalOperation.VisionForceDisable,
        }),
    new MacroOperationDescription(
        MacroOperation.FollowPath3,
        UserInputDevice.Test1,
        90,
        Shift.None,
        Shift.None,
        ButtonType.Toggle,
        () -> SequentialTask.Sequence(
                new PositionStartingTask(-TuningConstants.StartGridX, 16.8, 180),
                new FollowPathTask("GuardEdgetoChargeStationFar", false, false)
                ),
        new IOperation[]
        {
            DigitalOperation.PositionResetFieldOrientation,
            DigitalOperation.PositionResetRobotPitch,
            AnalogOperation.PositionStartingAngle,
            DigitalOperation.DriveTrainResetXYPosition,
            AnalogOperation.DriveTrainStartingXPosition,
            AnalogOperation.DriveTrainStartingYPosition,
            AnalogOperation.DriveTrainMoveForward,
            AnalogOperation.DriveTrainMoveRight,
            AnalogOperation.DriveTrainTurnAngleGoal,
            AnalogOperation.DriveTrainTurnSpeed,
            AnalogOperation.DriveTrainRotationA,
            AnalogOperation.DriveTrainRotationB,
            AnalogOperation.DriveTrainPathXGoal,
            AnalogOperation.DriveTrainPathYGoal,
            AnalogOperation.DriveTrainPathXVelocityGoal,
            AnalogOperation.DriveTrainPathYVelocityGoal,
            AnalogOperation.DriveTrainPathAngleVelocityGoal,
            AnalogOperation.DriveTrainPositionDrive1,
            AnalogOperation.DriveTrainPositionDrive2,
            AnalogOperation.DriveTrainPositionDrive3,
            AnalogOperation.DriveTrainPositionDrive4,
            AnalogOperation.DriveTrainPositionSteer1,
            AnalogOperation.DriveTrainPositionSteer2,
            AnalogOperation.DriveTrainPositionSteer3,
            AnalogOperation.DriveTrainPositionSteer4,
            AnalogOperation.DriveTrainTurnAngleReference,
            DigitalOperation.DriveTrainSteerMode,
            DigitalOperation.DriveTrainMaintainPositionMode,
            DigitalOperation.DriveTrainPathMode,
            DigitalOperation.DriveTrainReset,
            DigitalOperation.DriveTrainEnableFieldOrientation,
            DigitalOperation.DriveTrainDisableFieldOrientation,
            DigitalOperation.DriveTrainUseRobotOrientation,
            DigitalOperation.VisionDisableStream,
            DigitalOperation.VisionEnableAprilTagProcessing,
            DigitalOperation.VisionEnableRetroreflectiveProcessing,
            DigitalOperation.VisionForceDisable,
        }),
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
