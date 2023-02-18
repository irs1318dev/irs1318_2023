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
            AnalogOperation.ArmLowerPositionAdjustment,
            UserInputDevice.Codriver,
            AnalogAxis.XBONE_LSY,
            Shift.CodriverDebug,
            Shift.None,
            false,
            -TuningConstants.ARM_LOWER_VELOCITY_DEAZONE,
            TuningConstants.ARM_LOWER_VELOCITY_DEAZONE,
            1.0,
            TuningConstants.MAGIC_NULL_VALUE),
        new AnalogOperationDescription(
            AnalogOperation.ArmUpperPositionAdjustment,
            UserInputDevice.Codriver,
            AnalogAxis.XBONE_RSY,
            Shift.CodriverDebug,
            Shift.None,
            false,
            -TuningConstants.ARM_UPPER_VELOCITY_DEAZONE,
            TuningConstants.ARM_UPPER_VELOCITY_DEAZONE,
            1.0,
            TuningConstants.MAGIC_NULL_VALUE),

        new AnalogOperationDescription(
            AnalogOperation.ArmSimpleForceLower,
            UserInputDevice.Codriver,
            AnalogAxis.XBONE_LSY,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ElectronicsConstants.INVERT_XBONE_LEFT_Y_AXIS,
            -TuningConstants.ARM_LOWER_VELOCITY_DEAZONE,
            TuningConstants.ARM_LOWER_VELOCITY_DEAZONE),
        new AnalogOperationDescription(
            AnalogOperation.ArmSimpleForceUpper,
            UserInputDevice.Codriver,
            AnalogAxis.XBONE_RSY,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ElectronicsConstants.INVERT_XBONE_RIGHT_Y_AXIS,
            -TuningConstants.ARM_UPPER_VELOCITY_DEAZONE,
            TuningConstants.ARM_UPPER_VELOCITY_DEAZONE,
            -1.0), // forward is negative...

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
            UserInputDeviceButton.XBONE_RIGHT_STICK_BUTTON,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainReset,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_A_BUTTON,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainEnableFieldOrientation,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_X_BUTTON,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainDisableFieldOrientation,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_X_BUTTON,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainEnableMaintainDirectionMode,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_B_BUTTON,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainDisableMaintainDirectionMode,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_B_BUTTON,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.VisionEnableAprilTagProcessing,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_A_BUTTON,
            ButtonType.Toggle),
        new DigitalOperationDescription(
            DigitalOperation.IntakeExtend,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_LEFT_BUTTON,
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.IntakeRetract,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_LEFT_BUTTON,
            Shift.DriverDebug,
            Shift.DriverDebug,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.IntakeIn,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_RIGHT_BUTTON,
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.IntakeOut,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_RIGHT_BUTTON,
            Shift.DriverDebug,
            Shift.DriverDebug,
            ButtonType.Simple),
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

        // arm macros
        new MacroOperationDescription(
            MacroOperation.GroundPickup,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_A_BUTTON,
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> new ArmMMPositionTask(
                TuningConstants.ARM_LOWER_MM_GROUND_PICKUP,
                TuningConstants.ARM_UPPER_MM_GROUND_PICKUP),
            new IOperation[]
            {
                AnalogOperation.ArmMMLowerPosition,
                AnalogOperation.ArmMMUpperPosition,
            }),
        new MacroOperationDescription(
            MacroOperation.GroundPlace,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_A_BUTTON,
            Shift.DriverDebug,
            Shift.DriverDebug,
            ButtonType.Toggle,
            () -> new ArmMMPositionTask(
                TuningConstants.ARM_LOWER_MM_GROUND_PLACING,
                TuningConstants.ARM_UPPER_MM_GROUND_PLACING),
            new IOperation[]
            {
                AnalogOperation.ArmMMLowerPosition,
                AnalogOperation.ArmMMUpperPosition,
            }),
        new MacroOperationDescription(
            MacroOperation.MiddleCone,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_B_BUTTON,
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> new ArmMMPositionTask(
                TuningConstants.ARM_LOWER_MM_MIDDLE_CONE,
                TuningConstants.ARM_UPPER_MM_MIDDLE_CONE),
            new IOperation[]
            {
                AnalogOperation.ArmMMLowerPosition,
                AnalogOperation.ArmMMUpperPosition,
            }),
        new MacroOperationDescription(
            MacroOperation.MiddleCube,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_B_BUTTON,
            Shift.DriverDebug,
            Shift.DriverDebug,
            ButtonType.Toggle,
            () -> new ArmMMPositionTask(
                TuningConstants.ARM_LOWER_MM_MIDDLE_CUBE,
                TuningConstants.ARM_UPPER_MM_MIDDLE_CUBE),
            new IOperation[]
            {
                AnalogOperation.ArmMMLowerPosition,
                AnalogOperation.ArmMMUpperPosition,
            }),
        new MacroOperationDescription(
            MacroOperation.HighCone,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_X_BUTTON,
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> new ArmMMPositionTask(
                TuningConstants.ARM_LOWER_MM_HIGH_CONE,
                TuningConstants.ARM_UPPER_MM_HIGH_CONE),
            new IOperation[]
            {
                AnalogOperation.ArmMMLowerPosition,
                AnalogOperation.ArmMMUpperPosition,
            }),
        new MacroOperationDescription(
            MacroOperation.HighCube,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_X_BUTTON,
            Shift.DriverDebug,
            Shift.DriverDebug,
            ButtonType.Toggle,
            () -> new ArmMMPositionTask(
                TuningConstants.ARM_LOWER_MM_HIGH_CUBE,
                TuningConstants.ARM_UPPER_MM_HIGH_CUBE),
            new IOperation[]
            {
                AnalogOperation.ArmMMLowerPosition,
                AnalogOperation.ArmMMUpperPosition,
            }),
        new MacroOperationDescription(
            MacroOperation.SubstationPickup,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_Y_BUTTON,
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> new ArmMMPositionTask(
                TuningConstants.ARM_LOWER_MM_SUB_PICKUP,
                TuningConstants.ARM_UPPER_MM_SUB_PICKUP),
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
            Shift.None,
            Shift.None,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new PositionStartingTask(0.0, 0.0, 0.0),
                new FollowPathTask("goForward4ft2", false, false)),
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
            MacroOperation.FollowPathTest2,
            UserInputDevice.Test1,
            180,
            Shift.None,
            Shift.None,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                    new PositionStartingTask(0.0, 0.0, 0.0),
                    new FollowPathTask("pranavTest", false, false)),
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
