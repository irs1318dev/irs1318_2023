package frc.robot.driver;

import javax.inject.Singleton;

import frc.robot.*;
import frc.robot.common.Helpers;
import frc.robot.driver.common.*;
import frc.robot.driver.common.buttons.*;
import frc.robot.driver.common.descriptions.*;
import frc.robot.driver.controltasks.*;
import frc.robot.driver.controltasks.FollowPathTask.Type;
import frc.robot.driver.controltasks.VisionMoveAndTurnTaskBase.MoveSpeed;
import frc.robot.driver.controltasks.VisionMoveAndTurnTaskBase.MoveType;
import frc.robot.driver.controltasks.VisionTurningTask.TurnType;

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
            TuningConstants.DRIVETRAIN_DEAD_ZONE_VELOCITY_Y,
            1.0,
            TuningConstants.DRIVETRAIN_EXPONENTIAL),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainMoveRight,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_LSX,
            ElectronicsConstants.INVERT_XBONE_LEFT_X_AXIS,
            -TuningConstants.DRIVETRAIN_DEAD_ZONE_VELOCITY_X,
            TuningConstants.DRIVETRAIN_DEAD_ZONE_VELOCITY_X,
            1.0,
            TuningConstants.DRIVETRAIN_EXPONENTIAL),
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
            TuningConstants.ARM_LOWER_VELOCITY_DEAZONE),
        new AnalogOperationDescription(
            AnalogOperation.ArmIKXAdjustment,
            UserInputDevice.Codriver,
            AnalogAxis.XBONE_RSX,
            Shift.CodriverDebug,
            Shift.None,
            ElectronicsConstants.INVERT_XBONE_RIGHT_X_AXIS,
            -TuningConstants.ARM_UPPER_VELOCITY_DEAZONE,
            TuningConstants.ARM_UPPER_VELOCITY_DEAZONE),

        new AnalogOperationDescription(
            AnalogOperation.ArmLowerPositionAdjustment,
            UserInputDevice.Codriver,
            AnalogAxis.XBONE_LSY,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ElectronicsConstants.INVERT_XBONE_LEFT_Y_AXIS,
            -TuningConstants.ARM_LOWER_VELOCITY_DEAZONE,
            TuningConstants.ARM_LOWER_VELOCITY_DEAZONE),
        new AnalogOperationDescription(
            AnalogOperation.ArmUpperPositionAdjustment,
            UserInputDevice.Codriver,
            AnalogAxis.XBONE_RSY,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ElectronicsConstants.INVERT_XBONE_RIGHT_Y_AXIS,
            -TuningConstants.ARM_UPPER_VELOCITY_DEAZONE,
            TuningConstants.ARM_UPPER_VELOCITY_DEAZONE),

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

        new DigitalOperationDescription(
            DigitalOperation.CubeWantedFromSubstation,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_SELECT_BUTTON,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.ConeWantedFromSubstation,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_START_BUTTON,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ButtonType.Simple),

        // Test operations:
        new DigitalOperationDescription(
            DigitalOperation.VisionEnableAprilTagProcessing,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_A_BUTTON,
            ButtonType.Toggle),
        new DigitalOperationDescription(
            DigitalOperation.RainbowTest,
            UserInputDevice.Test2,
            UserInputDeviceButton.XBONE_RIGHT_BUTTON,
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.PurpleTest,
            UserInputDevice.Test2,
            UserInputDeviceButton.XBONE_LEFT_BUTTON,
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.BlueTest,
            UserInputDevice.Test2,
            UserInputDeviceButton.XBONE_X_BUTTON,
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.YellowTest,
            UserInputDevice.Test2,
            UserInputDeviceButton.XBONE_Y_BUTTON,
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.GreenTest,
            UserInputDevice.Test2,
            UserInputDeviceButton.XBONE_A_BUTTON,
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.RedTest,
            UserInputDevice.Test2,
            UserInputDeviceButton.XBONE_B_BUTTON,
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
            MacroOperation.VisionDoubleSubstationLeft,
            UserInputDevice.Driver,
            270, // DPAD-left
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> 
                SequentialTask.Sequence(
                    new ArmMMPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_APPROACH,
                        TuningConstants.ARM_UPPER_POSITION_APPROACH),
                    new VisionTurningTask(TurnType.AprilTagParallelizing),
                    new VisionMoveAndTurnTask(TurnType.AprilTagParallelizing, MoveType.AprilTagStrafe, MoveSpeed.Normal, false, true, 0.0),
                    new VisionMoveAndTurnTask(TurnType.AprilTagCentering, MoveType.Forward, MoveSpeed.Normal, false, true, 80.0),
                    new DriveTrainFieldOrientationModeTask(true),
                    new FollowPathTask("goLeft32inForward18in"),
                    new ArmMMPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_CONE_SUBSTATION_PICKUP,
                        TuningConstants.ARM_UPPER_POSITION_CONE_SUBSTATION_PICKUP)),
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
                AnalogOperation.ArmMMUpperPosition,
                AnalogOperation.ArmMMLowerPosition,
                DigitalOperation.IntakeIn,
                DigitalOperation.IntakeOut,
                DigitalOperation.IntakeRelease,
                DigitalOperation.IntakeGrab,
            }),
        new MacroOperationDescription(
            MacroOperation.VisionDoubleSubstationRight,
            UserInputDevice.Driver,
            90, // DPAD-right
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () ->
                SequentialTask.Sequence(
                    new ArmMMPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_APPROACH,
                        TuningConstants.ARM_UPPER_POSITION_APPROACH),
                    new VisionTurningTask(TurnType.AprilTagParallelizing),
                    new VisionMoveAndTurnTask(TurnType.AprilTagParallelizing, MoveType.AprilTagStrafe, MoveSpeed.Normal, false, true, 0.0),
                    new VisionMoveAndTurnTask(TurnType.AprilTagCentering, MoveType.Forward, MoveSpeed.Normal, false, true, 80.0),
                    new DriveTrainFieldOrientationModeTask(true),
                    new FollowPathTask("goRight32inForward18in"),
                    new ArmMMPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_CONE_SUBSTATION_PICKUP,
                        TuningConstants.ARM_UPPER_POSITION_CONE_SUBSTATION_PICKUP)),
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
                AnalogOperation.ArmMMUpperPosition,
                AnalogOperation.ArmMMLowerPosition,
                DigitalOperation.IntakeIn,
                DigitalOperation.IntakeOut,
                DigitalOperation.IntakeRelease,
                DigitalOperation.IntakeGrab,
            }),
        new MacroOperationDescription(
            MacroOperation.VisionGridCube,
            UserInputDevice.Driver,
            0, // DPAD-up
            Shift.DriverDebug,
            Shift.DriverDebug,
            ButtonType.Toggle,
            () ->
                SequentialTask.Sequence(
                    new ArmMMPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_APPROACH,
                        TuningConstants.ARM_UPPER_POSITION_APPROACH),
                    new VisionTurningTask(TurnType.AprilTagParallelizing),
                    new VisionMoveAndTurnTask(TurnType.AprilTagParallelizing, MoveType.AprilTagStrafe, MoveSpeed.Normal, false, true, 0.0),
                    new VisionMoveAndTurnTask(TurnType.AprilTagCentering, MoveType.Forward, MoveSpeed.Normal, false, true, 56.0),
                    new DriveTrainFieldOrientationModeTask(true)),
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
                AnalogOperation.ArmMMUpperPosition,
                AnalogOperation.ArmMMLowerPosition,
                DigitalOperation.IntakeIn,
                DigitalOperation.IntakeOut,
                DigitalOperation.IntakeRelease,
                DigitalOperation.IntakeGrab,
            }),
        new MacroOperationDescription(
            MacroOperation.VisionGridConeLeft,
            UserInputDevice.Driver,
            270, // DPAD-left
            Shift.DriverDebug,
            Shift.DriverDebug,
            ButtonType.Toggle,
            () ->
                SequentialTask.Sequence(
                    new ArmMMPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_APPROACH,
                        TuningConstants.ARM_UPPER_POSITION_APPROACH),
                    new VisionTurningTask(TurnType.AprilTagParallelizing),
                    new VisionMoveAndTurnTask(TurnType.AprilTagParallelizing, MoveType.AprilTagStrafe, MoveSpeed.Normal, false, true, 0.0),
                    new VisionMoveAndTurnTask(TurnType.AprilTagCentering, MoveType.Forward, MoveSpeed.Normal, false, true, 56.0),
                    new DriveTrainFieldOrientationModeTask(true),
                    new FollowPathTask("goLeft22in"),
                    new VisionMoveAndTurnTask(TurnType.None, MoveType.RetroReflectiveStrafe, MoveSpeed.Normal, false, false, 0.0),
                    new DriveTrainFieldOrientationModeTask(true)),
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
                AnalogOperation.ArmMMUpperPosition,
                AnalogOperation.ArmMMLowerPosition,
                DigitalOperation.IntakeIn,
                DigitalOperation.IntakeOut,
                DigitalOperation.IntakeRelease,
                DigitalOperation.IntakeGrab,
            }),
        new MacroOperationDescription(
            MacroOperation.VisionGridConeRight,
            UserInputDevice.Driver,
            90, // DPAD-right
            Shift.DriverDebug,
            Shift.DriverDebug,
            ButtonType.Toggle,
            () ->
                SequentialTask.Sequence(
                    new ArmMMPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_APPROACH,
                        TuningConstants.ARM_UPPER_POSITION_APPROACH),
                    new VisionTurningTask(TurnType.AprilTagParallelizing),
                    new VisionMoveAndTurnTask(TurnType.AprilTagParallelizing, MoveType.AprilTagStrafe, MoveSpeed.Normal, false, true, 0.0),
                    new VisionMoveAndTurnTask(TurnType.AprilTagCentering, MoveType.Forward, MoveSpeed.Normal, false, true, 56.0),
                    new DriveTrainFieldOrientationModeTask(true),
                    new FollowPathTask("goRight22in"),
                    new VisionMoveAndTurnTask(TurnType.None, MoveType.RetroReflectiveStrafe, MoveSpeed.Normal, false, false, 0.0),
                    new DriveTrainFieldOrientationModeTask(true)),
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
                AnalogOperation.ArmMMUpperPosition,
                AnalogOperation.ArmMMLowerPosition,
                DigitalOperation.IntakeIn,
                DigitalOperation.IntakeOut,
                DigitalOperation.IntakeRelease,
                DigitalOperation.IntakeGrab,
            }),
        new MacroOperationDescription(
            MacroOperation.VisionResetPosition,
            UserInputDevice.Test1,
            270, //DPAD left
            Shift.DriverDebug,
            Shift.DriverDebug,
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
            Shift.CodriverDebug,
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
            Shift.CodriverDebug,
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

        new MacroOperationDescription(
            MacroOperation.PickUpConeFromBehind,
            UserInputDevice.Driver,
            0,
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                ConcurrentTask.AllTasks(
                    new IntakeExtendTask(true),
                    new ArmMMPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_CONE_UPRIGHTING_MACRO,
                        TuningConstants.ARM_UPPER_POSITION_CONE_UPRIGHTING_MACRO)),
                ConcurrentTask.AllTasks(
                    new FollowPathTask("goBackwards1ft"),
                    SequentialTask.Sequence(
                        new WaitTask(0.25),
                        ConcurrentTask.AllTasks(
                            new ArmMMPositionTask(
                                TuningConstants.ARM_LOWER_POSITION_GROUND_PICKUP,
                                TuningConstants.ARM_UPPER_POSITION_GROUND_PICKUP),
                            new IntakeInTask(true, 0.15)))),
                ConcurrentTask.AllTasks(
                    new IntakeGamePieceTask(0.75),
                    new IntakeExtendTask(false))),
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
                AnalogOperation.ArmMMUpperPosition,
                AnalogOperation.ArmMMLowerPosition,
                DigitalOperation.IntakeIn,
                DigitalOperation.IntakeOut,
                DigitalOperation.IntakeRelease,
                DigitalOperation.IntakeGrab,
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
            180, // POV-down
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
            Shift.CodriverDebug,
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
            Shift.None,
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
            Shift.None,
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
            Shift.None,
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
            Shift.None,
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
            MacroOperation.ArmConeSubstationPickupPosition,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_X_BUTTON,
            Shift.None,
            Shift.None,
            ButtonType.Toggle,
            () -> new ArmMMPositionTask(
                TuningConstants.ARM_LOWER_POSITION_CONE_SUBSTATION_PICKUP,
                TuningConstants.ARM_UPPER_POSITION_CONE_SUBSTATION_PICKUP),
            new IOperation[]
            {
                AnalogOperation.ArmMMLowerPosition,
                AnalogOperation.ArmMMUpperPosition,
            }),
        new MacroOperationDescription(
            MacroOperation.ArmCubeSubstationPickupPosition,
            UserInputDevice.Codriver,
            270, // POV-left
            Shift.None,
            Shift.None,
            ButtonType.Toggle,
            () -> new ArmMMPositionTask(
                TuningConstants.ARM_LOWER_POSITION_CUBE_SUBSTATION_PICKUP,
                TuningConstants.ARM_UPPER_POSITION_CUBE_SUBSTATION_PICKUP),
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
                new ResetLevelTask(),
                new ChargeStationTaskv2(false),
                ConcurrentTask.AllTasks(
                    new PIDBrakeTask(),
                    new WaitTask(0.5))),
            new IOperation[]
            {
                DigitalOperation.PositionResetRobotLevel,
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
                DigitalOperation.DriveTrainEnableMaintainDirectionMode,
                DigitalOperation.DriveTrainIgnoreSlewRateLimitingMode
            }),
        new MacroOperationDescription(
            MacroOperation.ChargeStationBalanceReverse,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_START_BUTTON, // Left menu button
            Shift.Test1Debug,
            Shift.None,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new ResetLevelTask(), // calibration
                new ChargeStationTaskv2(true),
                ConcurrentTask.AllTasks(
                    new PIDBrakeTask(),
                    new WaitTask(0.5))),
            new IOperation[]
            {
                DigitalOperation.PositionResetRobotLevel,
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
                DigitalOperation.DriveTrainEnableMaintainDirectionMode,
                DigitalOperation.DriveTrainIgnoreSlewRateLimitingMode
            }),

        new MacroOperationDescription(
            MacroOperation.FollowPathTest1,
            UserInputDevice.Test1,
            0,
            Shift.Test1Debug,
            Shift.None,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                ConcurrentTask.AllTasks(
                    new ResetLevelTask(),
                    new PositionStartingTask(
                        false ? TuningConstants.StartGridX : -TuningConstants.StartGridX,
                        TuningConstants.StartFiveGridY,
                        false ? 0.0 : 180.0,
                        true,
                        true)),
    
                ConcurrentTask.AllTasks(
                    new FollowPathTask(false ? "5To11Red" : "5To11Blue", Type.Absolute),
                    SequentialTask.Sequence(
                        new WaitTask(0.5),
                        new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_HIGH_CUBE, TuningConstants.ARM_UPPER_POSITION_HIGH_CUBE)
                    )
                ),
    
                new FollowPathTask(false ? "11To5Red" : "11To5Blue", Type.Absolute),
                new IntakeExtendTask(true),
                new IntakeInTask(false, 1.5),
    
                ConcurrentTask.AllTasks(
                    new FollowPathTask(false ? "5To11Red" : "5To11Blue", Type.Absolute),
                    SequentialTask.Sequence(
                        new WaitTask(0.5),
                        new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_STOWED, TuningConstants.ARM_UPPER_POSITION_STOWED)
                    )
                ),

                new ResetLevelTask(),
                new ChargeStationTaskv2(true)
            ),
            new IOperation[]
            {
                DigitalOperation.PositionResetFieldOrientation,
                DigitalOperation.PositionResetRobotLevel,
                AnalogOperation.PositionStartingAngle,
                DigitalOperation.DriveTrainResetXYPosition,
                DigitalOperation.DriveTrainEnableMaintainDirectionMode,
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
                AnalogOperation.ArmMMUpperPosition,
                AnalogOperation.ArmMMLowerPosition,
                DigitalOperation.IntakeIn,
                DigitalOperation.IntakeOut,
                DigitalOperation.IntakeRelease,
                DigitalOperation.IntakeGrab,
            }),

        // Full auton test
        new MacroOperationDescription(
            MacroOperation.FollowPathTest2,
            UserInputDevice.Test1,
            180,
            Shift.None,
            Shift.None,
            ButtonType.Toggle,
            () -> new VisionMoveAndTurnTask(TurnType.None, MoveType.RetroReflectiveStrafe, MoveSpeed.Normal, false, false, 0.0),
            new IOperation[]
            {
                DigitalOperation.IntakeIn,
                DigitalOperation.IntakeOut,
                DigitalOperation.IntakeRelease,
                DigitalOperation.IntakeGrab,
                AnalogOperation.ArmMMLowerPosition,
                AnalogOperation.ArmMMUpperPosition,
                DigitalOperation.PositionResetFieldOrientation,
                DigitalOperation.PositionResetRobotLevel,
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
        MacroOperation.FollowPathTest3,
        UserInputDevice.Test1,
        270,
        Shift.None,
        Shift.None,
        ButtonType.Toggle,
        () -> SequentialTask.Sequence(
                new PositionStartingTask(-TuningConstants.GuardEdgeStartX, 17.5, 180),
                new FollowPathTask("GuardStartTo9", Type.Absolute),
                new WaitTask(0.3),
                new FollowPathTask("9To12", Type.Absolute),
                new FollowPathTask("12To23", Type.Absolute),
                new FollowPathTask("23To23", Type.Absolute),
                new FollowPathTask("23To12", Type.Absolute),
                new FollowPathTask("12To7", Type.Absolute)
                ),
        new IOperation[]
        {
            DigitalOperation.PositionResetFieldOrientation,
            DigitalOperation.PositionResetRobotLevel,
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
        MacroOperation.FollowPathTest4,
        UserInputDevice.Test1,
        90,
        Shift.None,
        Shift.None,
        ButtonType.Toggle,
        () -> SequentialTask.Sequence(
                new PositionStartingTask(-TuningConstants.StartGridX, 16.8, 180),
                new FollowPathTask("GuardEdgeToChargeStationFar", Type.Absolute)
                ),
        new IOperation[]
        {
            DigitalOperation.PositionResetFieldOrientation,
            DigitalOperation.PositionResetRobotLevel,
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
