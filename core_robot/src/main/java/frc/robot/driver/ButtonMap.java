package frc.robot.driver;

import javax.inject.Singleton;

import frc.lib.driver.*;
import frc.lib.driver.buttons.*;
import frc.lib.driver.descriptions.*;
import frc.lib.helpers.Helpers;
import frc.robot.*;
import frc.robot.driver.controltasks.*;
import frc.robot.driver.controltasks.ArmLAPositionTask.IntakeState;
import frc.robot.driver.controltasks.ChargeStationTask.Orientation;
import frc.robot.driver.controltasks.FollowPathTask.Type;
import frc.robot.driver.controltasks.VisionAprilTagTranslateTask.GridScoringPosition;
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
            AnalogAxis.XBONE_RSY,
            Shift.CodriverDebug,
            Shift.None,
            ElectronicsConstants.INVERT_XBONE_RIGHT_Y_AXIS,
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
            AnalogOperation.ArmTwistRight,
            UserInputDevice.Codriver,
            AnalogAxis.XBONE_LT,
            ElectronicsConstants.INVERT_XBONE_RIGHT_TRIGGER,
            TuningConstants.ZERO,
            TuningConstants.ARM_TWIST_DEAZONE),
        new AnalogOperationDescription(
            AnalogOperation.ArmTwistLeft,
            UserInputDevice.Codriver,
            AnalogAxis.XBONE_RT,
            ElectronicsConstants.INVERT_XBONE_LEFT_TRIGGER,
            TuningConstants.ZERO,
            TuningConstants.ARM_TWIST_DEAZONE),

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
            0, // DPAD-up
            Shift.DriverDebug,
            Shift.DriverDebug,
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
            270,
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainDisableFieldOrientation,
            UserInputDevice.Driver,
            270,
            Shift.DriverDebug,
            Shift.DriverDebug,
            ButtonType.Click),
        // new DigitalOperationDescription(
        //     DigitalOperation.DriveTrainEnableMaintainDirectionMode,
        //     UserInputDevice.Driver,
        //     90,
        //     Shift.DriverDebug,
        //     Shift.None,
        //     ButtonType.Click),
        // new DigitalOperationDescription(
        //     DigitalOperation.DriveTrainDisableMaintainDirectionMode,
        //     UserInputDevice.Driver,
        //     90,
        //     Shift.DriverDebug,
        //     Shift.DriverDebug,
        //     ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainSlowMode,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_A_BUTTON,
            Shift.None,
            Shift.None,
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
            DigitalOperation.IntakeDown,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_START_BUTTON,
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.IntakeUp,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_START_BUTTON,
            Shift.DriverDebug,
            Shift.DriverDebug,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.IntakeCone,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_LEFT_BUTTON,
            Shift.None,
            Shift.None,
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.IntakeCube,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_RIGHT_BUTTON,
            Shift.None,
            Shift.None,
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
            Shift.None,
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.ConeWantedFromSubstation,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_START_BUTTON,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.ForceRainbow,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_SELECT_BUTTON,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ButtonType.Simple),
        
            
            

        // Test operations:
        new DigitalOperationDescription(
            DigitalOperation.VisionEnableAprilTagProcessing,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_A_BUTTON,
            Shift.Test1Debug,
            Shift.None,
            ButtonType.Toggle),
        new DigitalOperationDescription(
            DigitalOperation.VisionEnableRetroreflectiveProcessing,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_A_BUTTON,
            Shift.Test1Debug,
            Shift.Test1Debug,
            ButtonType.Toggle),
    };

    public static MacroOperationDescription[] MacroSchema = new MacroOperationDescription[]
    {
        // driving macros
        // new MacroOperationDescription(
        //     MacroOperation.PIDLightBrake,
        //     UserInputDevice.Driver,
        //     UserInputDeviceButton.XBONE_LEFT_STICK_BUTTON,
        //     Shift.DriverDebug,
        //     Shift.None,
        //     ButtonType.Simple,
        //     () -> new PIDBrakeTask(false),
        //     new IOperation[]
        //     {
        //         AnalogOperation.DriveTrainMoveForward,
        //         AnalogOperation.DriveTrainMoveRight,
        //         AnalogOperation.DriveTrainTurnAngleGoal,
        //         AnalogOperation.DriveTrainSpinLeft,
        //         AnalogOperation.DriveTrainSpinRight,
        //         AnalogOperation.DriveTrainRotationA,
        //         AnalogOperation.DriveTrainRotationB,
        //         AnalogOperation.DriveTrainPathXGoal,
        //         AnalogOperation.DriveTrainPathYGoal,
        //         AnalogOperation.DriveTrainPathXVelocityGoal,
        //         AnalogOperation.DriveTrainPathYVelocityGoal,
        //         AnalogOperation.DriveTrainPathAngleVelocityGoal,
        //         AnalogOperation.DriveTrainPositionDrive1,
        //         AnalogOperation.DriveTrainPositionDrive2,
        //         AnalogOperation.DriveTrainPositionDrive3,
        //         AnalogOperation.DriveTrainPositionDrive4,
        //         AnalogOperation.DriveTrainPositionSteer1,
        //         AnalogOperation.DriveTrainPositionSteer2,
        //         AnalogOperation.DriveTrainPositionSteer3,
        //         AnalogOperation.DriveTrainPositionSteer4,
        //         DigitalOperation.DriveTrainSteerMode,
        //         DigitalOperation.DriveTrainMaintainPositionMode,
        //         DigitalOperation.DriveTrainPathMode,
        //         DigitalOperation.DriveTrainReset,
        //         DigitalOperation.DriveTrainEnableFieldOrientation,
        //         DigitalOperation.DriveTrainDisableFieldOrientation,
        //         DigitalOperation.DriveTrainUseRobotOrientation,
        //     }),
        // new MacroOperationDescription(
        //     MacroOperation.PIDHeavyBrake,
        //     UserInputDevice.Driver,
        //     UserInputDeviceButton.XBONE_LEFT_STICK_BUTTON,
        //     Shift.DriverDebug,
        //     Shift.DriverDebug,
        //     ButtonType.Simple,
        //     () -> new PIDBrakeTask(true),
        //     new IOperation[]
        //     {
        //         AnalogOperation.DriveTrainMoveForward,
        //         AnalogOperation.DriveTrainMoveRight,
        //         AnalogOperation.DriveTrainTurnAngleGoal,
        //         AnalogOperation.DriveTrainSpinLeft,
        //         AnalogOperation.DriveTrainSpinRight,
        //         AnalogOperation.DriveTrainRotationA,
        //         AnalogOperation.DriveTrainRotationB,
        //         AnalogOperation.DriveTrainPathXGoal,
        //         AnalogOperation.DriveTrainPathYGoal,
        //         AnalogOperation.DriveTrainPathXVelocityGoal,
        //         AnalogOperation.DriveTrainPathYVelocityGoal,
        //         AnalogOperation.DriveTrainPathAngleVelocityGoal,
        //         AnalogOperation.DriveTrainPositionDrive1,
        //         AnalogOperation.DriveTrainPositionDrive2,
        //         AnalogOperation.DriveTrainPositionDrive3,
        //         AnalogOperation.DriveTrainPositionDrive4,
        //         AnalogOperation.DriveTrainPositionSteer1,
        //         AnalogOperation.DriveTrainPositionSteer2,
        //         AnalogOperation.DriveTrainPositionSteer3,
        //         AnalogOperation.DriveTrainPositionSteer4,
        //         DigitalOperation.DriveTrainSteerMode,
        //         DigitalOperation.DriveTrainMaintainPositionMode,
        //         DigitalOperation.DriveTrainPathMode,
        //         DigitalOperation.DriveTrainReset,
        //         DigitalOperation.DriveTrainEnableFieldOrientation,
        //         DigitalOperation.DriveTrainDisableFieldOrientation,
        //         DigitalOperation.DriveTrainUseRobotOrientation,
        //     }),
        // new MacroOperationDescription(
        //     MacroOperation.VisionDoubleSubstationLeft,
        //     UserInputDevice.Driver,
        //     270, // DPAD-left
        //     Shift.DriverDebug,
        //     Shift.None,
        //     ButtonType.Toggle,
        //     () -> ConcurrentTask.AnyTasks(
        //         SequentialTask.Sequence(
        //             ConcurrentTask.AllTasks(
        //                 new OrientationTask(0.0),
        //                 new ArmMMPositionTask(
        //                     TuningConstants.ARM_LOWER_POSITION_APPROACH,
        //                     TuningConstants.ARM_UPPER_POSITION_APPROACH)),
        //             new VisionMoveAndTurnTask(TurnType.None, MoveType.AprilTagStrafe, MoveSpeed.Normal, false, false, -32.0),
        //             new ArmMMPositionTask(
        //                 TuningConstants.ARM_LOWER_POSITION_CONE_SUBSTATION_PICKUP,
        //                 TuningConstants.ARM_UPPER_POSITION_CONE_SUBSTATION_PICKUP)),
        //         new RumbleTask(),
        //         new StrobeTask()),
        //     new IOperation[]
        //     {
        //         DigitalOperation.ForcePurpleStrobe,
        //         DigitalOperation.ForceLightDriverRumble,
        //         AnalogOperation.DriveTrainMoveForward,
        //         AnalogOperation.DriveTrainMoveRight,
        //         AnalogOperation.DriveTrainTurnAngleGoal,
        //         AnalogOperation.DriveTrainSpinLeft,
        //         AnalogOperation.DriveTrainSpinRight,
        //         AnalogOperation.DriveTrainRotationA,
        //         AnalogOperation.DriveTrainRotationB,
        //         AnalogOperation.DriveTrainPathXGoal,
        //         AnalogOperation.DriveTrainPathYGoal,
        //         AnalogOperation.DriveTrainPathXVelocityGoal,
        //         AnalogOperation.DriveTrainPathYVelocityGoal,
        //         AnalogOperation.DriveTrainPathAngleVelocityGoal,
        //         AnalogOperation.DriveTrainPositionDrive1,
        //         AnalogOperation.DriveTrainPositionDrive2,
        //         AnalogOperation.DriveTrainPositionDrive3,
        //         AnalogOperation.DriveTrainPositionDrive4,
        //         AnalogOperation.DriveTrainPositionSteer1,
        //         AnalogOperation.DriveTrainPositionSteer2,
        //         AnalogOperation.DriveTrainPositionSteer3,
        //         AnalogOperation.DriveTrainPositionSteer4,
        //         DigitalOperation.DriveTrainSteerMode,
        //         DigitalOperation.DriveTrainMaintainPositionMode,
        //         DigitalOperation.DriveTrainPathMode,
        //         DigitalOperation.DriveTrainReset,
        //         DigitalOperation.DriveTrainEnableFieldOrientation,
        //         DigitalOperation.DriveTrainDisableFieldOrientation,
        //         DigitalOperation.DriveTrainUseRobotOrientation,
        //         DigitalOperation.VisionDisableStream,
        //         DigitalOperation.VisionEnableAprilTagProcessing,
        //         DigitalOperation.VisionEnableRetroreflectiveProcessing,
        //         DigitalOperation.VisionForceDisable,
        //         AnalogOperation.ArmMMUpperPosition,
        //         AnalogOperation.ArmMMLowerPosition,
        //         DigitalOperation.IntakeCube,
        //         DigitalOperation.IntakeCone,
        //         DigitalOperation.IntakeDown,
        //         DigitalOperation.IntakeUp,
        //     }),
        // new MacroOperationDescription(
        //     MacroOperation.VisionDoubleSubstationRight,
        //     UserInputDevice.Driver,
        //     90, // DPAD-right
        //     Shift.DriverDebug,
        //     Shift.None,
        //     ButtonType.Toggle,
        //     () -> ConcurrentTask.AnyTasks(
        //         SequentialTask.Sequence(
        //             ConcurrentTask.AllTasks(
        //                 new OrientationTask(0.0),
        //                 new ArmMMPositionTask(
        //                     TuningConstants.ARM_LOWER_POSITION_APPROACH,
        //                     TuningConstants.ARM_UPPER_POSITION_APPROACH)),
        //             new VisionMoveAndTurnTask(TurnType.None, MoveType.AprilTagStrafe, MoveSpeed.Normal, false, false, 32.0),
        //             new ArmMMPositionTask(
        //                 TuningConstants.ARM_LOWER_POSITION_CONE_SUBSTATION_PICKUP,
        //                 TuningConstants.ARM_UPPER_POSITION_CONE_SUBSTATION_PICKUP)),
        //         new RumbleTask(),
        //         new StrobeTask()),
        //     new IOperation[]
        //     {
        //         DigitalOperation.ForcePurpleStrobe,
        //         DigitalOperation.ForceLightDriverRumble,
        //         AnalogOperation.DriveTrainMoveForward,
        //         AnalogOperation.DriveTrainMoveRight,
        //         AnalogOperation.DriveTrainTurnAngleGoal,
        //         AnalogOperation.DriveTrainSpinLeft,
        //         AnalogOperation.DriveTrainSpinRight,
        //         AnalogOperation.DriveTrainRotationA,
        //         AnalogOperation.DriveTrainRotationB,
        //         AnalogOperation.DriveTrainPathXGoal,
        //         AnalogOperation.DriveTrainPathYGoal,
        //         AnalogOperation.DriveTrainPathXVelocityGoal,
        //         AnalogOperation.DriveTrainPathYVelocityGoal,
        //         AnalogOperation.DriveTrainPathAngleVelocityGoal,
        //         AnalogOperation.DriveTrainPositionDrive1,
        //         AnalogOperation.DriveTrainPositionDrive2,
        //         AnalogOperation.DriveTrainPositionDrive3,
        //         AnalogOperation.DriveTrainPositionDrive4,
        //         AnalogOperation.DriveTrainPositionSteer1,
        //         AnalogOperation.DriveTrainPositionSteer2,
        //         AnalogOperation.DriveTrainPositionSteer3,
        //         AnalogOperation.DriveTrainPositionSteer4,
        //         DigitalOperation.DriveTrainSteerMode,
        //         DigitalOperation.DriveTrainMaintainPositionMode,
        //         DigitalOperation.DriveTrainPathMode,
        //         DigitalOperation.DriveTrainReset,
        //         DigitalOperation.DriveTrainEnableFieldOrientation,
        //         DigitalOperation.DriveTrainDisableFieldOrientation,
        //         DigitalOperation.DriveTrainUseRobotOrientation,
        //         DigitalOperation.VisionDisableStream,
        //         DigitalOperation.VisionEnableAprilTagProcessing,
        //         DigitalOperation.VisionEnableRetroreflectiveProcessing,
        //         DigitalOperation.VisionForceDisable,
        //         AnalogOperation.ArmMMUpperPosition,
        //         AnalogOperation.ArmMMLowerPosition,
        //         DigitalOperation.IntakeCube,
        //         DigitalOperation.IntakeCone,
        //         DigitalOperation.IntakeDown,
        //         DigitalOperation.IntakeUp,
        //     }),

        new MacroOperationDescription(
            MacroOperation.FaceSubstationWall,
            UserInputDevice.Driver,
            90,
            ButtonType.Toggle,
            () -> new FaceSubstationWallTask(),
            new IOperation[]
            {
                AnalogOperation.DriveTrainTurnAngleGoal,
            }),
        new MacroOperationDescription(
            MacroOperation.ArmSubstationHover,
            UserInputDevice.Test2,
            UserInputDeviceButton.XBONE_A_BUTTON,
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> ConcurrentTask.AnyTasks(
                SequentialTask.Sequence(
                    new ArmIKPositionTask(TuningConstants.ARM_UPPER_POSITION_INITIAL_HOVER_SUBSTATION, TuningConstants.ARM_LOWER_POSITION_INITIAL_HOVER_SUBSTATION),
                    new ArmIKPositionTask(TuningConstants.ARM_UPPER_POSITION_TARGET_HOVER_SUBSTATION_MIDPOINT, TuningConstants.ARM_LOWER_POSITION_TARGET_HOVER_SUBSTATION_MIDPOINT),
                    ConcurrentTask.AllTasks(
                        new ArmIKPositionTask(TuningConstants.ARM_UPPER_POSITION_TARGET_HOVER_SUBSTATION_FINAL, TuningConstants.ARM_LOWER_POSITION_TARGET_HOVER_SUBSTATION_FINAL),
                        new IntakeGamePieceTask(false, 1)
                    )
                )),
            new IOperation[]
            {
                DigitalOperation.IntakeCube,
                DigitalOperation.IntakeCone
            }),

        new MacroOperationDescription(
            MacroOperation.VisionGridCube,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_Y_BUTTON,
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> ConcurrentTask.AnyTasks(
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new OrientationTask(180.0),
                        new CheckArmPositionTask(
                            TuningConstants.ARM_LOWER_POSITION_APPROACH,
                            TuningConstants.ARM_UPPER_POSITION_APPROACH,
                            IntakeState.Up,
                            TuningConstants.ARM_IKZ_MINIMUM_VISION_HEIGHT)),
                    ConcurrentTask.AnyTasks(
                        new VisionAprilTagTranslateTask(GridScoringPosition.MiddleCube),
                        new OrientationTask(180.0, true, true))
                    // ConcurrentTask.AnyTasks(
                    //     new VisionAprilTagTranslateTask(GridScoringPosition.MiddleCube),
                    //     new OrientationTask(180.0, true, true))
                        ),
                    new RumbleTask(),
                new StrobeTask()),
            new IOperation[]
            {
                DigitalOperation.ForcePurpleStrobe,
                DigitalOperation.ForceLightDriverRumble,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
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
                DigitalOperation.IntakeDown,
                DigitalOperation.IntakeUp,
            },
            new IOperation[]
            {
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
                DigitalOperation.IntakeCube,
            }),
        new MacroOperationDescription(
            MacroOperation.VisionGridConeLeft,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_X_BUTTON,
            Shift.None,
            Shift.None,
            ButtonType.Toggle,
            () -> ConcurrentTask.AnyTasks(
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new OrientationTask(180.0),
                        new CheckArmPositionTask(
                            TuningConstants.ARM_LOWER_POSITION_APPROACH,
                            TuningConstants.ARM_UPPER_POSITION_APPROACH,
                            IntakeState.Down,
                            TuningConstants.ARM_IKZ_MINIMUM_VISION_HEIGHT)),
                    ConcurrentTask.AnyTasks(
                        new VisionAprilTagTranslateTask(GridScoringPosition.LeftCone),
                        new OrientationTask(180.0, true, true))
                    // ConcurrentTask.AnyTasks(
                    //     new VisionAprilTagTranslateTask(GridScoringPosition.LeftCone),
                    //     new OrientationTask(180.0, true, true))
                        ),
                    // new VisionMoveAndTurnTask(TurnType.None, MoveType.RetroReflectiveStrafe, MoveSpeed.Normal, false, false, 0.0)
                new RumbleTask(),
                new StrobeTask()),
            new IOperation[]
            {
                DigitalOperation.ForcePurpleStrobe,
                DigitalOperation.ForceLightDriverRumble,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
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
                DigitalOperation.IntakeDown,
                DigitalOperation.IntakeUp,
            },
            new IOperation[]
            {
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
                DigitalOperation.IntakeCube,
            }),
        new MacroOperationDescription(
            MacroOperation.VisionGridConeRight,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_B_BUTTON,
            Shift.None,
            Shift.None,
            ButtonType.Toggle,
            () -> ConcurrentTask.AllTasks(
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new OrientationTask(180.0, true),
                        new CheckArmPositionTask(
                            TuningConstants.ARM_LOWER_POSITION_APPROACH,
                            TuningConstants.ARM_UPPER_POSITION_APPROACH,
                            IntakeState.Down,
                            TuningConstants.ARM_IKZ_MINIMUM_VISION_HEIGHT)),
                    ConcurrentTask.AnyTasks(
                        new VisionAprilTagTranslateTask(GridScoringPosition.RightCone),
                        new OrientationTask(180.0, true, true))
                    // ConcurrentTask.AnyTasks(
                        // new VisionAprilTagTranslateTask(GridScoringPosition.RightCone),
                        // new OrientationTask(180.0, true, true))
                        ),
                    // new VisionMoveAndTurnTask(TurnType.None, MoveType.RetroReflectiveStrafe, MoveSpeed.Normal, false, false, 0.0)),
                new RumbleTask(),
                new StrobeTask()),
            new IOperation[]
            {
                DigitalOperation.ForcePurpleStrobe,
                DigitalOperation.ForceLightDriverRumble,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
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
                DigitalOperation.IntakeDown,
                DigitalOperation.IntakeUp,
            },
            new IOperation[]
            {
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
                DigitalOperation.IntakeCube,
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

        new MacroOperationDescription(
            MacroOperation.PIDBackupBrake,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_START_BUTTON, // Right random button
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ButtonType.Simple,
            () -> new PIDBrakeTask(true),
            new IOperation[]
            {
                DigitalOperation.PositionResetRobotLevel,
                DigitalOperation.PositionResetFieldOrientation,
                AnalogOperation.PositionStartingAngle,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
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
            },
            new IOperation[]
            {
                DigitalOperation.PositionResetRobotLevel,
                DigitalOperation.PositionResetFieldOrientation,
                AnalogOperation.PositionStartingAngle
            }),
        
        new MacroOperationDescription(
            MacroOperation.FaceForward,
            UserInputDevice.Driver,
            0, // DPAD-up
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> new OrientationTask(0),
            new IOperation[]
            {
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
            }),
        
        new MacroOperationDescription(
            MacroOperation.FaceBackward,
            UserInputDevice.Driver,
            180, // DPAD-down
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> new OrientationTask(180),
            new IOperation[]
            {
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
            }),

        new MacroOperationDescription(
            MacroOperation.PickUpConeFromBehind,
            UserInputDevice.Driver,
            180, // DPAD-down
            Shift.DriverDebug,
            Shift.DriverDebug,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new ArmLAPositionTask(
                    TuningConstants.ARM_LOWER_POSITION_CONE_UPRIGHTING_MACRO,
                    TuningConstants.ARM_UPPER_POSITION_CONE_UPRIGHTING_MACRO,
                    true),
                new FollowPathTask("goBackwards30in")),
            new IOperation[]
            {
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
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
                DigitalOperation.IntakeCube,
                DigitalOperation.IntakeCone,
                DigitalOperation.IntakeDown,
                DigitalOperation.IntakeUp,
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
            MacroOperation.ArmGroundPickupPositionCone,
            UserInputDevice.Codriver,
            180, // POV-down
            Shift.None,
            Shift.None,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new IntakePositionTask(false),
                new ArmLAPositionTask(
                    TuningConstants.ARM_LOWER_POSITION_CONE_GROUND_PICKUP,
                    TuningConstants.ARM_UPPER_POSITION_CONE_GROUND_PICKUP,
                    true,
                    IntakeState.Up),
                new IndicatorTask(2.0, DigitalOperation.SubstationIntakeReady)),
            new IOperation[]
            {
                AnalogOperation.ArmMMLowerPosition,
                AnalogOperation.ArmMMUpperPosition,
                DigitalOperation.IntakeDown,
                DigitalOperation.IntakeUp,
                DigitalOperation.SubstationIntakeReady,
                DigitalOperation.ForcePurpleStrobe,
                DigitalOperation.ForceRainbow
            }),

        new MacroOperationDescription(
            MacroOperation.ArmMiddleConePosition,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_B_BUTTON,
            Shift.None,
            Shift.None,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new ArmLAPositionTask(
                    TuningConstants.ARM_LOWER_POSITION_MIDDLE_CONE,
                    TuningConstants.ARM_UPPER_POSITION_MIDDLE_CONE,
                    IntakeState.Down),
                new IntakePositionTask(true)),
            new IOperation[]
            {
                AnalogOperation.ArmMMLowerPosition,
                AnalogOperation.ArmMMUpperPosition,
                DigitalOperation.IntakeDown,
                DigitalOperation.IntakeUp,
            }),
        new MacroOperationDescription(
            MacroOperation.ArmMiddleCubePosition,
            UserInputDevice.Codriver,
            90, // POV-right
            Shift.None,
            Shift.None,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new ArmLAPositionTask(
                    TuningConstants.ARM_LOWER_POSITION_MIDDLE_CUBE,
                    TuningConstants.ARM_UPPER_POSITION_MIDDLE_CUBE,
                    IntakeState.Up),
                new IntakePositionTask(false)),
            new IOperation[]
            {
                AnalogOperation.ArmMMLowerPosition,
                AnalogOperation.ArmMMUpperPosition,
                DigitalOperation.IntakeDown,
                DigitalOperation.IntakeUp,
            }),
        new MacroOperationDescription(
            MacroOperation.ArmHighConePositionUp,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_Y_BUTTON,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new ArmLAPositionTask(
                    TuningConstants.ARM_LOWER_POSITION_HIGH_CONE_UP,
                    TuningConstants.ARM_UPPER_POSITION_HIGH_CONE_UP,
                    IntakeState.Up),
                new IntakePositionTask(false)),
            new IOperation[]
            {
                AnalogOperation.ArmMMLowerPosition,
                AnalogOperation.ArmMMUpperPosition,
                DigitalOperation.IntakeDown,
                DigitalOperation.IntakeUp,
            }),
        new MacroOperationDescription(
            MacroOperation.ArmHighConePositionDown,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_Y_BUTTON,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new ArmLAPositionTask(
                    TuningConstants.ARM_LOWER_POSITION_HIGH_CONE_DOWN,
                    TuningConstants.ARM_UPPER_POSITION_HIGH_CONE_DOWN,
                    IntakeState.Down),
                new IntakePositionTask(true)),
            new IOperation[]
            {
                AnalogOperation.ArmMMLowerPosition,
                AnalogOperation.ArmMMUpperPosition,
                DigitalOperation.IntakeDown,
                DigitalOperation.IntakeUp,
            }),
        new MacroOperationDescription(
            MacroOperation.ArmHighCubePosition,
            UserInputDevice.Codriver,
            0, // POV-up
            Shift.None,
            Shift.None,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new ArmLAPositionTask(
                    TuningConstants.ARM_LOWER_POSITION_HIGH_CUBE,
                    TuningConstants.ARM_UPPER_POSITION_HIGH_CUBE,
                    IntakeState.Up),
                new IntakePositionTask(false)),
            new IOperation[]
            {
                AnalogOperation.ArmMMLowerPosition,
                AnalogOperation.ArmMMUpperPosition,
                DigitalOperation.IntakeDown,
                DigitalOperation.IntakeUp,
            }),
        new MacroOperationDescription(
            MacroOperation.ArmConeSubstationPickupPosition,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_X_BUTTON,
            Shift.None,
            Shift.None,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new IntakePositionTask(true),
                new ArmLAPositionTask(
                    TuningConstants.ARM_LOWER_POSITION_CONE_SUBSTATION_PICKUP_APPROACH,
                    TuningConstants.ARM_UPPER_POSITION_CONE_SUBSTATION_PICKUP_APPROACH,
                    true),
                new ArmLAPositionTask(
                    TuningConstants.ARM_LOWER_POSITION_CONE_SUBSTATION_PICKUP,
                    TuningConstants.ARM_UPPER_POSITION_CONE_SUBSTATION_PICKUP,
                    true,
                    IntakeState.Down),
                new IndicatorTask(2.0, DigitalOperation.SubstationIntakeReady)),
            new IOperation[]
            {
                AnalogOperation.ArmMMLowerPosition,
                AnalogOperation.ArmMMUpperPosition,
                DigitalOperation.IntakeDown,
                DigitalOperation.IntakeUp,
                DigitalOperation.SubstationIntakeReady,
                DigitalOperation.ForcePurpleStrobe,
                DigitalOperation.ForceRainbow
            }),

        new MacroOperationDescription(
            MacroOperation.ArmGroundPickupPositionCube,
            UserInputDevice.Codriver,
            270, // POV-left
            Shift.None,
            Shift.None,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new ArmLAPositionTask(
                    TuningConstants.ARM_LOWER_POSITION_CUBE_GROUND_PICKUP,
                    TuningConstants.ARM_UPPER_POSITION_CUBE_GROUND_PICKUP,
                    true,
                    IntakeState.Down),
                new IntakePositionTask(true),
                new IndicatorTask(2.0, DigitalOperation.SubstationIntakeReady)),
            new IOperation[]
            {
                AnalogOperation.ArmMMLowerPosition,
                AnalogOperation.ArmMMUpperPosition,
                DigitalOperation.IntakeDown,
                DigitalOperation.IntakeUp,
                DigitalOperation.SubstationIntakeReady,
                DigitalOperation.ForcePurpleStrobe,
                DigitalOperation.ForceRainbow
            }),

        
        // new MacroOperationDescription(
        //     MacroOperation.ArmCubeSingleSubstationPickupPosition,
        //     UserInputDevice.Codriver,
        //     270, // POV-left
        //     Shift.CodriverDebug,
        //     Shift.CodriverDebug,
        //     ButtonType.Toggle,
        //     () -> SequentialTask.Sequence(
        //         new ArmMMPositionTask(
        //             TuningConstants.ARM_LOWER_POSITION_CUBE_SINGLE_SUBSTATION_PICKUP,
        //             TuningConstants.ARM_UPPER_POSITION_CUBE_SINGLE_SUBSTATION_PICKUP,
        //             IntakeState.Up),
        //         new IntakePositionTask(false)),
        //     new IOperation[]
        //     {
        //         AnalogOperation.ArmMMLowerPosition,
        //         AnalogOperation.ArmMMUpperPosition,
        //         DigitalOperation.IntakeDown,
        //         DigitalOperation.IntakeUp,
        //     }),
        new MacroOperationDescription(
            MacroOperation.ArmStowedPosition,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_A_BUTTON,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new ArmLAPositionTask(
                    TuningConstants.ARM_LOWER_POSITION_STOWED,
                    TuningConstants.ARM_UPPER_POSITION_STOWED,
                    IntakeState.Up),
                new IntakePositionTask(false)),
            new IOperation[]
            {
                AnalogOperation.ArmMMLowerPosition,
                AnalogOperation.ArmMMUpperPosition,
                DigitalOperation.IntakeDown,
                DigitalOperation.IntakeUp,
            }),
        new MacroOperationDescription(
            MacroOperation.ArmApproachPosition,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_A_BUTTON,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ButtonType.Toggle,
            () -> new ArmLAPositionTask(
                TuningConstants.ARM_LOWER_POSITION_APPROACH,
                TuningConstants.ARM_UPPER_POSITION_APPROACH),
            new IOperation[]
            {
                AnalogOperation.ArmMMLowerPosition,
                AnalogOperation.ArmMMUpperPosition,
                DigitalOperation.IntakeDown,
                DigitalOperation.IntakeUp,
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
                new ChargeStationTask(false, Orientation.Forwards),
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
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
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
            MacroOperation.ChargeStationBalanceFacingBackwards,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_SELECT_BUTTON, // Left menu button
            Shift.Test1Debug,
            Shift.Test1Debug,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new ResetLevelTask(),
                new ChargeStationTask(false, Orientation.Backwards),
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
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
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
            UserInputDeviceButton.XBONE_START_BUTTON, // right menu button
            Shift.Test1Debug,
            Shift.None,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new ResetLevelTask(), // calibration
                new ChargeStationTask(true, Orientation.Forwards),
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
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
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
            MacroOperation.GoOverChargeStationTask,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_RIGHT_BUTTON, // right bumper
            Shift.Test1Debug,
            Shift.None,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new ResetLevelTask(), // calibration
                new GoOverChargeStationTask(false, false),
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
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
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
            MacroOperation.ChargeStationBalanceReverseFacingBackwards,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_START_BUTTON, // right menu button
            Shift.Test1Debug,
            Shift.Test1Debug,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new ResetLevelTask(), // calibration
                new ChargeStationTask(true, Orientation.Backwards),
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
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
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
                new FollowPathTask("LoadTaxi", Type.RobotRelativeFromCurrentPose)
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
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
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
                DigitalOperation.IntakeCone,
                DigitalOperation.IntakeCube,
                DigitalOperation.IntakeDown,
                DigitalOperation.IntakeUp,
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
                DigitalOperation.IntakeCone,
                DigitalOperation.IntakeCube,
                DigitalOperation.IntakeDown,
                DigitalOperation.IntakeUp,
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
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
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
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
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
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
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
            })
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
