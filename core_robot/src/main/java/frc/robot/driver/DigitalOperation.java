package frc.robot.driver;

public enum DigitalOperation implements IOperation
{
    PositionResetFieldOrientation,
    PositionResetRobotPitch,

    // Driver interaction operations
    ForceLightDriverRumble,

    // Vision operations:
    VisionForceDisable,
    VisionDisableStream,
    VisionEnableAprilTagProcessing,
    VisionEnableRetroreflectiveProcessing,

    // Compressor operations:
    CompressorForceDisable,

    // DriveTrain operations:
    DriveTrainPathMode,
    DriveTrainSteerMode,
    DriveTrainMaintainPositionMode,
    DriveTrainReset,
    DriveTrainEnableFieldOrientation,
    DriveTrainDisableFieldOrientation,
    DriveTrainUseRobotOrientation,
    DriveTrainEnableMaintainDirectionMode,
    DriveTrainDisableMaintainDirectionMode,
    DriveTrainResetXYPosition,

    // Intake Operations
    IntakeExtend,
    IntakeRetract,
    IntakeIn,
    IntakeOut,

    // Arm operations
    ExtendLeftFlipper,
    ExtendRightFlipper,
    ArmUpperExtend,
    ArmLowerExtend,

    ArmEnableSimpleMode,
    ArmDisableSimpleMode,
}
