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
    DriveTrainSlowMode,
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

    // Arm operations
    IntakeRelease,
    IntakeGrab,
    IntakeIn,
    IntakeOut,
    ExtendLeftConeFlipper,
    ExtendRightConeFlipper,
    ArmUpperExtend,
    ArmLowerExtend,
    ArmEnableSimpleMode,
    ArmDisableSimpleMode,
    ArmForceReset,

    // Indicator Light Manager
    CubeWantedFromSubstation,
    ConeWantedFromSubstation,
}
