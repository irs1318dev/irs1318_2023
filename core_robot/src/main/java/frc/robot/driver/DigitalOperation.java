package frc.robot.driver;

public enum DigitalOperation implements IOperation
{
    PositionResetFieldOrientation,
    PositionResetRobotLevel,

    // Driver interaction operations
    ForceLightDriverRumble,
    CubeWantedFromSubstation,
    ConeWantedFromSubstation,
    Rainbow, // for testing
    //FireFlame, // for testing
    Purple, // for testing
    Red, // for testing
    Blue, // for testing
    Green, // for testing
    Yellow, // for testing

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
}
