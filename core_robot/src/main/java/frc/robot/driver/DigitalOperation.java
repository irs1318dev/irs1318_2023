package frc.robot.driver;

public enum DigitalOperation implements IOperation
{
    PositionResetFieldOrientation,
    PositionResetRobotLevel,

    // Driver interaction operations
    ForceLightDriverRumble,
    CubeWantedFromSubstation,
    ConeWantedFromSubstation,
    RainbowTest, // for testing
    //FireFlame, // for testing
    PurpleTest, // for testing
    RedTest, // for testing
    BlueTest, // for testing
    GreenTest, // for testing
    YellowTest, // for testing

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
    DriveTrainIgnoreSlewRateLimitingMode,

    // Arm operations
    IntakeDown,
    IntakeUp,
    IntakeCube,
    IntakeCone,
    OutakeCubeFast,
    ExtendLeftConeFlipper,
    ExtendRightConeFlipper,
    
    ArmUpperExtend,
    ArmLowerExtend,
    ArmEnableSimpleMode,
    ArmDisableSimpleMode,
    ArmForceReset,
}
