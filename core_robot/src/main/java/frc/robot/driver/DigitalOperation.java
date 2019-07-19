package frc.robot.driver;

public enum DigitalOperation implements IOperation
{
    // Vision operations:
    VisionForceDisable,
    VisionDisable,
    VisionEnable,
    VisionEnableOffboardStream,
    VisionEnableOffboardProcessing,

    // Compressor operations:
    CompressorForceDisable,

    // DriveTrain operations:
    DriveTrainEnablePID,
    DriveTrainDisablePID,
    DriveTrainSimpleMode,
    DriveTrainUseBrakeMode,
    DriveTrainUsePositionalMode,
    DriveTrainUseSimplePathMode,
    DriveTrainUsePathMode,
    DriveTrainSwapFrontOrientation,
}
