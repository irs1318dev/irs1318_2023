package frc.robot.driver;

public enum Operation
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
    DriveTrainMoveForward,
    DriveTrainTurn,
    DriveTrainSimpleMode,
    DriveTrainUseBrakeMode,
    DriveTrainUsePositionalMode,
    DriveTrainUseSimplePathMode,
    DriveTrainUsePathMode,
    DriveTrainLeftPosition,
    DriveTrainRightPosition,
    DriveTrainLeftVelocity,
    DriveTrainRightVelocity,
    DriveTrainHeadingCorrection,
    DriveTrainSwapFrontOrientation,
    PositionStartingAngle,
}
