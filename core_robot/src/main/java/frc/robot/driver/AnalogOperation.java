package frc.robot.driver;

public enum AnalogOperation implements IOperation
{
    PositionStartingAngle,

    // DriveTrain operations:
    DriveTrainMoveForward,
    DriveTrainTurn,
    DriveTrainLeftPosition,
    DriveTrainRightPosition,
    DriveTrainLeftVelocity,
    DriveTrainRightVelocity,
    DriveTrainHeadingCorrection,
}
