package frc.robot.driver;

public enum MacroOperation implements IOperation
{
    AutonomousRoutine,

    // DriveTrain operations:
    PIDBrake,
    TurnInPlaceLeft,
    TurnInPlaceRight,

    // Testing operations
    FollowSomePath,
    FollowAnotherPath,
    FollowADifferentPath,
}
