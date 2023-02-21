package frc.robot.driver;

public enum MacroOperation implements IOperation
{
    AutonomousRoutine,

    // Path testing
    FollowPathTest1,

    // Final Paths
    FollowPath2,
    FollowPath3,

    // DriveTrain operations:
    PIDLightBrake,
    PIDHeavyBrake,

    // Vision operations
    VisionCenterRetroReflective,
    VisionCenterAprilTag,
    VisionParallelAprilTag,

    // Charge station
    ChargeStationBalance,
    ChargeStationBalanceReverse,
    ChargeStationBalanceGyro,

    // Intake macros:
    IntakeGamePiece,

    // Arm Movement
    ArmResetToZero,
    ArmGroundPickupPosition,
    ArmGroundPlacePosition,
    ArmMiddleConePosition,
    ArmMiddleCubePosition,
    ArmHighConePosition,
    ArmHighCubePosition,
    ArmSubstationPickupPosition,
    ArmStowedPosition,
    ArmApproachPosition,
}
