package frc.robot.driver;

public enum MacroOperation implements IOperation
{
    AutonomousRoutine,

    // Path testing
    FollowPathTest1,
    FollowPath2,
    FollowPath3,
    FollowPath4,

    // DriveTrain operations:
    PIDLightBrake,
    PIDHeavyBrake,

    // Vision operations
    VisionCenterRetroReflective,
    VisionCenterAprilTag,
    VisionParallelAprilTag,
    VisionResetPosition,

    // Charge station
    ChargeStationBalance,
    ChargeStationBalanceReverse,
    ChargeStationBalanceGyro,

    // Intake macros:
    IntakeGamePiece,

    // Cone flipepr macros:
    ExtendLeftConeFlipper,
    ExtendRightConeFlipper,

    // Arm Movement
    ArmResetToZero,
    ArmGroundPickupPosition,
    ArmGroundPlacePosition,
    ArmMiddleConePosition,
    ArmMiddleCubePosition,
    ArmHighConePosition,
    ArmHighCubePosition,
    ArmConeSubstationPickupPosition,
    ArmCubeSubstationPickupPosition,
    ArmStowedPosition,
    ArmApproachPosition,
}
