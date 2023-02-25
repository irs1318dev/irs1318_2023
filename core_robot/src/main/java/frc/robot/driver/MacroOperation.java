package frc.robot.driver;

public enum MacroOperation implements IOperation
{
    AutonomousRoutine,

    // DriveTrain operations:
    PIDLightBrake,
    PIDHeavyBrake,

    // Vision operations:
    VisionCenterRetroReflective,
    VisionCenterAprilTag,
    VisionParallelAprilTag,
    VisionResetPosition,

    // Charge station:
    ChargeStationBalance,
    ChargeStationBalanceReverse,
    ChargeStationBalanceGyro,

    // Intake macros:
    IntakeGamePiece,

    // Cone flipepr macros:
    ExtendLeftConeFlipper,
    ExtendRightConeFlipper,
    PickUpConeFromBehind,

    // Arm Movement:
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

    // Path testing:
    FollowPathTest1,
    FollowPathTest2,
    FollowPathTest3,
    FollowPathTest4,
}
