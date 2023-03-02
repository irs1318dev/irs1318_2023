package frc.robot.driver;

public enum MacroOperation implements IOperation
{
    AutonomousRoutine,

    // DriveTrain operations:
    PIDLightBrake,
    PIDHeavyBrake,

    // Vision operations:
    VisionDoubleSubstationLeft,
    VisionDoubleSubstationRight,
    VisionGridCube,
    VisionGridConeLeft,
    VisionGridConeRight,
    VisionResetPosition,

    // Charge station:
    ChargeStationBalance,
    ChargeStationBalanceReverse,

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
