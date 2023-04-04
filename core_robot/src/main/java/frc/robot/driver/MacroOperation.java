package frc.robot.driver;

import frc.lib.driver.IOperation;

public enum MacroOperation implements IOperation
{
    AutonomousRoutine,

    // DriveTrain operations:
    PIDLightBrake,
    PIDHeavyBrake,
    FaceForward,
    FaceBackward,

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
    ChargeStationBalanceFacingBackwards,
    ChargeStationBalanceReverseFacingBackwards,
    GoOverChargeStationTask,

    // Cone flipepr macros:
    PickUpConeFromBehind,

    // Arm Movement:
    ArmResetToZero,
    ArmGroundPickupPositionCube,
    ArmGroundPickupPositionCone,
    ArmGroundPlacePosition,
    ArmMiddleConePosition,
    ArmMiddleCubePosition,
    ArmHighConePositionUp,
    ArmHighConePositionDown,
    ArmHighCubePosition,
    ArmConeSubstationPickupPosition,
    ArmSubstationHover,
    // ArmCubeSingleSubstationPickupPosition,
    ArmStowedPosition,
    ArmApproachPosition,

    // Path testing:
    FollowPathTest1,
    FollowPathTest2,
    FollowPathTest3,
    FollowPathTest4,
}
