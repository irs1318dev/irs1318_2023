package frc.robot.driver;

public enum MacroOperation implements IOperation
{
    AutonomousRoutine,

    // Path testing
    FollowPathTest1,
    FollowPathTest2,

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

    //Arm Movement
    GroundPickup,
    GroundPlace,
    MiddleCone,
    MiddleCube,
    HighCone,
    HighCube,
    SubstationPickup,

}
