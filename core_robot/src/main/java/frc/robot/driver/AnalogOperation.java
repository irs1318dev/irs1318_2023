package frc.robot.driver;

public enum AnalogOperation implements IOperation
{
    PositionStartingAngle,

    // DriveTrain operations:
    DriveTrainMoveForward,
    DriveTrainMoveRight,
    DriveTrainTurnAngleReference,
    DriveTrainTurnAngleGoal,
    DriveTrainTurnSpeed,
    DriveTrainSpinLeft,
    DriveTrainSpinRight,
    DriveTrainRotationA,
    DriveTrainRotationB,
    DriveTrainPathXGoal,
    DriveTrainPathYGoal,
    DriveTrainPathXVelocityGoal,
    DriveTrainPathYVelocityGoal,
    DriveTrainPathAngleVelocityGoal,
    DriveTrainPositionSteer1,
    DriveTrainPositionSteer2,
    DriveTrainPositionSteer3,
    DriveTrainPositionSteer4,
    DriveTrainPositionDrive1,
    DriveTrainPositionDrive2,
    DriveTrainPositionDrive3,
    DriveTrainPositionDrive4,
    DriveTrainStartingXPosition,
    DriveTrainStartingYPosition,

    // Arm operations;
    ArmIKXPosition, // Inverse Kinematics
    ArmIKZPosition, // Inverse Kinematics
    ArmMMUpperPosition, // Magic Motion
    ArmMMLowerPosition, // Magic Motion
    ArmLowerPositionAdjustment, // Codriver minor adjustment
    ArmUpperPositionAdjustment, // Codriver minor adjustment
    ArmSimpleForceLower, // Simple control - in case encoder(s) break
    ArmSimpleForceUpper, // Simple control - in case encoder(s) break
}
