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

    // Arm operations;
    LowerArmPosition,
    UpperArmPosition,

    //Inverse Kinematics
    ArmIKXPosition,
    ArmIKYPosition,
    //Magic Motion
    ArmMMUpperPosition, 
    ArmMMLowerPosition,
    //Codriver Adjustment
    ArmLowerPositionAdjustment,
    ArmUpperPositionAdjustment,
    //Simple - if encoders break
    ArmSimpleForceUpper,
    ArmSimpleForceLower,
    

    LowerArmVelocity,
    
}
