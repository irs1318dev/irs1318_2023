package frc.robot.driver;

import frc.lib.driver.IOperation;

public enum AnalogOperation implements IOperation
{
    PositionStartingAngle,

    // DriveTrain operations:
    DriveTrainMoveForward,
    DriveTrainMoveRight,
    DriveTrainTurnAngleGoal,
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
    ArmIKXAdjustment, // minor adjustments for IK
    ArmIKZAdjustment, // minor adjustments for IK
    ArmLowerPositionAdjustment, // minor adjustments for LA position, or simple mode
    ArmUpperPositionAdjustment, // minor adjustments for LA position, or simple mode
    ArmTwistLeft, // amount to twist to the left
    ArmTwistRight, // amount to twist to the right
}
