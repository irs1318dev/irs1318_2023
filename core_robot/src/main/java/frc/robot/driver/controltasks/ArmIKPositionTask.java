package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.driver.*;
import frc.robot.mechanisms.*;

/**
 * Task that sets the Arm to the desired position using IK
 * 
 */
public class ArmIKPositionTask extends ControlTaskBase
{
    private enum ArmIKState
    {
        DesiredIntermidate,
        DesiredGoal,
        Completed,
    }

    private final double xPosition;
    private final double zPosition;
    private final boolean waitUntilPositionReached;

    private ArmMechanism arm;

    private ArmIKState currentArmState;

    public ArmIKPositionTask(double xPosition, double zPosition)
    {
        this(xPosition, zPosition, false);
    }

    public ArmIKPositionTask(double xPosition, double zPosition, boolean waitUntilPositionReached)
    {
        this.xPosition = xPosition;
        this.zPosition = zPosition;
        this.waitUntilPositionReached = waitUntilPositionReached;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.arm = this.getInjector().getInstance(ArmMechanism.class);

        boolean currentIsIn = (this.arm.getFKXPosition() < TuningConstants.ARM_X_IK_IN_TRESHOLD && this.arm.getFKZPosition() < TuningConstants.ARM_Z_IK_IN_TRESHOLD);
        boolean goalIsIn = (this.xPosition < TuningConstants.ARM_X_IK_IN_TRESHOLD && this.zPosition < TuningConstants.ARM_Z_IK_IN_TRESHOLD);
        if (currentIsIn != goalIsIn)
        {
            this.currentArmState = ArmIKState.DesiredIntermidate;
        }
        else
        {
            this.currentArmState = ArmIKState.DesiredGoal;
        }
    }

    /**
     * Run an iteration of the current task and apply any control changes 
     */
    @Override
    public void update()
    {
        if (this.currentArmState == ArmIKState.DesiredIntermidate)
        {
            if (Math.abs(this.arm.getFKXPosition() - TuningConstants.ARM_X_IK_INTERMIDATE) < TuningConstants.ARM_X_IK_GOAL_THRESHOLD &&
                Math.abs(this.arm.getFKZPosition() - TuningConstants.ARM_Z_IK_INTERMIDATE) < TuningConstants.ARM_Z_IK_GOAL_THRESHOLD)
            {
                this.currentArmState = ArmIKState.DesiredGoal;
            }
        }
        else if (this.currentArmState == ArmIKState.DesiredGoal)
        {
            if (Math.abs(this.arm.getFKXPosition() - this.xPosition) < TuningConstants.ARM_X_IK_GOAL_THRESHOLD &&
                Math.abs(this.arm.getFKZPosition() - this.zPosition) < TuningConstants.ARM_Z_IK_GOAL_THRESHOLD)
            {
                this.currentArmState = ArmIKState.Completed;
            }
            else if (!this.waitUntilPositionReached)
            {
                this.currentArmState = ArmIKState.Completed;
            }
        }

        switch (this.currentArmState)
        {
            case DesiredIntermidate:
                this.setAnalogOperationState(AnalogOperation.ArmIKXPosition, TuningConstants.ARM_X_IK_INTERMIDATE);
                this.setAnalogOperationState(AnalogOperation.ArmIKZPosition, TuningConstants.ARM_Z_IK_INTERMIDATE);
                break;

            case DesiredGoal:
                this.setAnalogOperationState(AnalogOperation.ArmIKXPosition, this.xPosition);
                this.setAnalogOperationState(AnalogOperation.ArmIKZPosition, this.zPosition);
                break;

            default:
            case Completed:
                this.setAnalogOperationState(AnalogOperation.ArmIKXPosition, TuningConstants.MAGIC_NULL_VALUE);
                this.setAnalogOperationState(AnalogOperation.ArmIKZPosition, TuningConstants.MAGIC_NULL_VALUE);
                break;
        }
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        this.setAnalogOperationState(AnalogOperation.ArmIKXPosition, TuningConstants.MAGIC_NULL_VALUE);
        this.setAnalogOperationState(AnalogOperation.ArmIKZPosition, TuningConstants.MAGIC_NULL_VALUE);
    }

    @Override
    public boolean hasCompleted()
    {
        return this.currentArmState == ArmIKState.Completed;
    }
}
