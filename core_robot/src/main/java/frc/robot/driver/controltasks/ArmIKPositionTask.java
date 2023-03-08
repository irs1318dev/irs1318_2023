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
        DesiredLowerIntermidate,
        DesiredHighIntermidate,
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

        double armXPos = this.arm.getFKXPosition();
        double armZPos = this.arm.getFKZPosition();

        boolean currentIsInside = (armXPos < TuningConstants.ARM_X_IK_INSIDE_TRESHOLD && armZPos < TuningConstants.ARM_Z_IK_INSIDE_TRESHOLD);
        boolean goalIsInside = (this.xPosition < TuningConstants.ARM_X_IK_INSIDE_TRESHOLD && this.zPosition < TuningConstants.ARM_Z_IK_INSIDE_TRESHOLD);

        boolean currentIsHigh = (armXPos > TuningConstants.ARM_X_IK_HIGH_TRESHOLD && armZPos > TuningConstants.ARM_Z_IK_HIGH_TRESHOLD);
        boolean goalIsHigh = (this.xPosition > TuningConstants.ARM_X_IK_HIGH_TRESHOLD && this.zPosition > TuningConstants.ARM_Z_IK_HIGH_TRESHOLD);
        if (currentIsHigh != goalIsHigh)
        {
            this.currentArmState = ArmIKState.DesiredHighIntermidate;
        }
        else if (currentIsInside != goalIsInside)
        {
            this.currentArmState = ArmIKState.DesiredLowerIntermidate;
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
        if (this.currentArmState == ArmIKState.DesiredLowerIntermidate)
        {
            if (Math.abs(this.arm.getFKXPosition() - TuningConstants.ARM_X_IK_LOWER_INTERMIDATE) < TuningConstants.ARM_X_IK_GOAL_THRESHOLD &&
                Math.abs(this.arm.getFKZPosition() - TuningConstants.ARM_Z_IK_LOWER_INTERMIDATE) < TuningConstants.ARM_Z_IK_GOAL_THRESHOLD)
            {
                this.currentArmState = ArmIKState.DesiredGoal;
            }
        }
        else if (this.currentArmState == ArmIKState.DesiredHighIntermidate)
        {
            if (Math.abs(this.arm.getFKXPosition() - TuningConstants.ARM_X_IK_HIGH_INTERMIDATE) < TuningConstants.ARM_X_IK_GOAL_THRESHOLD &&
                Math.abs(this.arm.getFKZPosition() - TuningConstants.ARM_Z_IK_HIGH_INTERMIDATE) < TuningConstants.ARM_Z_IK_GOAL_THRESHOLD)
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
            case DesiredLowerIntermidate:
                this.setAnalogOperationState(AnalogOperation.ArmIKXPosition, TuningConstants.ARM_X_IK_LOWER_INTERMIDATE);
                this.setAnalogOperationState(AnalogOperation.ArmIKZPosition, TuningConstants.ARM_Z_IK_LOWER_INTERMIDATE);
                break;

            case DesiredHighIntermidate:
                this.setAnalogOperationState(AnalogOperation.ArmIKXPosition, TuningConstants.ARM_X_IK_HIGH_INTERMIDATE);
                this.setAnalogOperationState(AnalogOperation.ArmIKZPosition, TuningConstants.ARM_Z_IK_HIGH_INTERMIDATE);
                break;

            default:
            case Completed:
            case DesiredGoal:
                this.setAnalogOperationState(AnalogOperation.ArmIKXPosition, this.xPosition);
                this.setAnalogOperationState(AnalogOperation.ArmIKZPosition, this.zPosition);
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
