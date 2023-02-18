package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.driver.*;
import frc.robot.mechanisms.*;

/**
 * Task that sets the Arm to the desired position using IK
 * 
 */
public class ArmIKPositionTask extends UpdateCycleTask
{
    private final double xPosition;
    private final double zPosition;
    private final boolean waitUntilPositionReached;

    private ArmMechanism arm;

    private enum armState{
        Completed,
        DesiredIntermidate,
        DesiredGoal
    }

    private armState currentArmState;

    public ArmIKPositionTask(double xPosition, double zPosition)
    {
        this(xPosition, zPosition, false);
    }

    public ArmIKPositionTask(double xPosition, double zPosition, boolean waitUntilPositionReached)
    {
        super(1);

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
        super.begin();

        this.arm = this.getInjector().getInstance(ArmMechanism.class);
        boolean currentIsIn = (this.arm.getFKXPosition() < TuningConstants.ARM_X_IK_IN_TRESHOLD && this.arm.getFKZPosition() < TuningConstants.ARM_Z_IK_IN_TRESHOLD);
        boolean goalIsIn = (this.xPosition < TuningConstants.ARM_X_IK_IN_TRESHOLD && this.zPosition < TuningConstants.ARM_Z_IK_IN_TRESHOLD);
        if(currentIsIn != goalIsIn)
        {
            this.currentArmState = armState.DesiredIntermidate;
        }
        else
        {
            this.currentArmState = armState.DesiredGoal;
        }
    }

    /**
     * Run an iteration of the current task and apply any control changes 
     */
    @Override
    public void update()
    {
        super.update();

        if(this.currentArmState == armState.DesiredIntermidate)
        {
            if( Math.abs(this.arm.getFKXPosition() - TuningConstants.ARM_X_IK_INTERMIDATE) < TuningConstants.ARM_X_IK_GOAL_THRESHOLD &&
                Math.abs(this.arm.getMMUpperPosition() - TuningConstants.ARM_Z_IK_INTERMIDATE) < TuningConstants.ARM_Z_IK_GOAL_THRESHOLD )
                {
                    this.currentArmState = armState.DesiredGoal;
                }
        }

        else if(this.currentArmState == armState.DesiredGoal)
        {
            if( Math.abs(this.arm.getFKXPosition() - this.xPosition) < TuningConstants.ARM_X_IK_GOAL_THRESHOLD &&
                Math.abs(this.arm.getFKZPosition() - this.zPosition) < TuningConstants.ARM_Z_IK_GOAL_THRESHOLD)
            {
                this.currentArmState = armState.Completed;
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
        super.end();

        this.setAnalogOperationState(AnalogOperation.ArmIKXPosition, TuningConstants.MAGIC_NULL_VALUE);
        this.setAnalogOperationState(AnalogOperation.ArmIKZPosition, TuningConstants.MAGIC_NULL_VALUE);
    }

    @Override
    public boolean hasCompleted()
    {
        if (!this.waitUntilPositionReached)
        {
            return super.hasCompleted();
        }

        if (Math.abs(this.arm.getFKXPosition() - this.xPosition) < TuningConstants.ARM_X_IK_GOAL_THRESHOLD &&
            Math.abs(this.arm.getFKZPosition() - this.zPosition) < TuningConstants.ARM_Z_IK_GOAL_THRESHOLD)
        {
            return true;
        }

        return false;
    }
}
