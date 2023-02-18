package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.driver.*;
import frc.robot.mechanisms.*;

/**
 * Task that sets the Arm to the desired position using MM
 * 
 */
public class ArmMMPositionTask extends UpdateCycleTask
{
    private final double lowerExtensionLength;
    private final double upperExtensionLength;
    private final boolean waitUntilPositionReached;

    private enum armState{
        Completed,
        DesiredIntermidate,
        DesiredGoal
    }

    private armState currentArmState;

    private ArmMechanism arm;

    public ArmMMPositionTask(double lowerExtensionLength, double upperExtensionLength)
    {
        this(lowerExtensionLength, upperExtensionLength, false);
    }

    public ArmMMPositionTask(double lowerExtensionLength, double upperExtensionLength, boolean waitUntilPositionReached)
    {
        super(1);

        this.lowerExtensionLength = lowerExtensionLength;
        this.upperExtensionLength = upperExtensionLength;
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
        boolean currentIsIn = (this.arm.getMMLowerPosition() < TuningConstants.ARM_LOWER_MM_IN_TRESHOLD && this.arm.getMMUpperPosition() < TuningConstants.ARM_UPPER_MM_IN_TRESHOLD);
        boolean goalIsIn = (this.lowerExtensionLength < TuningConstants.ARM_LOWER_MM_IN_TRESHOLD && this.upperExtensionLength < TuningConstants.ARM_UPPER_MM_IN_TRESHOLD);
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
            if( Math.abs(this.arm.getMMLowerPosition() - TuningConstants.ARM_LOWER_MM_INTERMIDATE) < TuningConstants.ARM_LOWER_MM_GOAL_THRESHOLD &&
                Math.abs(this.arm.getMMUpperPosition() - TuningConstants.ARM_UPPER_MM_INTERMIDATE) < TuningConstants.ARM_UPPER_MM_GOAL_THRESHOLD )
                {
                    this.currentArmState = armState.DesiredGoal;
                }
        }

        else if(this.currentArmState == armState.DesiredGoal)
        {
            if( Math.abs(this.arm.getMMLowerPosition() - this.lowerExtensionLength) < TuningConstants.ARM_LOWER_MM_GOAL_THRESHOLD &&
                Math.abs(this.arm.getMMUpperPosition() - this.upperExtensionLength) < TuningConstants.ARM_UPPER_MM_GOAL_THRESHOLD)
            {
                this.currentArmState = armState.Completed;
            }
        }

        switch (this.currentArmState)
        {
            case DesiredIntermidate:
                this.setAnalogOperationState(AnalogOperation.ArmMMLowerPosition, TuningConstants.ARM_LOWER_MM_INTERMIDATE);
                this.setAnalogOperationState(AnalogOperation.ArmMMUpperPosition, TuningConstants.ARM_UPPER_MM_INTERMIDATE);
                break;
            
            case DesiredGoal:
                this.setAnalogOperationState(AnalogOperation.ArmMMLowerPosition, this.lowerExtensionLength);
                this.setAnalogOperationState(AnalogOperation.ArmMMUpperPosition, this.upperExtensionLength);
                break;
            
            default:
            case Completed:
                this.setAnalogOperationState(AnalogOperation.ArmMMLowerPosition, TuningConstants.MAGIC_NULL_VALUE);
                this.setAnalogOperationState(AnalogOperation.ArmMMUpperPosition, TuningConstants.MAGIC_NULL_VALUE);
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

        this.setAnalogOperationState(AnalogOperation.ArmMMLowerPosition, TuningConstants.MAGIC_NULL_VALUE);
        this.setAnalogOperationState(AnalogOperation.ArmMMUpperPosition, TuningConstants.MAGIC_NULL_VALUE);
    }

    @Override
    public boolean hasCompleted()
    {
        if (!this.waitUntilPositionReached)
        {
            return super.hasCompleted();
        }

        if (Math.abs(this.arm.getMMLowerPosition() - this.lowerExtensionLength) < TuningConstants.ARM_LOWER_MM_GOAL_THRESHOLD &&
            Math.abs(this.arm.getMMUpperPosition() - this.upperExtensionLength) < TuningConstants.ARM_UPPER_MM_GOAL_THRESHOLD)
        {
            return true;
        }

        return false;
    }
}
