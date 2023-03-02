package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.driver.*;
import frc.robot.mechanisms.*;

/**
 * Task that sets the Arm to the desired position using MM
 * 
 */
public class ArmMMPositionTask extends ControlTaskBase
{
    private enum ArmMMState
    {
        DesiredIntermidate,
        DesiredGoal,
        Completed,
    }

    private final double lowerExtensionLength;
    private final double upperExtensionLength;
    private final boolean waitUntilPositionReached;

    private ArmMechanism arm;

    private ArmMMState currentArmState;

    public ArmMMPositionTask(double lowerExtensionLength, double upperExtensionLength)
    {
        this(lowerExtensionLength, upperExtensionLength, false);
    }

    public ArmMMPositionTask(double lowerExtensionLength, double upperExtensionLength, boolean waitUntilPositionReached)
    {
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
        this.arm = this.getInjector().getInstance(ArmMechanism.class);

        if (this.arm.getInSimpleMode())
        {
            this.currentArmState = ArmMMState.Completed;
            return;
        }

        boolean currentIsIn = (this.arm.getMMLowerPosition() > TuningConstants.ARM_LOWER_MM_IN_TRESHOLD && this.arm.getMMUpperPosition() < TuningConstants.ARM_UPPER_MM_IN_TRESHOLD);
        boolean goalIsIn = (this.lowerExtensionLength > TuningConstants.ARM_LOWER_MM_IN_TRESHOLD && this.upperExtensionLength < TuningConstants.ARM_UPPER_MM_IN_TRESHOLD);
        if (currentIsIn != goalIsIn)
        {
            this.currentArmState = ArmMMState.DesiredIntermidate;
        }
        else
        {
            this.currentArmState = ArmMMState.DesiredGoal;
        }
    }

    /**
     * Run an iteration of the current task and apply any control changes
     */
    @Override
    public void update()
    {
        if (this.currentArmState == ArmMMState.DesiredIntermidate)
        {
            if (Math.abs(this.arm.getMMLowerPosition() - TuningConstants.ARM_LOWER_MM_INTERMIDATE) < TuningConstants.ARM_LOWER_MM_GOAL_THRESHOLD &&
                Math.abs(this.arm.getMMUpperPosition() - TuningConstants.ARM_UPPER_MM_INTERMIDATE) < TuningConstants.ARM_UPPER_MM_GOAL_THRESHOLD)
            {
                this.currentArmState = ArmMMState.DesiredGoal;
            }
        }
        else if (this.currentArmState == ArmMMState.DesiredGoal)
        {
            if (Math.abs(this.arm.getMMLowerPosition() - this.lowerExtensionLength) < TuningConstants.ARM_LOWER_MM_GOAL_THRESHOLD &&
                Math.abs(this.arm.getMMUpperPosition() - this.upperExtensionLength) < TuningConstants.ARM_UPPER_MM_GOAL_THRESHOLD)
            {
                this.currentArmState = ArmMMState.Completed;
            }
            else if (!this.waitUntilPositionReached)
            {
                this.currentArmState = ArmMMState.Completed;
            }
        }

        switch (this.currentArmState)
        {
            case DesiredIntermidate:
                this.setAnalogOperationState(AnalogOperation.ArmMMLowerPosition, TuningConstants.ARM_LOWER_MM_INTERMIDATE);
                this.setAnalogOperationState(AnalogOperation.ArmMMUpperPosition, TuningConstants.ARM_UPPER_MM_INTERMIDATE);
                break;

            default:
            case Completed:
            case DesiredGoal:
                this.setAnalogOperationState(AnalogOperation.ArmMMLowerPosition, this.lowerExtensionLength);
                this.setAnalogOperationState(AnalogOperation.ArmMMUpperPosition, this.upperExtensionLength);
                break;
        }
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        this.setAnalogOperationState(AnalogOperation.ArmMMLowerPosition, TuningConstants.MAGIC_NULL_VALUE);
        this.setAnalogOperationState(AnalogOperation.ArmMMUpperPosition, TuningConstants.MAGIC_NULL_VALUE);
    }

    @Override
    public boolean hasCompleted()
    {
        return this.currentArmState == ArmMMState.Completed;
    }
}
