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
        DesiredLowerIntermidate,
        DesiredHighIntermidate,
        DesiredGoal,
        Completed,
    }

    private enum IntakeState
    {
        IntakeUp,
        IntakeDown,
        Unchanged,
    }

    private final double lowerExtensionLength;
    private final double upperExtensionLength;
    private final boolean waitUntilPositionReached;

    private ArmMechanism arm;

    private ArmMMState currentArmState;
    private final IntakeState desiredState;

    public ArmMMPositionTask(double lowerExtensionLength, double upperExtensionLength)
    {
        this(lowerExtensionLength, upperExtensionLength, false);
    }

    public ArmMMPositionTask(double lowerExtensionLength, double upperExtensionLength, IntakeState state)
    {
        this(lowerExtensionLength, upperExtensionLength, false, IntakeState.Unchanged);

    }

    public ArmMMPositionTask(double lowerExtensionLength, double upperExtensionLength, boolean waitUntilPositionReached)
    {
        this(lowerExtensionLength, upperExtensionLength, waitUntilPositionReached, IntakeState.Unchanged);
    }
    public ArmMMPositionTask(double lowerExtensionLength, double upperExtensionLength, boolean waitUntilPositionReached, IntakeState state)
    {
        this.lowerExtensionLength = lowerExtensionLength;
        this.upperExtensionLength = upperExtensionLength;
        this.waitUntilPositionReached = waitUntilPositionReached;
        this.desiredState = state;
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

        double armLowerPosition = this.arm.getLowerPosition();
        double armUpperPosition = this.arm.getUpperPosition();

        boolean currentIsInside = (armLowerPosition > TuningConstants.ARM_LOWER_POSITION_INSIDE_TRESHOLD && armUpperPosition < TuningConstants.ARM_UPPER_POSITION_INSIDE_TRESHOLD);
        boolean goalIsInside = (this.lowerExtensionLength > TuningConstants.ARM_LOWER_POSITION_INSIDE_TRESHOLD && this.upperExtensionLength < TuningConstants.ARM_UPPER_POSITION_INSIDE_TRESHOLD);

        boolean currentIsHigh = (armLowerPosition < TuningConstants.ARM_LOWER_POSITION_HIGH_TRESHOLD && armUpperPosition > TuningConstants.ARM_UPPER_POSITION_HIGH_TRESHOLD);
        boolean goalIsHigh = (this.lowerExtensionLength < TuningConstants.ARM_LOWER_POSITION_HIGH_TRESHOLD && this.upperExtensionLength > TuningConstants.ARM_UPPER_POSITION_HIGH_TRESHOLD);
        if (goalIsHigh != currentIsHigh)
        {
            this.currentArmState = ArmMMState.DesiredHighIntermidate;
        }
        else if (goalIsInside != currentIsInside)
        {
            this.currentArmState = ArmMMState.DesiredLowerIntermidate;
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
        if (this.currentArmState == ArmMMState.DesiredLowerIntermidate)
        {
            if (Math.abs(this.arm.getLowerPosition() - TuningConstants.ARM_LOWER_POSITION_LOWER_INTERMIDATE) < TuningConstants.ARM_LOWER_MM_GOAL_THRESHOLD &&
                Math.abs(this.arm.getUpperPosition() - TuningConstants.ARM_UPPER_POSITION_LOWER_INTERMIDATE) < TuningConstants.ARM_UPPER_MM_GOAL_THRESHOLD)
            {
                this.currentArmState = ArmMMState.DesiredGoal;
            }
        }
        else if (this.currentArmState == ArmMMState.DesiredHighIntermidate)
        {
            if (Math.abs(this.arm.getLowerPosition() - TuningConstants.ARM_LOWER_POSITION_HIGH_INTERMIDATE) < TuningConstants.ARM_LOWER_MM_GOAL_THRESHOLD &&
                Math.abs(this.arm.getUpperPosition() - TuningConstants.ARM_UPPER_POSITION_HIGH_INTERMIDATE) < TuningConstants.ARM_UPPER_MM_GOAL_THRESHOLD)
            {
                this.currentArmState = ArmMMState.DesiredGoal;
            }
        }
        else if (this.currentArmState == ArmMMState.DesiredGoal)
        {
            if (Math.abs(this.arm.getLowerPosition() - this.lowerExtensionLength) < TuningConstants.ARM_LOWER_MM_GOAL_THRESHOLD &&
                Math.abs(this.arm.getUpperPosition() - this.upperExtensionLength) < TuningConstants.ARM_UPPER_MM_GOAL_THRESHOLD)
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
            case DesiredLowerIntermidate:
                this.setAnalogOperationState(AnalogOperation.ArmMMLowerPosition, TuningConstants.ARM_LOWER_POSITION_LOWER_INTERMIDATE);
                this.setAnalogOperationState(AnalogOperation.ArmMMUpperPosition, TuningConstants.ARM_UPPER_POSITION_LOWER_INTERMIDATE);
                break;

            case DesiredHighIntermidate:
                this.setAnalogOperationState(AnalogOperation.ArmMMLowerPosition, TuningConstants.ARM_LOWER_POSITION_HIGH_INTERMIDATE);
                this.setAnalogOperationState(AnalogOperation.ArmMMUpperPosition, TuningConstants.ARM_UPPER_POSITION_HIGH_INTERMIDATE);
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
