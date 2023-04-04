package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.driver.*;
import frc.robot.mechanisms.*;

/**
 * Task that sets the Arm to the desired position using MM
 * 
 */
public class ArmLAPositionTask extends ControlTaskBase
{
    private enum ArmMMState
    {
        DesiredLowerIntermidate,
        DesiredHighIntermidate,
        DesiredGoal,
        Completed,
    }

    public enum IntakeState
    {
        Up,
        Down,
        Unchanged,
    }

    private final IntakeState desiredState;

    private final double lowerExtensionLength;
    private final double upperExtensionLength;
    private final boolean waitUntilPositionReached;
    private final boolean useAutoBehavior;

    private ArmMechanism arm;

    private ArmMMState currentArmState;
    public double upperArmHighIntermediate;

    public ArmLAPositionTask(double lowerExtensionLength, double upperExtensionLength)
    {
        this(lowerExtensionLength, upperExtensionLength, false);
    }

    public ArmLAPositionTask(double lowerExtensionLength, double upperExtensionLength, IntakeState state)
    {
        this(lowerExtensionLength, upperExtensionLength, false, state, false);
    }

    public ArmLAPositionTask(double lowerExtensionLength, double upperExtensionLength, boolean waitUntilPositionReached)
    {
        this(lowerExtensionLength, upperExtensionLength, waitUntilPositionReached, IntakeState.Unchanged, false);
    }

    public ArmLAPositionTask(double lowerExtensionLength, double upperExtensionLength, boolean waitUntilPositionReached, IntakeState state, boolean useAutoBehavior)
    {
        this.lowerExtensionLength = lowerExtensionLength;
        this.upperExtensionLength = upperExtensionLength;
        this.waitUntilPositionReached = waitUntilPositionReached;
        this.desiredState = state;
        this.useAutoBehavior = useAutoBehavior;
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
            if (this.useAutoBehavior &&
                armUpperPosition > TuningConstants.ARM_UPPER_POSITION_HIGH_INTERMIDATE &&
                armUpperPosition < TuningConstants.ARM_USE_UPPER_POSITION_HIGH_INTERMIDATE_THRESHOLD)
            {
                this.upperArmHighIntermediate = armUpperPosition;
            }
            else
            {
                this.upperArmHighIntermediate = TuningConstants.ARM_UPPER_POSITION_HIGH_INTERMIDATE;
            }
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
                Math.abs(this.arm.getUpperPosition() - this.upperArmHighIntermediate) < TuningConstants.ARM_UPPER_MM_GOAL_THRESHOLD)
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

        if (this.currentArmState == ArmMMState.DesiredGoal)
        {
            switch (this.desiredState)
            {
                case Down:
                    this.setDigitalOperationState(DigitalOperation.IntakeDown, true);
                    this.setDigitalOperationState(DigitalOperation.IntakeUp, false);
                    break;

                case Up:
                    this.setDigitalOperationState(DigitalOperation.IntakeDown, false);
                    this.setDigitalOperationState(DigitalOperation.IntakeUp, true);
                    break;

                default:
                case Unchanged:
                    this.setDigitalOperationState(DigitalOperation.IntakeDown, false);
                    this.setDigitalOperationState(DigitalOperation.IntakeUp, false);
                    break;
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
                this.setAnalogOperationState(AnalogOperation.ArmMMUpperPosition, this.upperArmHighIntermediate);
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
        this.setDigitalOperationState(DigitalOperation.IntakeDown, false);
        this.setDigitalOperationState(DigitalOperation.IntakeUp, false);
    }

    @Override
    public boolean hasCompleted()
    {
        return this.currentArmState == ArmMMState.Completed;
    }
}
