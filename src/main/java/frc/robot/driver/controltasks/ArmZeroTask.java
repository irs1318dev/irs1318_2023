package frc.robot.driver.controltasks;

import frc.lib.robotprovider.ITimer;
import frc.robot.TuningConstants;
import frc.robot.driver.*;
import frc.robot.mechanisms.*;

/**
 * Task that returns the arm to its default, fully-retracted position
 * 
 */
public class ArmZeroTask extends ControlTaskBase
{
    // Note: we retract lower arm before upper arm to make sure that we don't crunch the intake
    private enum ArmZeroState
    {
        RetractLowerArm,
        RetractUpperArm,
        Reset,
        Completed;
    }

    private ArmMechanism arm;
    private ITimer timer;

    private ArmZeroState state;
    private double transitionTime;

    public ArmZeroTask()
    {
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.arm = this.getInjector().getInstance(ArmMechanism.class);
        this.timer = this.getInjector().getInstance(ITimer.class);

        if (this.arm.getInSimpleMode())
        {
            this.state = ArmZeroState.Completed;
        }
        else
        {
            this.state = ArmZeroState.RetractLowerArm;
            this.transitionTime = timer.get();
        }
    }

    /**
     * Run an iteration of the current task and apply any control changes 
     */
    @Override
    public void update()
    {
        double currTime = this.timer.get();
        if (this.state == ArmZeroState.RetractLowerArm)
        {
            if (currTime >= this.transitionTime + TuningConstants.ARM_POWER_TRACKING_DURATION &&
                ((this.arm.getLowerLAsStalled() ||
                    (this.arm.getLowerLeftLAPowerAverage() <= TuningConstants.ARM_NOT_MOVING_POWER_THRESHOLD ||
                    this.arm.getLowerRightLAPowerAverage() <= TuningConstants.ARM_NOT_MOVING_POWER_THRESHOLD)) ||
                currTime >= this.transitionTime + 2.5))
            {
                this.state = ArmZeroState.RetractUpperArm;
                this.transitionTime = currTime;
            }
        }
        else if (this.state == ArmZeroState.RetractUpperArm)
        {
            if (currTime >= this.transitionTime + TuningConstants.ARM_POWER_TRACKING_DURATION &&
                ((this.arm.getUpperLAsStalled() ||
                    this.arm.getUpperLAsPowerAverage() <= TuningConstants.ARM_NOT_MOVING_POWER_THRESHOLD) ||
                currTime >= this.transitionTime + 2.0))
            {
                this.state = ArmZeroState.Reset;
            }
        }
        else if (this.state == ArmZeroState.Reset)
        {
            this.state = ArmZeroState.Completed;
        }

        switch (this.state)
        {
            case RetractLowerArm:
                this.setAnalogOperationState(AnalogOperation.ArmMMLowerPosition, TuningConstants.ARM_LOWER_ZEROING_POSITION);
                this.setAnalogOperationState(AnalogOperation.ArmMMUpperPosition, TuningConstants.MAGIC_NULL_VALUE);
                this.setDigitalOperationState(DigitalOperation.ArmForceReset, false);
                break;

            case RetractUpperArm:
                this.setAnalogOperationState(AnalogOperation.ArmMMLowerPosition, TuningConstants.ARM_LOWER_ZEROING_POSITION);
                this.setAnalogOperationState(AnalogOperation.ArmMMUpperPosition, TuningConstants.ARM_UPPER_ZEROING_POSITION);
                this.setDigitalOperationState(DigitalOperation.ArmForceReset, false);
                break;

            case Reset:
                this.setAnalogOperationState(AnalogOperation.ArmMMLowerPosition, TuningConstants.MAGIC_NULL_VALUE);
                this.setAnalogOperationState(AnalogOperation.ArmMMUpperPosition, TuningConstants.MAGIC_NULL_VALUE);
                this.setDigitalOperationState(DigitalOperation.ArmForceReset, true);
                break;

            default:
            case Completed:
                this.setAnalogOperationState(AnalogOperation.ArmMMLowerPosition, TuningConstants.MAGIC_NULL_VALUE);
                this.setAnalogOperationState(AnalogOperation.ArmMMUpperPosition, TuningConstants.MAGIC_NULL_VALUE);
                this.setDigitalOperationState(DigitalOperation.ArmForceReset, false);
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
        this.setDigitalOperationState(DigitalOperation.ArmForceReset, false);
    }

    @Override
    public boolean hasCompleted()
    {
        return this.state == ArmZeroState.Completed;
    }
}
