package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.common.robotprovider.ITimer;
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

    private boolean useSimpleMode;
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

        this.useSimpleMode = this.arm.getInSimpleMode();
        this.state = ArmZeroState.RetractLowerArm;
        this.transitionTime = timer.get();

        this.setDigitalOperationState(DigitalOperation.ArmEnableSimpleMode, true);
        this.setDigitalOperationState(DigitalOperation.ArmDisableSimpleMode, false);
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
                this.arm.getLowerLeftLAPower() <= TuningConstants.ARM_NOT_MOVING_POWER_THRESHOLD &&
                this.arm.getLowerRightLAPower() <= TuningConstants.ARM_NOT_MOVING_POWER_THRESHOLD)
            {
                this.state = ArmZeroState.RetractUpperArm;
                this.transitionTime = currTime;
            }
        }
        else if (this.state == ArmZeroState.RetractUpperArm)
        {
            if (currTime >= this.transitionTime + TuningConstants.ARM_POWER_TRACKING_DURATION &&
                this.arm.getUpperLAPower() <= TuningConstants.ARM_NOT_MOVING_POWER_THRESHOLD)
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
                this.setAnalogOperationState(AnalogOperation.ArmSimpleForceLower, TuningConstants.ARM_MAX_FORWARD_SIMPLE_VELOCITY);
                this.setAnalogOperationState(AnalogOperation.ArmSimpleForceUpper, TuningConstants.ZERO);
                this.setDigitalOperationState(DigitalOperation.ArmEnableSimpleMode, true);
                this.setDigitalOperationState(DigitalOperation.ArmDisableSimpleMode, false);
                this.setDigitalOperationState(DigitalOperation.ArmForceReset, false);
                break;

            case RetractUpperArm:
                this.setAnalogOperationState(AnalogOperation.ArmSimpleForceLower, TuningConstants.ARM_MAX_FORWARD_SIMPLE_VELOCITY);
                this.setAnalogOperationState(AnalogOperation.ArmSimpleForceUpper, TuningConstants.ARM_MAX_REVERSE_SIMPLE_VELOCITY);
                this.setDigitalOperationState(DigitalOperation.ArmEnableSimpleMode, true);
                this.setDigitalOperationState(DigitalOperation.ArmDisableSimpleMode, false);
                this.setDigitalOperationState(DigitalOperation.ArmForceReset, false);
                break;

            case Reset:
                this.setAnalogOperationState(AnalogOperation.ArmSimpleForceLower, TuningConstants.ZERO);
                this.setAnalogOperationState(AnalogOperation.ArmSimpleForceUpper, TuningConstants.ZERO);
                this.setDigitalOperationState(DigitalOperation.ArmEnableSimpleMode, this.useSimpleMode);
                this.setDigitalOperationState(DigitalOperation.ArmDisableSimpleMode, !this.useSimpleMode);
                this.setDigitalOperationState(DigitalOperation.ArmForceReset, true);
                break;

            default:
            case Completed:
                this.setAnalogOperationState(AnalogOperation.ArmSimpleForceLower, TuningConstants.ZERO);
                this.setAnalogOperationState(AnalogOperation.ArmSimpleForceUpper, TuningConstants.ZERO);
                this.setDigitalOperationState(DigitalOperation.ArmEnableSimpleMode, false);
                this.setDigitalOperationState(DigitalOperation.ArmDisableSimpleMode, false);
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
        this.setAnalogOperationState(AnalogOperation.ArmSimpleForceLower, TuningConstants.ZERO);
        this.setAnalogOperationState(AnalogOperation.ArmSimpleForceUpper, TuningConstants.ZERO);
        this.setDigitalOperationState(DigitalOperation.ArmEnableSimpleMode, false);
        this.setDigitalOperationState(DigitalOperation.ArmDisableSimpleMode, false);
        this.setDigitalOperationState(DigitalOperation.ArmForceReset, false);
    }

    @Override
    public boolean hasCompleted()
    {
        return this.state == ArmZeroState.Completed;
    }
}
