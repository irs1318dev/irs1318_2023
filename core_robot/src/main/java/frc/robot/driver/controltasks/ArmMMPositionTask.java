package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.driver.*;
import frc.robot.mechanisms.*;

/**
 * Task that resets the robot's pitch to be 0
 * 
 */
public class ArmMMPositionTask extends UpdateCycleTask
{
    private final double lowerExtensionLength;
    private final double upperExtensionLength;
    private final boolean waitUntilPositionReached;

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
    }

    /**
     * Run an iteration of the current task and apply any control changes 
     */
    @Override
    public void update()
    {
        super.update();

        this.setAnalogOperationState(AnalogOperation.ArmMMLowerPosition, this.lowerExtensionLength);
        this.setAnalogOperationState(AnalogOperation.ArmMMUpperPosition, this.upperExtensionLength);
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

        if (Math.abs(this.arm.getMMLowerPosition() - this.lowerExtensionLength) < TuningConstants.ARM_LOWER_MIN_EXTENTION_THRESHOLD &&
            Math.abs(this.arm.getMMUpperPosition() - this.upperExtensionLength) < TuningConstants.ARM_UPPER_MIN_EXTENTION_THRESHOLD)
        {
            return true;
        }

        return false;
    }
}
