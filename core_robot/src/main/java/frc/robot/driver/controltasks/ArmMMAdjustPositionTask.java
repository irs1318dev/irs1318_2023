package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;

/**
 * Task that simply waits for a short period of time.
 * 
 */
public class ArmMMAdjustPositionTask extends TimedTask
{
    private final double lowerRate;
    private final double upperRate;

    /**
     * Initializes a new ArmMMAdjustPositionTask
     * @param duration to adjust in seconds
     * @param lowerRate the rate to adjust the lower (percentage of TuningConstants.ARM_LOWER_EXTENSION_ADJUSTMENT_VELOCITY)
     * @param upperRate the rate to adjust the upper (percentage of TuningConstants.ARM_UPPER_EXTENSION_ADJUSTMENT_VELOCITY)
     */
    public ArmMMAdjustPositionTask(double duration, double lowerRate, double upperRate)
    {
        super(duration);

        this.lowerRate = lowerRate;
        this.upperRate = upperRate;
    }

    /**
     * Run an iteration of the current task and apply any control changes 
     */
    @Override
    public void update()
    {
        this.setAnalogOperationState(AnalogOperation.ArmLowerPositionAdjustment, this.lowerRate);
        this.setAnalogOperationState(AnalogOperation.ArmUpperPositionAdjustment, this.upperRate);
    }

    @Override
    public void end()
    {
        this.setAnalogOperationState(AnalogOperation.ArmLowerPositionAdjustment, TuningConstants.ZERO);
        this.setAnalogOperationState(AnalogOperation.ArmUpperPositionAdjustment, TuningConstants.ZERO);
    }
}
