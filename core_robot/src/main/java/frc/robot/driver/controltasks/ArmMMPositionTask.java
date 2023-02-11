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


    private double lowerExtensionLength;
    private double upperExtensionLength;
    private ArmMechanism arm;
    
    public ArmMMPositionTask(double lowerExtensionLength, double upperExtensionLength)
    {
        super(1);
        this.lowerExtensionLength = lowerExtensionLength;
        this.upperExtensionLength = upperExtensionLength;
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
        this.setAnalogOperationState(AnalogOperation.ArmMMLowerPosition, this.lowerExtensionLength);
        this.setAnalogOperationState(AnalogOperation.ArmMMUpperPosition, this.upperExtensionLength);
        
        super.update();
        
    }

    /**
     * End the current task and reset control changes appropriately
     */

    @Override
    public void end()
    {
        super.end();
    }

    @Override
    public boolean hasCompleted()
    {
        if(Math.abs(this.arm.getMMLowerPosition() - this.lowerExtensionLength) < TuningConstants.LOWER_MIN_EXTENTION_THRESHOLD)
        {
            if(Math.abs(this.arm.getMMUpperPosition() - this.upperExtensionLength) < TuningConstants.UPPER_MIN_EXTENTION_THRESHOLD)
            {
                return true;
            }
        }
        return false;
    }
}
