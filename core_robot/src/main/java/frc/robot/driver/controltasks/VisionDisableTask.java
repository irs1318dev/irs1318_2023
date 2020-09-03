package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class VisionDisableTask extends CompositeOperationTask
{
    private static final DigitalOperation[] possibleOperations =
        {
            DigitalOperation.VisionForceDisable,
        };

    public VisionDisableTask()
    {
        super(1.0, DigitalOperation.VisionForceDisable, VisionDisableTask.possibleOperations);
    }

    @Override
    public boolean hasCompleted()
    {
        return false;
    }
}
