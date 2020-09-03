package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class CompressorDisableTask extends CompositeOperationTask
{
    private static final DigitalOperation[] possibleOperations =
        {
            DigitalOperation.CompressorForceDisable,
        };

    public CompressorDisableTask()
    {
        super(1.0, DigitalOperation.CompressorForceDisable, CompressorDisableTask.possibleOperations);
    }

    @Override
    public boolean hasCompleted()
    {
        return false;
    }
}
