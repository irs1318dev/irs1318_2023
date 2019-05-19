package frc.robot.driver.controltasks;

import frc.robot.driver.Operation;

public class CompressorDisableTask extends CompositeOperationTask
{
    private static final Operation[] possibleOperations =
        {
            Operation.CompressorForceDisable,
        };
 
    public CompressorDisableTask()
    {
        super(1.0, Operation.CompressorForceDisable, CompressorDisableTask.possibleOperations);
    }   

    @Override
    public boolean hasCompleted()
    {
        return false;
    }
}
