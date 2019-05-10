package frc.robot.driver.controltasks;

import frc.robot.driver.Operation;

public class VisionDisableTask extends CompositeOperationTask
{
    private static final Operation[] possibleOperations =
        {
            Operation.VisionForceDisable,
        };
 
    public VisionDisableTask()
    {
        super(1.0, Operation.VisionForceDisable, VisionDisableTask.possibleOperations);
    }   

    @Override
    public boolean hasCompleted()
    {
        return false;
    }
}
