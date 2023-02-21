package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class IntakeExtendTask extends CompositeOperationTask
{
    private static final DigitalOperation[] possibleOperations =
        {
            DigitalOperation.IntakeRelease,
            DigitalOperation.IntakeGrab,
        };

    public IntakeExtendTask(boolean extend)
    {
        super(
            extend ? DigitalOperation.IntakeRelease : DigitalOperation.IntakeGrab,
            IntakeExtendTask.possibleOperations);
    }
}

