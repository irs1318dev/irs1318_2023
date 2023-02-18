package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class IntakeExtendTask extends CompositeOperationTask
{
    private static final DigitalOperation[] possibleOperations =
        {
            DigitalOperation.IntakeExtend,
            DigitalOperation.IntakeRetract,
        };

    public IntakeExtendTask(boolean extend)
    {
        super(
            extend ? DigitalOperation.IntakeExtend : DigitalOperation.IntakeRetract,
            IntakeExtendTask.possibleOperations);
    }
}

