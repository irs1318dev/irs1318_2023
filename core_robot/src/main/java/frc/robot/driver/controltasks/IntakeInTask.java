package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class IntakeInTask extends CompositeOperationTask
{
    private static final DigitalOperation[] possibleOperations =
        {
            DigitalOperation.IntakeIn,
            DigitalOperation.IntakeOut,
        };

    public IntakeInTask(boolean intakeIn)
    {
        super(
            intakeIn ? DigitalOperation.IntakeIn : DigitalOperation.IntakeOut,
            IntakeInTask.possibleOperations);
    }

    public IntakeInTask(boolean intakeIn, double timeout)
    {
        super(
            intakeIn ? DigitalOperation.IntakeIn : DigitalOperation.IntakeOut,
            IntakeInTask.possibleOperations,
            timeout);
    }
}

