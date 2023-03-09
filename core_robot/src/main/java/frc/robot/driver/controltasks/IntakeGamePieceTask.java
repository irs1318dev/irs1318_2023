package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class IntakeGamePieceTask extends CompositeOperationTask
{
    private static final DigitalOperation[] possibleOperations =
        {
            DigitalOperation.IntakeCube,
            DigitalOperation.IntakeCone,
        };

    public IntakeInTask(boolean intakeIn)
    {
        super(
            intakeIn ? DigitalOperation.IntakeCube : DigitalOperation.IntakeCone,
            IntakeInTask.possibleOperations,
            true);
    }

    public IntakeInTask(boolean intakeIn, double timeout)
    {
        super(
            intakeIn ? DigitalOperation.IntakeCube : DigitalOperation.IntakeCone,
            IntakeInTask.possibleOperations,
            timeout);
    }
}

