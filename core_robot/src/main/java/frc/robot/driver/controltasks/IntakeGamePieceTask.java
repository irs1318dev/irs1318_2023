package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class IntakeGamePieceTask extends CompositeOperationTask
{
    private static final DigitalOperation[] possibleOperations =
        {
            DigitalOperation.IntakeCube,
            DigitalOperation.IntakeCone,
        };

    public IntakeGamePieceTask(boolean intakeCube)
    {
        super(
            intakeCube ? DigitalOperation.IntakeCube : DigitalOperation.IntakeCone,
            IntakeGamePieceTask.possibleOperations,
            true);
    }

    public IntakeGamePieceTask(boolean intakeCube, double timeout)
    {
        super(
            intakeCube ? DigitalOperation.IntakeCube : DigitalOperation.IntakeCone,
            IntakeGamePieceTask.possibleOperations,
            timeout);
    }
}

