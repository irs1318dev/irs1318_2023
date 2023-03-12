package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class IntakeGamePieceTask extends CompositeOperationTask
{
    private static final DigitalOperation[] possibleOperations =
        {
            DigitalOperation.IntakeCube,
            DigitalOperation.IntakeCone,
        };

    public IntakeGamePieceTask(boolean intakeIn)
    {
        super(
            intakeIn ? DigitalOperation.IntakeCube : DigitalOperation.IntakeCone,
            IntakeGamePieceTask.possibleOperations,
            true);
    }

    public IntakeGamePieceTask(boolean intakeIn, double timeout)
    {
        super(
            intakeIn ? DigitalOperation.IntakeCube : DigitalOperation.IntakeCone,
            IntakeGamePieceTask.possibleOperations,
            timeout);
    }
}

