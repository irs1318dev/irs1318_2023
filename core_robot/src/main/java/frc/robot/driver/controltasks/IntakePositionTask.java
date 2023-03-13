package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class IntakePositionTask extends CompositeOperationTask
{
    private static final DigitalOperation[] possibleOperations =
        {
            DigitalOperation.IntakeDown,
            DigitalOperation.IntakeUp,
        };

    /**
     * Task to raise/lwoer the intake pneumatics
     * @param down true to put the intake down, or false to put the intake up
     */
    public IntakePositionTask(boolean down)
    {
        super(
            down ? DigitalOperation.IntakeDown : DigitalOperation.IntakeUp,
            IntakePositionTask.possibleOperations);
    }
}

