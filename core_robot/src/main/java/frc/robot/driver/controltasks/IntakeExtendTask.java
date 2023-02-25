package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class IntakeExtendTask extends CompositeOperationTask
{
    private static final DigitalOperation[] possibleOperations =
        {
            DigitalOperation.IntakeRelease,
            DigitalOperation.IntakeGrab,
        };

    /**
     * Task to extend/retract (open/grab) the intake pneumatics
     * @param extend true to open, or false to close the intake
     */
    public IntakeExtendTask(boolean extend)
    {
        super(
            extend ? DigitalOperation.IntakeRelease : DigitalOperation.IntakeGrab,
            IntakeExtendTask.possibleOperations);
    }
}

