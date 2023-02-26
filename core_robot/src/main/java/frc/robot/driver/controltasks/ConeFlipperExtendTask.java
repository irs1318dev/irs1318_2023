package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class ConeFlipperExtendTask extends CompositeOperationTask
{
    private static final DigitalOperation[] possibleOperations =
        {
            DigitalOperation.ExtendLeftConeFlipper,
            DigitalOperation.ExtendRightConeFlipper,
        };

    private final boolean neverEnd;

    public ConeFlipperExtendTask(boolean left, boolean neverEnd)
    {
        super(
            left ? DigitalOperation.ExtendLeftConeFlipper : DigitalOperation.ExtendRightConeFlipper,
            ConeFlipperExtendTask.possibleOperations);

        this.neverEnd = true;
    }

    @Override
    public boolean hasCompleted()
    {
        return !this.neverEnd && super.hasCompleted();
    }
}

