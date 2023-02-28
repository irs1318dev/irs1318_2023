package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class ConeFlipperExtendTask extends CompositeOperationTask
{
    private static final DigitalOperation[] possibleOperations =
        {
            DigitalOperation.ExtendLeftConeFlipper,
            DigitalOperation.ExtendRightConeFlipper,
        };

    public ConeFlipperExtendTask(boolean left)
    {
        super(
            left ? DigitalOperation.ExtendLeftConeFlipper : DigitalOperation.ExtendRightConeFlipper,
            ConeFlipperExtendTask.possibleOperations,
            true);
    }

    public ConeFlipperExtendTask(boolean left, double timeout)
    {
        super(
            left ? DigitalOperation.ExtendLeftConeFlipper : DigitalOperation.ExtendRightConeFlipper,
            ConeFlipperExtendTask.possibleOperations,
            timeout);
    }
}

