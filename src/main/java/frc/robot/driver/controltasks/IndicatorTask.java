package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class IndicatorTask extends CompositeOperationTask
{
    private static final DigitalOperation[] Operations = new DigitalOperation[] { DigitalOperation.SubstationIntakeReady, DigitalOperation.ForcePurpleStrobe, DigitalOperation.ForceRainbow };

    public IndicatorTask(double duration, DigitalOperation operation)
    {
        super(operation, IndicatorTask.Operations, duration);
    }

    public IndicatorTask(DigitalOperation operation)
    {
        super(operation, IndicatorTask.Operations, true);
    }
}
