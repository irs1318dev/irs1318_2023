package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class StrobeTask extends ControlTaskBase
{
    public StrobeTask()
    {
    }

    @Override
    public void begin()
    {
    }

    @Override
    public void update()
    {
        this.setDigitalOperationState(DigitalOperation.ForcePurpleStrobe, true);
    }

    @Override
    public void end()
    {
        this.setDigitalOperationState(DigitalOperation.ForcePurpleStrobe, false);
    }

    @Override
    public boolean hasCompleted()
    {
        return false;
    }
}
