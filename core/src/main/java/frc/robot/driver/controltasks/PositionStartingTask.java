package frc.robot.driver.controltasks;

import frc.robot.driver.Operation;
import frc.robot.driver.common.IControlTask;

/**
 * Task that applies the starting angle
 * 
 */
public class PositionStartingTask extends TimedTask implements IControlTask
{
    private final double angle;

    /**
     * Initializes a new PositionStartingTask
     * @param angle to set
     */
    public PositionStartingTask(double angle)
    {
        super(1.0);

        this.angle = angle;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        super.begin();
        this.setAnalogOperationState(Operation.PositionStartingAngle, this.angle);
    }

    /**
     * Run an iteration of the current task and apply any control changes 
     */
    @Override
    public void update()
    {
        this.setAnalogOperationState(Operation.PositionStartingAngle, this.angle);
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        super.end();
        this.setAnalogOperationState(Operation.PositionStartingAngle, 0.0);
    }
}
