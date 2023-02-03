package frc.robot.driver.controltasks;

import frc.robot.driver.*;

/**
 * Task that resets the robot's pitch to be 0
 * 
 */
public class PitchResetTask extends UpdateCycleTask
{
    /**
     * Initializes a new PitchResetTask
     */
    public PitchResetTask()
    {
        super(1);
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        super.begin();

        this.setDigitalOperationState(DigitalOperation.PositionResetRobotPitch, true);
    }

    /**
     * Run an iteration of the current task and apply any control changes 
     */
    @Override
    public void update()
    {
        super.update();

        this.setDigitalOperationState(DigitalOperation.PositionResetRobotPitch, true);
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        super.end();

        this.setDigitalOperationState(DigitalOperation.PositionResetRobotPitch, false);
    }
}
