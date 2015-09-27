package org.usfirst.frc.team1318.robot.Driver.Autonomous.Tasks;

import org.usfirst.frc.team1318.robot.Driver.IControlTask;

/**
 * Autonomous task that simply waits for a short period of time.
 * 
 * @author Will
 *
 */
public class WaitAutonomousTask extends TimedAutonomousTask implements IControlTask
{
    /**
     * Initializes a new WaitAutonomousTask
     * @param duration to wait in seconds
     */
    public WaitAutonomousTask(double duration)
    {
        super(duration);
    }

    /**
     * Run an iteration of the current task and apply any control changes 
     */
    @Override
    public void update()
    {
        // no-op
    }
}
