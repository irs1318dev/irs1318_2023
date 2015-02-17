package org.usfirst.frc.team1318.robot.Autonomous.Tasks;

import org.usfirst.frc.team1318.robot.Autonomous.AutonomousControlData;
import org.usfirst.frc.team1318.robot.Autonomous.IAutonomousTask;

/**
 * Autonomous task that simply waits for a short period of time.
 * 
 * @author Will
 *
 */
public class WaitAutonomousTask extends TimedAutonomousTask implements IAutonomousTask
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
     * @param data to which we should apply updated settings
     */
    @Override
    public void update(AutonomousControlData data)
    {
        // no-op
    }
}
