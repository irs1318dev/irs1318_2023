package org.usfirst.frc.team1318.robot.Autonomous.Tasks;

import org.usfirst.frc.team1318.robot.Autonomous.AutonomousControlData;
import org.usfirst.frc.team1318.robot.Autonomous.IAutonomousTask;

import edu.wpi.first.wpilibj.Timer;

/**
 * Abstract class defining a task that lasts only for a certain duration.
 * 
 * @author Will
 *
 */
public abstract class TimedAutonomousTask implements IAutonomousTask
{
    private final double duration;

    private final Timer timer;
    private Double startTime;

    /**
     * Initializes a new TimedAutonomousTask
     * @param duration to perform the task in seconds
     */
    protected TimedAutonomousTask(double duration)
    {
        this.duration = duration;
        this.timer = new Timer();

        this.startTime = null;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.timer.start();
        this.startTime = this.timer.get();
    }

    /**
     * Run an iteration of the current task and apply any control changes 
     * @param data to which we should apply updated settings
     */
    public abstract void update(AutonomousControlData data);

    /**
     * Cancel the current task and clear control changes
     * @param data to which we should clear any updated control settings
     */
    @Override
    public void cancel(AutonomousControlData data)
    {
        this.startTime = null;
    }

    /**
     * End the current task and reset control changes appropriately
     * @param data to which we should apply updated settings
     */
    @Override
    public void end(AutonomousControlData data)
    {
    }

    /**
     * Checks whether this task has completed, or whether it should continue being processed
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        return this.timer.get() >= this.startTime + this.duration;
    }
}
