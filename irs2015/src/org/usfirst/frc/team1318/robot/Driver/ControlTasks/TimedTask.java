package org.usfirst.frc.team1318.robot.Driver.ControlTasks;

import org.usfirst.frc.team1318.robot.Driver.IControlTask;

import edu.wpi.first.wpilibj.Timer;

/**
 * Abstract class defining a task that lasts only for a certain duration.
 * 
 */
public abstract class TimedTask extends ControlTaskBase implements IControlTask
{
    protected final double duration;
    protected final Timer timer;
    protected Double startTime;

    /**
     * Initializes a new TimedTask
     * @param duration to perform the task in seconds
     */
    protected TimedTask(double duration)
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
     */
    public abstract void update();

    /**
     * Cancel the current task and clear control changes
     */
    @Override
    public void stop()
    {
        this.startTime = null;
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
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
