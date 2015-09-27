package org.usfirst.frc.team1318.robot.Driver;

/**
 * Interface describing a task that that controls the functioning of the robot.
 * 
 * @author Will
 *
 */
public interface IControlTask
{
    /**
     * Begin the current task.
     */
    public void begin();

    /**
     * Run an iteration of the current task.
     */
    public void update();

    /**
     * Stops the current task gracefully (but unexpectedly).
     */
    public void stop();

    /**
     * Ends the current task, called when it (or a master task) has completed.
     */
    public void end();

    /**
     * Checks whether this task has completed, or whether it should continue being processed.
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    public boolean hasCompleted();
}
