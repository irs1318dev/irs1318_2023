package org.usfirst.frc.team1318.robot.Driver.Macros;

import org.usfirst.frc.team1318.robot.Driver.IControlTask;

public abstract class MacroTask implements IControlTask
{
    /**
     * Begin the current task.
     */
    @Override
    public abstract void begin();

    /**
     * Run an iteration of the current task.
     */
    @Override
    public abstract void update();

    /**
     * Stops the current task gracefully (but unexpectedly).
     */
    @Override
    public abstract void stop();

    /**
     * Ends the current task, called when it (or a master task) has completed.
     */
    @Override
    public abstract void end();

    /**
     * Checks whether this task has completed, or whether it should continue being processed.
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public abstract boolean hasCompleted();
}
