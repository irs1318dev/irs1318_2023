package org.usfirst.frc.team1318.robot.Autonomous;

import java.util.Arrays;
import java.util.List;

/**
 * Autonomous task that holds multiple other tasks and executes them in parallel until certain conditions
 * are met.
 * 
 * AndTask indicates all tasks should be run until all tasks are considered "complete"
 * OrTask indicates that all tasks should be run until at least one of the tasks are considered "complete"
 * 
 * @author Will
 *
 */
public class CompositeTask implements IAutonomousTask
{
    private final boolean andTask;
    private List<IAutonomousTask> tasks;

    /**
     * Initializes a new CompositeTask
     * @param andTask indicates that we want to use AndTask semantics as opposed to OrTask semantics
     * @param tasks to run
     */
    private CompositeTask(boolean andTask, IAutonomousTask... tasks)
    {
        this.andTask = andTask;
        this.tasks = Arrays.asList(tasks);
    }

    /**
     * Create a task that combines all of the tasks and continues to run all of them until all are ready to continue
     * @param tasks to run
     * @return a task that runs all of the provided tasks until all of them are ready to continue
     */
    public static IAutonomousTask AndTasks(IAutonomousTask... tasks)
    {
        return new CompositeTask(true, tasks);
    }

    /**
     * Create a task that combines all of the tasks and continues to run all of them until one is ready to continue
     * @param tasks to run
     * @return a task that runs all of the provided tasks until one of them is ready to continue
     */
    public static IAutonomousTask OrTasks(IAutonomousTask... tasks)
    {
        return new CompositeTask(false, tasks);
    }

    /**
     * Begin the current task
     */
    public void begin()
    {
        for (IAutonomousTask task : this.tasks)
        {
            task.begin();
        }
    }

    /**
     * Run an iteration of the current task and apply any control changes 
     * @param data to which we should apply updated settings
     */
    public void update(AutonomousControlData data)
    {
        for (IAutonomousTask task : this.tasks)
        {
            task.begin();
        }
    }

    /**
     * Cancel the current task and clear control changes
     * @param data to which we should clear any updated control settings
     */
    public void cancel(AutonomousControlData data)
    {
        for (IAutonomousTask task : this.tasks)
        {
            task.cancel(data);
        }
    }

    /**
     * End the current task and reset control changes appropriately
     * @param data to which we should apply updated settings
     */
    public void end(AutonomousControlData data)
    {
        for (IAutonomousTask task : this.tasks)
        {
            task.end(data);
        }
    }

    /**
     * Checks whether we should continue processing this task or whether it should end
     * @return true if we should continue, otherwise false
     */
    public boolean shouldContinue()
    {
        for (IAutonomousTask task : this.tasks)
        {
            boolean taskShouldContinue = task.shouldContinue();

            // for AndTasks, return false if any of them is false
            if (this.andTask && !taskShouldContinue)
            {
                return false;
            }

            // for OrTasks, return true if any of them is true
            if (!this.andTask && taskShouldContinue)
            {
                return true;
            }
        }

        // AndTasks return true when none of them are false.  OrTasks return false when none of them are true.
        return this.andTask;
    }
}
