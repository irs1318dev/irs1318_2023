package org.usfirst.frc.team1318.robot.Autonomous.Tasks;

import java.util.Arrays;
import java.util.List;

import org.usfirst.frc.team1318.robot.Autonomous.AutonomousControlData;
import org.usfirst.frc.team1318.robot.Autonomous.IAutonomousTask;

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
public class ConcurrentTask implements IAutonomousTask
{
    private final boolean anyTask;
    private final List<IAutonomousTask> tasks;

    /**
     * Initializes a new CompositeTask
     * @param anyTask indicates that we want to use AnyTask semantics as opposed to AllTask semantics
     * @param tasks to run
     */
    private ConcurrentTask(boolean anyTask, IAutonomousTask... tasks)
    {
        this.anyTask = anyTask;
        this.tasks = Arrays.asList(tasks);
    }

    /**
     * Create a task that continues processing all of the provided tasks until any of them is ready to continue
     * @param tasks to run
     * @return a task that runs all of the provided tasks until all of them are ready to continue
     */
    public static IAutonomousTask AnyTasks(IAutonomousTask... tasks)
    {
        return new ConcurrentTask(true, tasks);
    }

    /**
     * Create a task that continues processing all of the provided tasks until all of them are ready to continue
     * @param tasks to run
     * @return a task that runs all of the provided tasks until one of them is ready to continue
     */
    public static IAutonomousTask AllTasks(IAutonomousTask... tasks)
    {
        return new ConcurrentTask(false, tasks);
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
            task.update(data);
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
     * @return true if we should continue on the current task, otherwise false (to move to the next task)
     */
    public boolean shouldContinueProcessingTask()
    {
        for (IAutonomousTask task : this.tasks)
        {
            boolean taskShouldContinueTask = task.shouldContinueProcessingTask();

            // for AnyTask tasks, return that we're ready to end (false) if any of them are ready to end (false).
            if (this.anyTask && !taskShouldContinueTask)
            {
                return false;
            }

            // for AllTask tasks, that we aren't ready to end (true) if any is not ready to end (true).
            if (!this.anyTask && taskShouldContinueTask)
            {
                return true;
            }
        }

        // AnyTasks return true when none of them are false.  AllTasks return false when none of them are true.
        return this.anyTask;
    }
}
