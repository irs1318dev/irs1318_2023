package org.usfirst.frc.team1318.robot.Autonomous.Tasks;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.Queue;

import org.usfirst.frc.team1318.robot.Autonomous.AutonomousControlData;
import org.usfirst.frc.team1318.robot.Autonomous.IAutonomousTask;

/**
 * Autonomous task that holds multiple other tasks and executes them sequentially (in order).
 * 
 * @author Will
 *
 */
public class SequentialTask implements IAutonomousTask
{
    private final Queue<IAutonomousTask> autonomousTasks;
    private IAutonomousTask currentTask;

    /**
     * Initializes a new SequentialTask
     * @param tasks to run
     */
    public SequentialTask(IAutonomousTask[] tasks)
    {
        this.autonomousTasks = new LinkedList<IAutonomousTask>(Arrays.asList(tasks));
        this.currentTask = null;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
    }

    /**
     * Run an iteration of the current task and apply any control changes 
     * @param data to which we should apply updated settings
     */
    @Override
    public void update(AutonomousControlData data)
    {
        // check whether we should continue with the current task
        if (this.currentTask != null)
        {
            if (!this.currentTask.shouldContinueProcessingTask())
            {
                this.currentTask.end(data);
                this.currentTask = null;
            }
        }

        // if there's no current task, find the next one and start it (if any)
        if (this.currentTask == null)
        {
            this.currentTask = this.autonomousTasks.poll();

            // if there's no next task to run, then we are done
            if (this.currentTask == null)
            {
                return;
            }

            this.currentTask.begin();
        }

        // run the current task and apply the result to the control data
        this.currentTask.update(data);
    }

    /**
     * Cancel the current task and clear control changes
     * @param data to which we should clear any updated control settings
     */
    @Override
    public void cancel(AutonomousControlData data)
    {
        if (this.currentTask != null)
        {
            this.currentTask.cancel(data);
        }
    }

    /**
     * End the current task and reset control changes appropriately
     * @param data to which we should apply updated settings
     */
    @Override
    public void end(AutonomousControlData data)
    {
        if (this.currentTask != null)
        {
            this.currentTask.end(data);
        }
    }

    /**
     * Checks whether we should continue processing this task or whether it should end
     * @return true if we should continue on the current task, otherwise false (to move to the next task)
     */
    @Override
    public boolean shouldContinueProcessingTask()
    {
        return this.currentTask != null || !this.autonomousTasks.isEmpty();
    }
}
