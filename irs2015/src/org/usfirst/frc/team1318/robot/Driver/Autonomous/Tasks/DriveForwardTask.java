package org.usfirst.frc.team1318.robot.Driver.Autonomous.Tasks;

import org.usfirst.frc.team1318.robot.Driver.IControlTask;

/**
 * Simple drive-forward task
 *
 */
public class DriveForwardTask implements IControlTask
{
    public DriveForwardTask()
    {
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
     */
    @Override
    public void update()
    {
        //        data.setDriveTrainPositionMode(false);
        //        data.setDriveTrainYVelocity(.1);
    }

    /**
     * Cancel the current task and clear control changes
     */
    @Override
    public void stop()
    {
        //        data.setDriveTrainYVelocity(0);
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        //        data.setDriveTrainYVelocity(0);
    }

    /**
     * Checks whether this task has completed, or whether it should continue being processed
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        return false;
    }
}
