package org.usfirst.frc.team1318.robot.Autonomous.Tasks;

import org.usfirst.frc.team1318.robot.Autonomous.AutonomousControlData;
import org.usfirst.frc.team1318.robot.Autonomous.IAutonomousTask;

/**
 * Simple drive-forward task
 * 
 * @author Caroline
 *
 */
public class DriveForwardTask implements IAutonomousTask
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
     * @param data to which we should apply updated settings
     */
    @Override
    public void update(AutonomousControlData data)
    {
        data.setDriveTrainPositionMode(false);
        data.setDriveTrainYVelocity(.1);
    }

    /**
     * Cancel the current task and clear control changes
     * @param data to which we should clear any updated control settings
     */
    @Override
    public void cancel(AutonomousControlData data)
    {
        data.setDriveTrainYVelocity(0);
    }

    /**
     * End the current task and reset control changes appropriately
     * @param data to which we should apply updated settings
     */
    @Override
    public void end(AutonomousControlData data)
    {
        data.setDriveTrainYVelocity(0);
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
