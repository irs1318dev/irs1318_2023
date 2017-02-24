package org.usfirst.frc.team1318.robot.driver.controltasks;

import org.usfirst.frc.team1318.robot.driver.IControlTask;

public class PIDBrakeTask extends MoveDistanceOneShotTaskBase implements IControlTask
{
    /**
     * Initializes a new PIDBrakeTask
     */
    public PIDBrakeTask()
    {
        super(true);
    }

    /**
     * Determine the final encoder distance
     */
    @Override
    protected void determineFinalEncoderDistance()
    {
        this.desiredFinalLeftEncoderDistance = this.startLeftEncoderDistance;
        this.desiredFinalRightEncoderDistance = this.startRightEncoderDistance;
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
