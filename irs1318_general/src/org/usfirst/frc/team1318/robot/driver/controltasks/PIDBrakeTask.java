package org.usfirst.frc.team1318.robot.driver.controltasks;

import org.usfirst.frc.team1318.robot.driver.Operation;
import org.usfirst.frc.team1318.robot.driver.common.IControlTask;

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
        this.desiredFinalLeftTicks = this.startLeftTicks;
        this.desiredFinalRightTicks = this.startRightTicks;
    }

    @Override
    public void begin()
    {
        super.begin();

        this.setDigitalOperationState(Operation.DriveTrainUseBrakeMode, true);
    }

    @Override
    public void end()
    {
        super.end();

        this.setDigitalOperationState(Operation.DriveTrainUseBrakeMode, false);
    }

    @Override
    public void stop()
    {
        super.stop();

        this.setDigitalOperationState(Operation.DriveTrainUseBrakeMode, false);
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
