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
}
