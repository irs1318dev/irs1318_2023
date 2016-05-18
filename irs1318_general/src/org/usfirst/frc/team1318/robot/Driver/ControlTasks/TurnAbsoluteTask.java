package org.usfirst.frc.team1318.robot.Driver.ControlTasks;

import org.usfirst.frc.team1318.robot.Driver.Operation;
import org.usfirst.frc.team1318.robot.General.PositionManager;

/**
 * Task that turns the robot a certain amount clockwise or counterclockwise in-place.
 * This task turns at a certain speed based on the PositionManager to turn until it is within the acceptable
 * error from the orientation (relative to our starting position) that we want to have.
 * 
 */
public class TurnAbsoluteTask extends TimedTask
{
    private final double xVelocity;
    private final double absoluteDegrees;
    private final double acceptableError;
    private final PositionManager position;

    /**
     * Initializes a new TurnAbsoluteTask
     * @param maxDuration to perform the task in seconds
     * @param xVelocity to turn in the appropriate direction
     * @param absoluteDegrees indicates the direction we want to face when we are done turning.
     * @param acceptableError indicates how far off we find acceptable
     * @param position manager that can be used to calculate the current direction we are facing
     */
    public TurnAbsoluteTask(double maxDuration, double xVelocity, double absoluteDegrees, double acceptableError, PositionManager position)
    {
        super(maxDuration);

        this.xVelocity = xVelocity;
        this.absoluteDegrees = absoluteDegrees;
        this.acceptableError = acceptableError;
        this.position = position;
    }

    /**
     * Run an iteration of the current task and apply any control changes
     */
    @Override
    public void update()
    {
        double xVelocity = 0;
        double currentAngle = this.position.getOdometryAngle();

        double currentError = this.absoluteDegrees - currentAngle;
        if (currentError < -this.acceptableError)
        {
            xVelocity = -this.xVelocity;
        }
        else if (currentError > this.acceptableError)
        {
            xVelocity = this.xVelocity;
        }

        this.setDigitalOperationState(Operation.DriveTrainUsePositionalMode, false);
        this.setAnalogOperationState(Operation.DriveTrainMoveForward, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainTurn, xVelocity);
    }

    /**
     * Cancel the current task and clear control changes
     */
    @Override
    public void stop()
    {
        super.stop();

        this.setAnalogOperationState(Operation.DriveTrainMoveForward, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainTurn, 0.0);
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        super.end();

        this.setAnalogOperationState(Operation.DriveTrainMoveForward, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainTurn, 0.0);
    }

    /**
     * Checks whether this task has completed, or whether it should continue being processed
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        if (super.hasCompleted())
        {
            return true;
        }

        double currentAngle = this.position.getOdometryAngle();
        double currentError = this.absoluteDegrees - currentAngle;
        return currentError > -this.acceptableError && currentError < this.acceptableError;
    }
}
