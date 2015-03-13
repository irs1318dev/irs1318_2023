package org.usfirst.frc.team1318.robot.Autonomous.Tasks;

import org.usfirst.frc.team1318.robot.Autonomous.AutonomousControlData;
import org.usfirst.frc.team1318.robot.DriveTrain.PositionManager;

public class TurnAbsoluteTask extends TimedAutonomousTask
{
    private final double xVelocity;
    private final double absoluteDegrees;
    private final double acceptableError;
    private final PositionManager position;

    /**
     * Initializes a new DriveTimedAutonomousTask
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
     * @param data to which we should apply updated settings
     */
    @Override
    public void update(AutonomousControlData data)
    {
        double xVelocity = 0;
        double currentAngle = this.position.getAngle();

        double currentError = this.absoluteDegrees - currentAngle;
        if (currentError < -this.acceptableError)
        {
            xVelocity = -this.xVelocity;
        }
        else if (currentError > this.acceptableError)
        {
            xVelocity = this.xVelocity;
        }

        data.setDriveTrainPositionMode(false);
        data.setDriveTrainXVelocity(xVelocity);
        data.setDriveTrainYVelocity(0.0);
    }

    /**
     * Cancel the current task and clear control changes
     * @param data to which we should clear any updated control settings
     */
    @Override
    public void cancel(AutonomousControlData data)
    {
        super.cancel(data);

        data.setDriveTrainXVelocity(0.0);
        data.setDriveTrainYVelocity(0.0);
    }

    /**
     * End the current task and reset control changes appropriately
     * @param data to which we should apply updated settings
     */
    @Override
    public void end(AutonomousControlData data)
    {
        super.end(data);

        data.setDriveTrainXVelocity(0.0);
        data.setDriveTrainYVelocity(0.0);
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

        double currentAngle = this.position.getAngle();
        double currentError = this.absoluteDegrees - currentAngle;
        return currentError > -this.acceptableError && currentError < this.acceptableError;
    }
}
