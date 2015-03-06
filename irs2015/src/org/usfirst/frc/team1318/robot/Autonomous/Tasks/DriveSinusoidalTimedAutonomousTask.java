package org.usfirst.frc.team1318.robot.Autonomous.Tasks;

import org.usfirst.frc.team1318.robot.Autonomous.AutonomousControlData;

public class DriveSinusoidalTimedAutonomousTask extends TimedAutonomousTask
{
    private final double yVelocity;
    private final double magnitude;
    private final double rightMult;

    /**
     * Initializes a new DriveSinusoidalTimedAutonomousTask
     * @param duration to perform the task in seconds
     * @param yVelocity to apply to the driveTrain
     * @param magnitude of the sine wave to use for X velocity
     * @param rightMulti multiplier to use for the right side
     */
    public DriveSinusoidalTimedAutonomousTask(double duration, double yVelocity, double magnitude, double rightMult)
    {
        super(duration);

        this.yVelocity = yVelocity;
        this.magnitude = magnitude;
        this.rightMult = rightMult;
    }

    /**
     * Run an iteration of the current task and apply any control changes 
     * @param data to which we should apply updated settings
     */
    @Override
    public void update(AutonomousControlData data)
    {
        // calculate how far through our duration we are (as a percentage)
        double percentageTime = (this.timer.get() - this.startTime) / this.duration;
        if (percentageTime > 1.0)
        {
            percentageTime = 1.0;
        }
        else if (percentageTime < 0.0)
        {
            percentageTime = 0.0;
        }

        // apply x velocity based on how far through our duration we are 
        double xVelocity = this.magnitude * Math.sin(2 * Math.PI * percentageTime);
        if (xVelocity > 0.0)
        {
            xVelocity *= this.rightMult;
        }

        data.setDriveTrainPositionMode(false);
        data.setDriveTrainXVelocity(xVelocity);
        data.setDriveTrainYVelocity(this.yVelocity);
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
}
