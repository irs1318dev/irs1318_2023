package org.usfirst.frc.team1318.robot.Autonomous.Tasks;

import org.usfirst.frc.team1318.robot.Autonomous.AutonomousControlData;
import org.usfirst.frc.team1318.robot.DriveTrain.DriveTrainComponent;
import org.usfirst.frc.team1318.robot.DriveTrain.PositionManager;

public class DriveSinusoidalTimedWithAngleTask extends TimedAutonomousTask
{
    private final double yVelocity;
    private final double magnitude;
    private final double rightMult;
    private PositionManager position;
    private DriveTrainComponent driveTrain;

    /**
     * Initializes a new DriveSinusoidalTimedAutonomousTask
     * @param duration to perform the task in seconds
     * @param yVelocity to apply to the driveTrain
     * @param magnitude of the sine wave to use for X velocity
     * @param rightMulti multiplier to use for the right side
     */
    public DriveSinusoidalTimedWithAngleTask(
        double duration, double yVelocity, double magnitude, double rightMult, PositionManager position,
        DriveTrainComponent driveTrainComponent)
    {
        super(duration);

        this.yVelocity = yVelocity;
        this.magnitude = magnitude;
        this.rightMult = rightMult;
        this.position = position;
        this.driveTrain = driveTrainComponent;
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
        double xVelocity = this.magnitude * Math.cos(2 * Math.PI * percentageTime);
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

    /**
     * Checks whether this task has completed, or whether it should continue being processed
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        if (super.hasCompleted())
            return true;

        double percentageTime = (this.timer.get() - this.startTime) / this.duration;
        if (percentageTime < .75)
            return false;

        double angle = this.position.getAngle();
        return (angle < 2.0 && angle > -2.0);
    }
}
