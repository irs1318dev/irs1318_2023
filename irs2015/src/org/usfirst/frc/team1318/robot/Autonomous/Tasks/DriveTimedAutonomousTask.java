package org.usfirst.frc.team1318.robot.Autonomous.Tasks;

import org.usfirst.frc.team1318.robot.Autonomous.AutonomousControlData;

public class DriveTimedAutonomousTask extends TimedAutonomousTask
{
    private final double xVelocity;
    private final double yVelocity;

    /**
     * Initializes a new DriveTimedAutonomousTask
     * @param duration to perform the task in seconds
     * @param xVelocity to apply to the driveTrain
     * @param yVelocity to apply to the driveTrain
     */
    public DriveTimedAutonomousTask(double duration, double xVelocity, double yVelocity)
    {
        super(duration);

        this.xVelocity = xVelocity;
        this.yVelocity = yVelocity;
    }

    /**
     * Run an iteration of the current task and apply any control changes 
     * @param data to which we should apply updated settings
     */
    @Override
    public void update(AutonomousControlData data)
    {
        data.setDriveTrainPositionMode(false);
        data.setDriveTrainXVelocity(this.xVelocity);
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
