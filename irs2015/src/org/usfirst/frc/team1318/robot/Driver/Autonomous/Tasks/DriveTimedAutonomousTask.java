package org.usfirst.frc.team1318.robot.Driver.Autonomous.Tasks;

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
     */
    @Override
    public void update()
    {
        //        data.setDriveTrainPositionMode(false);
        //        data.setDriveTrainXVelocity(this.xVelocity);
        //        data.setDriveTrainYVelocity(this.yVelocity);
    }

    /**
     * Cancel the current task and clear control changes
     */
    @Override
    public void stop()
    {
        super.stop();

        //        data.setDriveTrainXVelocity(0.0);
        //        data.setDriveTrainYVelocity(0.0);
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        super.end();

        //        data.setDriveTrainXVelocity(0.0);
        //        data.setDriveTrainYVelocity(0.0);
    }
}
