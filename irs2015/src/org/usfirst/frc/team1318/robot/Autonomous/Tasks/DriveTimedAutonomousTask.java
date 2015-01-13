package org.usfirst.frc.team1318.robot.Autonomous.Tasks;

import org.usfirst.frc.team1318.robot.Autonomous.AutonomousControlData;

public class DriveTimedAutonomousTask extends TimedAutonomousTask
{
    private double xVelocity;
    private double yVelocity;

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
        data.setDriveTrainXVelocity(this.xVelocity);
        data.setDriveTrainYVelocity(this.yVelocity);
    }
}
