package frc.robot.driver.controltasks;

import frc.robot.driver.*;

/**
 * Task that drives at a certain velocity for a certain duration.
 * 
 */
public class DriveVelocityTimedTask extends TimedTask
{
    private final double xVelocity;
    private final double yVelocity;

    /**
     * Initializes a new DriveTimedTask
     * @param duration to perform the task in seconds
     * @param xVelocity to apply to the driveTrain
     * @param yVelocity to apply to the driveTrain
     */
    public DriveVelocityTimedTask(double duration, double xVelocity, double yVelocity)
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
        this.setDigitalOperationState(DigitalOperation.DriveTrainUsePositionalMode, false);
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, this.yVelocity);
        this.setAnalogOperationState(AnalogOperation.DriveTrainTurn, this.xVelocity);
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        super.end();

        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainTurn, 0.0);
    }
}
