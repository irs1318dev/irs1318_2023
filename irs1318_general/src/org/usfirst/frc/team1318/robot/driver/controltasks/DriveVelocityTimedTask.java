package org.usfirst.frc.team1318.robot.driver.controltasks;

import org.usfirst.frc.team1318.robot.driver.Operation;

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
        this.setDigitalOperationState(Operation.DriveTrainUsePositionalMode, false);
        this.setAnalogOperationState(Operation.DriveTrainMoveForward, this.yVelocity);
        this.setAnalogOperationState(Operation.DriveTrainTurn, this.xVelocity);
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
}
