package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.driver.*;
import frc.robot.mechanisms.DriveTrainMechanism;

/**
 * Abstract class defining a task that moves the robot a certain distance using Positional PID.
 * 
 */
public abstract class MoveDistanceOneShotTaskBase extends ControlTaskBase
{
    private final boolean resetPositionalOnEnd;
    private DriveTrainMechanism driveTrain;

    protected double startLeftTicks;
    protected double startRightTicks;

    protected double desiredFinalLeftTicks;
    protected double desiredFinalRightTicks;

    /**
     * Initializes a new MoveDistanceTaskBase
     * @param resetPositionalOnEnd
     */
    protected MoveDistanceOneShotTaskBase(boolean resetPositionalOnEnd)
    {
        this.resetPositionalOnEnd = resetPositionalOnEnd;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.driveTrain = this.getInjector().getInstance(DriveTrainMechanism.class);

        // set the start location
        this.setStartEncoderDistance();

        // calculate the desired end location
        this.determineFinalEncoderDistance();
    }

    /**
     * Set the start left and right encoder distances...
     */
    protected void setStartEncoderDistance()
    {
        this.startLeftTicks = this.driveTrain.getLeftPosition();
        this.startRightTicks = this.driveTrain.getRightPosition();
    }

    /**
     * Determine the final encoder distance
     */
    protected abstract void determineFinalEncoderDistance();

    /**
     * Run an iteration of the current task and apply any control changes
     */
    @Override
    public void update()
    {
        this.setDigitalOperationState(DigitalOperation.DriveTrainUsePositionalMode, true);
        this.setAnalogOperationState(AnalogOperation.DriveTrainLeftPosition, this.desiredFinalLeftTicks);
        this.setAnalogOperationState(AnalogOperation.DriveTrainRightPosition, this.desiredFinalRightTicks);
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        if (this.resetPositionalOnEnd)
        {
            this.setDigitalOperationState(DigitalOperation.DriveTrainUsePositionalMode, false);
            this.setAnalogOperationState(AnalogOperation.DriveTrainLeftPosition, 0.0);
            this.setAnalogOperationState(AnalogOperation.DriveTrainRightPosition, 0.0);
        }
    }

    /**
     * Checks whether this task has completed, or whether it should continue being processed
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        double leftTicks = this.driveTrain.getLeftPosition();
        double rightTicks = this.driveTrain.getRightPosition();

        // check how far away we are from the desired end location
        double leftDelta = Math.abs(this.desiredFinalLeftTicks - leftTicks);
        double rightDelta = Math.abs(this.desiredFinalRightTicks - rightTicks);

        // return that we have completed this task if are within an acceptable distance
        // from the desired end location for both left and right. 
        return leftDelta < TuningConstants.DRIVETRAIN_POSITIONAL_ACCEPTABLE_DELTA
            &&
            rightDelta < TuningConstants.DRIVETRAIN_POSITIONAL_ACCEPTABLE_DELTA;
    }
}
