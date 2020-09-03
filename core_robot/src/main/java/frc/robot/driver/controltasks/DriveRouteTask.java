package frc.robot.driver.controltasks;

import java.util.function.Function;

import frc.robot.TuningConstants;
import frc.robot.driver.*;
import frc.robot.mechanisms.DriveTrainMechanism;

public class DriveRouteTask extends TimedTask
{
    private final Function<Double, Double> leftPositionPerTime;
    private final Function<Double, Double> rightPositionPerTime;
    private DriveTrainMechanism driveTrain;

    private double startLeftTicks;
    private double startRightTicks;

    private double endLeftTicks;
    private double endRightTicks;

    /**
     * Initializes a new DriveRouteTask
     * @param left function that calculates the desired left tick position per ratio of time that has elapsed
     * @param right function that calculates the desired right tick position per ratio of time that has elapsed
     * @param duration to take to drive the route
     */
    public DriveRouteTask(Function<Double, Double> left, Function<Double, Double> right, double duration)
    {
        super(duration);

        this.leftPositionPerTime = left;
        this.rightPositionPerTime = right;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        super.begin();

        this.driveTrain = this.getInjector().getInstance(DriveTrainMechanism.class);

        this.startLeftTicks = this.driveTrain.getLeftPosition();
        this.startRightTicks = this.driveTrain.getRightPosition();

        this.endLeftTicks = this.startLeftTicks + this.leftPositionPerTime.apply(1.0);
        this.endRightTicks = this.startRightTicks + this.rightPositionPerTime.apply(1.0);
    }

    /**
     * Run an iteration of the current task and apply any control changes
     */
    @Override
    public void update()
    {
        double t = this.getRatioComplete();
        if (t >= 1.0)
        {
            t = 1.0;
        }
        else if (t <= 0.0)
        {
            t = 0.0;
        }

        this.setDigitalOperationState(DigitalOperation.DriveTrainUsePositionalMode, true);
        this.setAnalogOperationState(AnalogOperation.DriveTrainLeftPosition, this.startLeftTicks + this.leftPositionPerTime.apply(t));
        this.setAnalogOperationState(AnalogOperation.DriveTrainRightPosition, this.startRightTicks + this.rightPositionPerTime.apply(t));
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        super.end();

        this.setAnalogOperationState(AnalogOperation.DriveTrainLeftPosition, this.endLeftTicks);
        this.setAnalogOperationState(AnalogOperation.DriveTrainRightPosition, this.endRightTicks);

        this.setDigitalOperationState(DigitalOperation.DriveTrainUsePositionalMode, false);
    }

    /**
     * Checks whether this task has completed, or whether it should continue being processed
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        double leftEncoderTicks = this.driveTrain.getLeftPosition();
        double rightEncoderTicks = this.driveTrain.getRightPosition();

        // check how far away we are from the desired end location
        double leftDelta = Math.abs(this.endLeftTicks - leftEncoderTicks);
        double rightDelta = Math.abs(this.endRightTicks - rightEncoderTicks);

        // return that we have completed this task if are within an acceptable distance
        // from the desired end location for both left and right. 
        return super.hasCompleted()
            || (leftDelta < TuningConstants.DRIVETRAIN_POSITIONAL_ACCEPTABLE_DELTA
                && rightDelta < TuningConstants.DRIVETRAIN_POSITIONAL_ACCEPTABLE_DELTA);
    }
}
