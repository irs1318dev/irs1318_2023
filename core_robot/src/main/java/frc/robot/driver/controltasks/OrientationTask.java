package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.mechanisms.PigeonManager;

public class OrientationTask extends UpdateCycleTask
{
    private final double orientation;
    private final boolean waitUntilGoalReached;

    private PigeonManager pigeonManager;

    public OrientationTask(double orientation)
    {
        this(orientation, false);
    }

    public OrientationTask(double orientation, boolean waitUntilGoalReached)
    {
        super(1);

        this.orientation = orientation;
        this.waitUntilGoalReached = waitUntilGoalReached;
    }

    @Override
    public void begin()
    {
        this.pigeonManager = this.getInjector().getInstance(PigeonManager.class);

        this.setAnalogOperationState(AnalogOperation.DriveTrainTurnAngleGoal, this.orientation);
    }

    @Override
    public void update()
    {
        this.setAnalogOperationState(AnalogOperation.DriveTrainTurnAngleGoal, this.orientation);
    }

    @Override
    public void end()
    {
        this.setAnalogOperationState(AnalogOperation.DriveTrainTurnAngleGoal, TuningConstants.MAGIC_NULL_VALUE);
    }

    @Override
    public boolean hasCompleted()
    {
        if (!this.waitUntilGoalReached)
        {
            return super.hasCompleted();
        }

        return Math.abs(this.pigeonManager.getYaw() - this.orientation) < TuningConstants.DRIVETRAIN_TURN_APPROXIMATION_STATIONARY;
    }
}
