package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.common.AnglePair;
import frc.robot.driver.AnalogOperation;
import frc.robot.mechanisms.PigeonManager;

public class OrientationTask extends UpdateCycleTask
{
    private final double orientation;
    private final boolean waitUntilGoalReached;

    private PigeonManager pigeonManager;

    public OrientationTask(double orientation)
    {
        this(orientation, true);
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
        super.update();
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

        double currentYaw = this.pigeonManager.getYaw();
        double yawGoal = AnglePair.getClosestAngle(this.orientation, currentYaw, false).getAngle();
        return Math.abs(currentYaw - yawGoal) < TuningConstants.DRIVETRAIN_TURN_APPROXIMATION_STATIONARY;
    }
}
