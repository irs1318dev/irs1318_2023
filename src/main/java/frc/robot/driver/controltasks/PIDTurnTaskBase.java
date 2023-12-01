package frc.robot.driver.controltasks;

import frc.lib.controllers.PIDHandler;
import frc.lib.robotprovider.ITimer;
import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;

/**
 * Task that turns the robot a certain amount clockwise or counterclockwise in-place based on vision center
 */
public abstract class PIDTurnTaskBase extends ControlTaskBase
{
    private static final int NO_ANGLE_THRESHOLD = 40;

    private final boolean useTime;
    private final boolean bestEffort;

    private ITimer timer;
    private PIDHandler turnPidHandler;

    private Double centeredTime;

    private int noAngleCount;

    /**
     * Initializes a new PIDTurnTaskBase
     * @param useTime whether to make sure we are centered for a second or not
     * @param bestEffort whether to end (true) or cancel (false, default) when we cannot see the game piece or vision target (for sequential tasks, whether to continue on or not)
     */
    public PIDTurnTaskBase(boolean useTime, boolean bestEffort)
    {
        this.useTime = useTime;
        this.bestEffort = bestEffort;

        this.turnPidHandler = null;
        this.centeredTime = null;

        this.noAngleCount = 0;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.turnPidHandler = this.createTurnHandler();

        if (this.useTime)
        {
            this.timer = this.getInjector().getInstance(ITimer.class);
        }

        this.setDigitalOperationState(DigitalOperation.VisionDisableStream, false);
        this.setDigitalOperationState(DigitalOperation.DriveTrainEnableFieldOrientation, false);
        this.setDigitalOperationState(DigitalOperation.DriveTrainDisableFieldOrientation, false);
        this.setDigitalOperationState(DigitalOperation.DriveTrainUseRobotOrientation, true);
    }

    /**
     * Run an iteration of the current task and apply any control changes
     */
    @Override
    public void update()
    {
        Double currentMeasuredAngle = this.getHorizontalAngle();
        if (currentMeasuredAngle != null)
        {
            double turnSpeed = this.turnPidHandler.calculatePosition(0.0, currentMeasuredAngle);
            this.setAnalogOperationState(AnalogOperation.DriveTrainSpinRight, turnSpeed);
        }
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        this.setAnalogOperationState(AnalogOperation.DriveTrainSpinRight, 0.0);

        this.setDigitalOperationState(DigitalOperation.VisionDisableStream, false);
        this.setDigitalOperationState(DigitalOperation.DriveTrainEnableFieldOrientation, false);
        this.setDigitalOperationState(DigitalOperation.DriveTrainDisableFieldOrientation, false);
        this.setDigitalOperationState(DigitalOperation.DriveTrainUseRobotOrientation, false);
    }

    /**
     * Checks whether this task has completed, or whether it should continue being processed
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        Double currentMeasuredAngle = this.getHorizontalAngle();
        if (this.bestEffort)
        {
            if (currentMeasuredAngle == null)
            {
                this.noAngleCount++;

                return this.noAngleCount >= PIDTurnTaskBase.NO_ANGLE_THRESHOLD;
            }

            this.noAngleCount = 0;
        }
        else if (currentMeasuredAngle == null)
        {
            return false;
        }

        double centerAngleDifference = Math.abs(currentMeasuredAngle);
        if (centerAngleDifference > TuningConstants.MAX_PID_TURNING_RANGE_DEGREES)
        {
            return false;
        }

        if (!this.useTime)
        {
            return true;
        }

        // otherwise, use time:
        double currTime = this.timer.get();
        if (this.centeredTime == null)
        {
            this.centeredTime = currTime;
            return false;
        }
        else if (currTime - this.centeredTime < TuningConstants.PID_TURNING_DURATION)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    /**
     * Checks whether this task should be stopped, or whether it should continue being processed.
     * @return true if we should cancel this task (and stop performing any subsequent tasks), otherwise false (to keep processing this task)
     */
    @Override
    public boolean shouldCancel()
    {
        if (this.bestEffort)
        {
            // note: in best-effort mode, this is done in hasCompleted() instead.
            return super.shouldCancel();
        }

        if (this.getHorizontalAngle() == null)
        {
            this.noAngleCount++;
        }
        else
        {
            this.noAngleCount = 0;
        }

        return this.noAngleCount >= PIDTurnTaskBase.NO_ANGLE_THRESHOLD || super.shouldCancel();
    }

    protected abstract Double getHorizontalAngle();

    protected PIDHandler createTurnHandler()
    {
        return new PIDHandler(
            TuningConstants.STATIONARY_PID_TURNING_PID_KP,
            TuningConstants.STATIONARY_PID_TURNING_PID_KI,
            TuningConstants.STATIONARY_PID_TURNING_PID_KD,
            TuningConstants.STATIONARY_PID_TURNING_PID_KF,
            TuningConstants.STATIONARY_PID_TURNING_PID_KS,
            TuningConstants.STATIONARY_PID_TURNING_PID_MIN,
            TuningConstants.STATIONARY_PID_TURNING_PID_MAX,
            this.getInjector().getInstance(ITimer.class));
    }
}