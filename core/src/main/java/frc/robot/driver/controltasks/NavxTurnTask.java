package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.common.Helpers;
import frc.robot.common.PIDHandler;
import frc.robot.common.robotprovider.ITimer;
import frc.robot.driver.Operation;
import frc.robot.driver.common.IControlTask;
import frc.robot.mechanisms.DriveTrainMechanism;
import frc.robot.mechanisms.PositionManager;

/**
 * Task that turns the robot a certain amount clockwise or counterclockwise in-place based on vision center
 */
public class NavxTurnTask extends ControlTaskBase implements IControlTask
{
    private final boolean useTime;
    private final double turnAngle;
    private final double waitTime;
    private final boolean relativeMode;
    private final boolean fastMode;

    private PIDHandler turnPidHandler;
    private PositionManager pManager;
    private DriveTrainMechanism dt;
    private ITimer timer;

    private double desiredTurnVelocity;
    private Double completeTime;
    private double startingAngle;

    /**
    * Initializes a new NavxTurnTask using time to make sure we completed turn
    * @param turnAngle the desired angle
    */
    public NavxTurnTask(double turnAngle)
    {
        this(true, turnAngle);
    }

    /**
    * Initializes a new NavxTurnTask
    * @param useTime whether to make sure we completed turn for a second or not
    * @param turnAngle the desired angle
    */
    public NavxTurnTask(boolean useTime, double turnAngle)
    {
        this(
            useTime,
            turnAngle,
            TuningConstants.NAVX_TURN_COMPLETE_TIME,
            false,
            false);
    }

    /**
    * Initializes a new NavxTurnTask
    * @param useTime whether to make sure we completed turn for a second or not
    * @param turnAngle the desired angle
    * @param relativeMode whether to use relative mode
    * @param fastMode whether to use fast mode
    */
    public NavxTurnTask(boolean useTime, double turnAngle, boolean relativeMode, boolean fastMode)
    {
        this(
            useTime,
            turnAngle,
            TuningConstants.NAVX_TURN_COMPLETE_TIME,
            relativeMode,
            fastMode);
    }

    /**
     * Initializes a new NavxTurnTask using a variable wait time after turn has reached the goal
     * @param turnAngle the desired angle
     * @param waitTime the desired wait time
     */
    public NavxTurnTask(double turnAngle, double waitTime)
    {
        this(
            true,
            turnAngle,
            waitTime,
            false,
            false);

    }

    /**
    * Initializes a new NavxTurnTask
    * @param useTime whether to make sure we completed turn for a second or not
    * @param turnAngle the desired angle
    * @param waitTime the desired wait time
    * @param relativeMode whether to use relative mode (turn relative to current angle), or absolute mode (turn relative to starting orientation)
    * @param fastMode whether to use fast mode (or slow/consistent mode)
    */
    public NavxTurnTask(boolean useTime, double turnAngle, double waitTime, boolean relativeMode, boolean fastMode)
    {
        this.useTime = useTime;
        this.turnAngle = turnAngle;
        this.waitTime = waitTime;
        this.relativeMode = relativeMode;
        this.fastMode = fastMode;

        this.startingAngle = 0;
        this.turnPidHandler = null;
        this.completeTime = null;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.pManager = this.getInjector().getInstance(PositionManager.class);
        this.dt = this.getInjector().getInstance(DriveTrainMechanism.class);
        this.timer = this.getInjector().getInstance(ITimer.class);

        this.turnPidHandler = this.createTurnHandler();
        if (this.relativeMode)
        {
            this.startingAngle = this.pManager.getNavxAngle();

            // if the navx isn't working, let's fall back to using odometry
            if (!this.pManager.getNavxIsConnected())
            {
                this.startingAngle = this.pManager.getOdometryAngle();
            }
        }
    }

    /**
     * Run an iteration of the current task and apply any control changes
     */
    @Override
    public void update()
    {
        this.setDigitalOperationState(Operation.DriveTrainUsePositionalMode, false);
        this.setDigitalOperationState(Operation.DriveTrainSimpleMode, this.fastMode);

        double currentMeasuredAngle = this.pManager.getNavxAngle();
        double currentDesiredAngle = this.turnAngle + this.startingAngle;

        // if the navx isn't connected, let's fall back to using odometry
        if (!this.pManager.getNavxIsConnected())
        {
            currentMeasuredAngle = this.pManager.getOdometryAngle();
            currentDesiredAngle = currentDesiredAngle % 360.0; // odometry measures angles from 0 to 30 only
        }

        this.desiredTurnVelocity = this.turnPidHandler.calculatePosition(currentDesiredAngle, currentMeasuredAngle);

        this.setAnalogOperationState(Operation.DriveTrainTurn, this.desiredTurnVelocity);
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        this.setDigitalOperationState(Operation.DriveTrainUsePositionalMode, false);
        this.setDigitalOperationState(Operation.DriveTrainSimpleMode, false);
        this.setAnalogOperationState(Operation.DriveTrainTurn, 0.0);
    }

    /**
     * Checks whether this task has completed, or whether it should continue being processed
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        double currentMeasuredAngle = this.pManager.getNavxAngle();
        double currentTurnVelocity = this.dt.getLeftVelocity();
        double currentDesiredAngle = this.startingAngle + this.turnAngle;

        // if the navx isn't connected, let's fall back to using odometry
        if (!this.pManager.getNavxIsConnected())
        {
            currentMeasuredAngle = this.pManager.getOdometryAngle();
            currentDesiredAngle = currentDesiredAngle % 360.0;
        }

        double centerAngleDifference = Math.abs(currentMeasuredAngle - currentDesiredAngle);
        if ((!this.fastMode && centerAngleDifference > TuningConstants.MAX_NAVX_TURN_RANGE_DEGREES) ||
            (this.fastMode && centerAngleDifference > TuningConstants.MAX_NAVX_FAST_TURN_RANGE_DEGREES))
        {
            return false;
        }

        if (!this.useTime)
        {
            return true;
        }
        else
        {
            // If desired and current turn velocity are near 0, complete this task. Otherwise, use timer.
            if (Helpers.WithinDelta(currentTurnVelocity, 0.0, TuningConstants.NAVX_TURN_COMPLETE_CURRENT_VELOCITY_DELTA)
                && Helpers.WithinDelta(this.desiredTurnVelocity, 0.0, TuningConstants.NAVX_TURN_COMPLETE_DESIRED_VELOCITY_DELTA))
            {
                return true;
            }

            if (this.completeTime == null)
            {
                this.completeTime = timer.get();
                return false;
            }
            else if (timer.get() - this.completeTime < this.waitTime)
            {
                return false;
            }
            else
            {
                return true;
            }
        }
    }

    protected PIDHandler createTurnHandler()
    {
        return new PIDHandler(
            this.fastMode ? TuningConstants.NAVX_FAST_TURN_PID_KP : TuningConstants.NAVX_TURN_PID_KP,
            this.fastMode ? TuningConstants.NAVX_FAST_TURN_PID_KI : TuningConstants.NAVX_TURN_PID_KI,
            this.fastMode ? TuningConstants.NAVX_FAST_TURN_PID_KD : TuningConstants.NAVX_TURN_PID_KD,
            this.fastMode ? TuningConstants.NAVX_FAST_TURN_PID_KF : TuningConstants.NAVX_TURN_PID_KF,
            this.fastMode ? TuningConstants.NAVX_FAST_TURN_PID_KS : TuningConstants.NAVX_TURN_PID_KS,
            this.fastMode ? TuningConstants.NAVX_FAST_TURN_PID_MIN : TuningConstants.NAVX_TURN_PID_MIN,
            this.fastMode ? TuningConstants.NAVX_FAST_TURN_PID_MAX : TuningConstants.NAVX_TURN_PID_MAX,
            this.getInjector().getInstance(ITimer.class));
    }
}
