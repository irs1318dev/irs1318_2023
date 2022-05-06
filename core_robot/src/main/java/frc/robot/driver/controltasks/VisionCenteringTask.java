package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.common.PIDHandler;
import frc.robot.common.robotprovider.ITimer;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.OffboardVisionManager;

/**
 * Task that turns the robot a certain amount clockwise or counterclockwise in-place based on vision center
 */
public class VisionCenteringTask extends ControlTaskBase
{
    private static final int NO_CENTER_THRESHOLD = 40;

    private final boolean useTime;
    private final boolean gamePiece;
    private final boolean bestEffort;

    private OffboardVisionManager visionManager;
    private ITimer timer;
    private PIDHandler turnPidHandler;

    private Double centeredTime;

    private int noCenterCount;

    /**
    * Initializes a new VisionCenteringTask
     * @param gamePiece whether to center on game piece or vision target
    */
    public VisionCenteringTask(boolean gamePiece)
    {
        this(true, gamePiece, false);
    }

    /**
    * Initializes a new VisionCenteringTask
     * @param gamePiece whether to center on game piece or vision target
     * @param bestEffort whether to end (true) or cancel (false, default) when we cannot see the game piece or vision target (for sequential tasks, whether to continue on or not)
    */
    public VisionCenteringTask(boolean gamePiece, boolean bestEffort)
    {
        this(true, gamePiece, bestEffort);
    }

    /**
     * Initializes a new VisionCenteringTask
     * @param useTime whether to make sure we are centered for a second or not
     * @param gamePiece whether to center on game piece or vision target
     * @param bestEffort whether to end (true) or cancel (false, default) when we cannot see the game piece or vision target (for sequential tasks, whether to continue on or not)
     */
    public VisionCenteringTask(boolean useTime, boolean gamePiece, boolean bestEffort)
    {
        this.useTime = useTime;
        this.gamePiece = gamePiece;
        this.bestEffort = bestEffort;

        this.turnPidHandler = null;
        this.centeredTime = null;

        this.noCenterCount = 0;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.visionManager = this.getInjector().getInstance(OffboardVisionManager.class);
        this.turnPidHandler = this.createTurnHandler();

        if (this.useTime)
        {
            this.timer = this.getInjector().getInstance(ITimer.class);
        }

        this.setDigitalOperationState(DigitalOperation.VisionDisableStream, false);
        this.setDigitalOperationState(DigitalOperation.DriveTrainEnableFieldOrientation, false);
        this.setDigitalOperationState(DigitalOperation.DriveTrainDisableFieldOrientation, false);
        this.setDigitalOperationState(DigitalOperation.DriveTrainUseRobotOrientation, true);
        this.setDigitalOperationState(DigitalOperation.VisionEnableRetroreflectiveProcessing, !this.gamePiece);
        this.setDigitalOperationState(DigitalOperation.VisionEnableGamePieceProcessing, this.gamePiece);
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
            this.setAnalogOperationState(
                AnalogOperation.DriveTrainTurnSpeed,
                turnSpeed);
        }
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        this.setAnalogOperationState(AnalogOperation.DriveTrainTurnSpeed, 0.0);

        this.setDigitalOperationState(DigitalOperation.VisionDisableStream, false);
        this.setDigitalOperationState(DigitalOperation.DriveTrainEnableFieldOrientation, false);
        this.setDigitalOperationState(DigitalOperation.DriveTrainDisableFieldOrientation, false);
        this.setDigitalOperationState(DigitalOperation.DriveTrainUseRobotOrientation, false);
        this.setDigitalOperationState(DigitalOperation.VisionEnableRetroreflectiveProcessing, false);
        this.setDigitalOperationState(DigitalOperation.VisionEnableGamePieceProcessing, false);
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
                this.noCenterCount++;

                return this.noCenterCount >= VisionCenteringTask.NO_CENTER_THRESHOLD;
            }

            this.noCenterCount = 0;
        }
        else if (currentMeasuredAngle == null)
        {
            return false;
        }

        double centerAngleDifference = Math.abs(currentMeasuredAngle);
        if (centerAngleDifference > TuningConstants.MAX_VISION_CENTERING_RANGE_DEGREES)
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
        else if (currTime - this.centeredTime < TuningConstants.VISION_CENTERING_DURATION)
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
            this.noCenterCount++;
        }
        else
        {
            this.noCenterCount = 0;
        }

        return this.noCenterCount >= VisionCenteringTask.NO_CENTER_THRESHOLD || super.shouldCancel();
    }

    protected Double getDistance()
    {
        Double distance;
        if (this.gamePiece)
        {
            distance = this.visionManager.getGamePieceDistance();
        }
        else
        {
            distance = this.visionManager.getVisionTargetDistance();
        }

        return distance;
    }

    protected Double getHorizontalAngle()
    {
        Double angle;
        if (this.gamePiece)
        {
            angle = this.visionManager.getGamePieceHorizontalAngle();
        }
        else
        {
            angle = this.visionManager.getVisionTargetHorizontalAngle();
        }

        return angle;
    }

    protected PIDHandler createTurnHandler()
    {
        return new PIDHandler(
            TuningConstants.VISION_STATIONARY_CENTERING_PID_KP,
            TuningConstants.VISION_STATIONARY_CENTERING_PID_KI,
            TuningConstants.VISION_STATIONARY_CENTERING_PID_KD,
            TuningConstants.VISION_STATIONARY_CENTERING_PID_KF,
            TuningConstants.VISION_STATIONARY_CENTERING_PID_KS,
            TuningConstants.VISION_STATIONARY_CENTERING_PID_MIN,
            TuningConstants.VISION_STATIONARY_CENTERING_PID_MAX,
            this.getInjector().getInstance(ITimer.class));
    }
}