package org.usfirst.frc.team1318.robot.driver.controltasks;

import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.common.PIDHandler;
import org.usfirst.frc.team1318.robot.common.wpilib.ITimer;
import org.usfirst.frc.team1318.robot.driver.Operation;
import org.usfirst.frc.team1318.robot.driver.common.IControlTask;
import org.usfirst.frc.team1318.robot.vision.VisionManager;

/**
 * Task that turns the robot a certain amount clockwise or counterclockwise in-place based on vision center
 */
public class VisionCenteringTask extends ControlTaskBase implements IControlTask
{
    private static final int NO_CENTER_THRESHOLD = 20;

    private final boolean useTime;

    private PIDHandler turnPidHandler;
    private Double centeredTime;
    protected VisionManager visionManager;

    private int noCenterCount;

    /**
    * Initializes a new VisionCenteringTask
    */
    public VisionCenteringTask()
    {
        this(true);
    }

    /**
    * Initializes a new VisionCenteringTask
    * @param useTime whether to make sure we are centered for a second or not
    */
    public VisionCenteringTask(boolean useTime)
    {
        this.useTime = useTime;

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
        this.visionManager = this.getInjector().getInstance(VisionManager.class);
        this.turnPidHandler = this.createTurnHandler();
        this.setDigitalOperationState(Operation.EnableVision, true);
    }

    /**
     * Run an iteration of the current task and apply any control changes
     */
    @Override
    public void update()
    {
        this.setDigitalOperationState(Operation.DriveTrainUsePositionalMode, false);

        Double currentMeasuredAngle = this.visionManager.getMeasuredAngle();
        Double currentDesiredAngle = this.visionManager.getDesiredAngle();
        if (currentMeasuredAngle != null && currentDesiredAngle != null)
        {
            this.setAnalogOperationState(
                Operation.DriveTrainTurn,
                -this.turnPidHandler.calculatePosition(currentDesiredAngle, currentMeasuredAngle));
        }
    }

    /**
     * Cancel the current task and clear control changes
     */
    @Override
    public void stop()
    {
        this.setDigitalOperationState(Operation.DriveTrainUsePositionalMode, false);
        this.setAnalogOperationState(Operation.DriveTrainTurn, 0.0);

        this.setDigitalOperationState(Operation.EnableVision, false);
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        this.setDigitalOperationState(Operation.DriveTrainUsePositionalMode, false);
        this.setAnalogOperationState(Operation.DriveTrainTurn, 0.0);

        this.setDigitalOperationState(Operation.EnableVision, false);
    }

    /**
     * Checks whether this task has completed, or whether it should continue being processed
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        Double currentMeasuredAngle = this.visionManager.getMeasuredAngle();
        Double currentDesiredAngle = this.visionManager.getDesiredAngle();
        if (currentMeasuredAngle == null || currentDesiredAngle == null)
        {
            return false;
        }

        double centerAngleDifference = Math.abs(currentMeasuredAngle - currentDesiredAngle);
        if (centerAngleDifference > TuningConstants.MAX_VISION_CENTERING_RANGE_DEGREES)
        {
            return false;
        }

        if (!this.useTime)
        {
            return true;
        }
        else
        {
            ITimer timer = this.getInjector().getInstance(ITimer.class);
            if (this.centeredTime == null)
            {
                this.centeredTime = timer.get();
                return false;
            }
            else if (timer.get() - this.centeredTime < 0.75)
            {
                return false;
            }
            else
            {
                return true;
            }
        }
    }

    @Override
    public boolean shouldCancel()
    {
        if (this.visionManager.getCenter() == null)
        {
            this.noCenterCount++;
        }
        else
        {
            this.noCenterCount = 0;
        }

        return this.noCenterCount >= VisionCenteringTask.NO_CENTER_THRESHOLD;
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
