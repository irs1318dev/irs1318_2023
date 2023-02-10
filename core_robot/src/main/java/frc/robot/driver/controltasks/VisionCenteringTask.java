package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.common.Helpers;
import frc.robot.common.PIDHandler;
import frc.robot.common.robotprovider.ITimer;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.OffboardVisionManager;

/**
 * Task that turns the robot a certain amount clockwise or counterclockwise in-place based on vision center
 */
public class VisionCenteringTask extends PIDTurnTaskBase
{
    protected final boolean aprilTag;

    protected OffboardVisionManager visionManager;

    /**
    * Initializes a new VisionCenteringTask
     * @param aprilTag whether to center on aprilTag or retroreflective vision target
    */
    public VisionCenteringTask(boolean aprilTag)
    {
        this(true, aprilTag, false);
    }

    /**
    * Initializes a new VisionCenteringTask
     * @param aprilTag whether to center on aprilTag or retroreflective vision target
     * @param bestEffort whether to end (true) or cancel (false, default) when we cannot see the game piece or vision target (for sequential tasks, whether to continue on or not)
    */
    public VisionCenteringTask(boolean aprilTag, boolean bestEffort)
    {
        this(true, aprilTag, bestEffort);
    }

    /**
     * Initializes a new VisionCenteringTask
     * @param useTime whether to make sure we are centered for a second or not
     * @param aprilTag whether to center on aprilTag or retroreflective vision target
     * @param bestEffort whether to end (true) or cancel (false, default) when we cannot see the game piece or vision target (for sequential tasks, whether to continue on or not)
     */
    public VisionCenteringTask(boolean useTime, boolean aprilTag, boolean bestEffort)
    {
        super(useTime, bestEffort);
        this.aprilTag = aprilTag;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.visionManager = this.getInjector().getInstance(OffboardVisionManager.class);

        this.setDigitalOperationState(DigitalOperation.VisionEnableRetroreflectiveProcessing, !this.aprilTag);
        this.setDigitalOperationState(DigitalOperation.VisionEnableAprilTagProcessing, this.aprilTag);
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        this.setDigitalOperationState(DigitalOperation.VisionEnableRetroreflectiveProcessing, false);
        this.setDigitalOperationState(DigitalOperation.VisionEnableAprilTagProcessing, false);
    }

    @Override
    protected Double getHorizontalAngle()
    {
        Double angle;
        if (this.aprilTag)
        {
            // Note: we want to point toward the AprilTag, not match its yaw (make ourselves parallel to it), so we can use the fact that tan(angle) = opposite / adjacent
            Double xOffset = this.visionManager.getAprilTagXOffset();
            Double yOffset = this.visionManager.getAprilTagYOffset();
            if (xOffset == null || yOffset == null)
            {
                angle = null;
            }
            else
            {
                angle = Helpers.atan2d(yOffset, xOffset);
            }
        }
        else
        {
            angle = this.visionManager.getVisionTargetHorizontalAngle();
        }

        return angle;
    }
}