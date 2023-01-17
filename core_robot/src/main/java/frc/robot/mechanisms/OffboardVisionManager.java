package frc.robot.mechanisms;

import frc.robot.*;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.*;
import frc.robot.driver.common.*;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * Offboard Vision manager.
 */
@Singleton
public class OffboardVisionManager implements IMechanism
{
    private final IDriver driver;
    private final ILogger logger;

    private final IDriverStation driverStation;
    private final INetworkTableProvider networkTable;

    private IDoubleSubscriber atXOffsetSubscriber;
    private IDoubleSubscriber atYOffsetSubscriber;
    private IDoubleSubscriber atZOffsetSubscriber;
    private IDoubleSubscriber atYawSubscriber;
    private IDoubleSubscriber atPitchSubscriber;
    private IDoubleSubscriber atRollSubscriber;
    private IDoubleSubscriber rrDistanceSubscriber;
    private IDoubleSubscriber rrAngleSubscriber;
    private IIntegerSubscriber heartbeatSubscriber;

    private Double atXOffset;
    private Double atYOffset;
    private Double atZOffset;
    private Double atYaw;
    private Double atPitch;
    private Double atRoll;
    private Double rrDistance;
    private Double rrAngle;

    private int missedHeartbeats;
    private long prevHeartbeat;

    /**
     * Initializes a new OffboardVisionManager
     * @param driver for obtaining operations
     * @param logger for logging to smart dashboard
     * @param provider for obtaining electronics objects
     */
    @Inject
    public OffboardVisionManager(IDriver driver, LoggingManager logger, IRobotProvider provider)
    {
        this.driver = driver;
        this.logger = logger;

        this.driverStation = provider.getDriverStation();
        this.networkTable = provider.getNetworkTableProvider();
        this.atXOffsetSubscriber = this.networkTable.getDoubleSubscriber("at.xOffset");
        this.atYOffsetSubscriber = this.networkTable.getDoubleSubscriber("at.yOffset");
        this.atZOffsetSubscriber = this.networkTable.getDoubleSubscriber("at.zOffset");
        this.atYawSubscriber = this.networkTable.getDoubleSubscriber("at.yaw");
        this.atPitchSubscriber = this.networkTable.getDoubleSubscriber("at.pitch");
        this.atRollSubscriber = this.networkTable.getDoubleSubscriber("at.roll");
        this.rrDistanceSubscriber = this.networkTable.getDoubleSubscriber("rr.distance");
        this.rrAngleSubscriber = this.networkTable.getDoubleSubscriber("rr.horizontalAngle");
        this.heartbeatSubscriber = this.networkTable.getIntegerSubscriber("v.heartbeat");

        this.atXOffset = null;
        this.atYOffset = null;
        this.atZOffset = null;
        this.atYaw = null;
        this.atPitch = null;
        this.atRoll = null;
        this.rrDistance = null;
        this.rrAngle = null;

        this.missedHeartbeats = 0;
        this.prevHeartbeat = 0L;
    }

    /**
     * read all of the sensors for the mechanism that we will use in macros/autonomous mode and record their values
     */
    @Override
    public void readSensors()
    {
        this.atXOffset = this.atXOffsetSubscriber.get();
        this.atYOffset = this.atYOffsetSubscriber.get();
        this.atZOffset = this.atZOffsetSubscriber.get();
        this.atYaw = this.atYawSubscriber.get();
        this.atPitch = this.atPitchSubscriber.get();
        this.atRoll = this.atRollSubscriber.get();
        this.rrDistance = this.rrDistanceSubscriber.get();
        this.rrAngle = this.rrAngleSubscriber.get();

        long newHeartbeat = this.heartbeatSubscriber.get();
        if (this.prevHeartbeat != newHeartbeat)
        {
            this.missedHeartbeats = 0;
        }
        else
        {
            this.missedHeartbeats++;
        }

        this.logger.logNumber(LoggingKey.OffboardVisionMissedHeartbeats, this.missedHeartbeats);

        boolean missedHeartbeatExceedsThreshold = this.missedHeartbeats > TuningConstants.VISION_MISSED_HEARTBEAT_THRESHOLD;

        // reset if we couldn't find the april tag
        if (missedHeartbeatExceedsThreshold || this.atXOffset == TuningConstants.MAGIC_NULL_VALUE || this.atYOffset == TuningConstants.MAGIC_NULL_VALUE || this.atZOffset == TuningConstants.MAGIC_NULL_VALUE)
        {
            this.atXOffset = null;
            this.atYOffset = null;
            this.atZOffset = null;
            this.atYaw = null;
            this.atPitch = null;
            this.atRoll = null;
        }

        // reset if we couldn't find the retro-reflective vision target
        if (missedHeartbeatExceedsThreshold || this.rrDistance < 0.0 || this.rrAngle == TuningConstants.MAGIC_NULL_VALUE)
        {
            this.rrDistance = null;
            this.rrAngle = null;
        }

        this.logger.logNumber(LoggingKey.OffboardVisionAprilTagXOffset, this.atXOffset);
        this.logger.logNumber(LoggingKey.OffboardVisionAprilTagYOffset, this.atYOffset);
        this.logger.logNumber(LoggingKey.OffboardVisionAprilTagZOffset, this.atZOffset);
        this.logger.logNumber(LoggingKey.OffboardVisionAprilTagYaw, this.atYaw);
        this.logger.logNumber(LoggingKey.OffboardVisionAprilTagPitch, this.atPitch);
        this.logger.logNumber(LoggingKey.OffboardVisionAprilTagRoll, this.atRoll);
        this.logger.logNumber(LoggingKey.OffboardVisionRRTargetDistance, this.rrDistance);
        this.logger.logNumber(LoggingKey.OffboardVisionRRTargetHorizontalAngle, this.rrAngle);
    }

    @Override
    public void update()
    {
        boolean enableVision = !this.driver.getDigital(DigitalOperation.VisionForceDisable);
        boolean enableVideoStream = !this.driver.getDigital(DigitalOperation.VisionDisableStream);
        boolean enableAprilTagProcessing = this.driver.getDigital(DigitalOperation.VisionEnableAprilTagProcessing);
        boolean enableRetroreflectiveProcessing = this.driver.getDigital(DigitalOperation.VisionEnableRetroreflectiveProcessing);

        double visionProcessingMode = 0.0;
        if (enableVision)
        {
            if (enableAprilTagProcessing)
            {
                visionProcessingMode = 1.0;
            }
            else if (enableRetroreflectiveProcessing)
            {
                visionProcessingMode = 2.0;
            }
        }

        this.logger.logBoolean(LoggingKey.OffboardVisionEnableStream, enableVideoStream);
        this.logger.logNumber(LoggingKey.OffboardVisionProcessingMode, visionProcessingMode);
    }

    @Override
    public void stop()
    {
        this.logger.logBoolean(LoggingKey.OffboardVisionEnableStream, false);
        this.logger.logNumber(LoggingKey.OffboardVisionProcessingMode, 0.0);
    }

    public Double getVisionTargetHorizontalAngle()
    {
        return this.rrAngle;
    }

    public Double getVisionTargetDistance()
    {
        return this.rrDistance;
    }

    public Double getAprilTagXOffset()
    {
        return this.atXOffset;
    }

    public Double getAprilTagYOffset()
    {
        return this.atXOffset;
    }

    public Double getAprilTagZOffset()
    {
        return this.atXOffset;
    }

    public Double getAprilTagYaw()
    {
        return this.atYaw;
    }

    public Double getAprilTagPitch()
    {
        return this.atPitch;
    }

    public Double getAprilTagRoll()
    {
        return this.atRoll;
    }

    private boolean isRedTeam()
    {
        Alliance currentAlliance = this.driverStation.getAlliance();
        switch (currentAlliance)
        {
            case Red:
                return true;

            case Blue:
                return false;

            case Invalid:
            default:
                break;
        }

        // fallback: use the game-specific message.  if "red", use red, otherwise use blue
        return this.driverStation.getGameSpecificMessage().equalsIgnoreCase("red");
    }
}
