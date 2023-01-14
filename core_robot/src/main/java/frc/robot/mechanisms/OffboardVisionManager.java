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

    private IDoubleSubscriber vDistanceSubsciber;
    private IDoubleSubscriber vAngleSubsciber;
    private IDoubleSubscriber aPointXSubsciber;
    private IDoubleSubscriber aPointYSubsciber;
    private IIntegerSubscriber heartbeatSubscriber;

    private Double vDistance;
    private Double vAngle;
    private Double aPointX;
    private Double aPointY;

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
        this.vDistanceSubsciber = this.networkTable.getDoubleSubscriber("v.distance");
        this.vAngleSubsciber = this.networkTable.getDoubleSubscriber("v.horizontalAngle");
        this.aPointXSubsciber = this.networkTable.getDoubleSubscriber("a.pointX");
        this.aPointYSubsciber = this.networkTable.getDoubleSubscriber("a.pointY");
        this.heartbeatSubscriber = this.networkTable.getIntegerSubscriber("v.heartbeat");

        this.vDistance = null;
        this.vAngle = null;
        this.aPointX = null;
        this.aPointY = null;

        this.missedHeartbeats = 0;
        this.prevHeartbeat = 0L;
    }

    /**
     * read all of the sensors for the mechanism that we will use in macros/autonomous mode and record their values
     */
    @Override
    public void readSensors()
    {
        this.vDistance = this.vDistanceSubsciber.get();
        this.vAngle = this.vAngleSubsciber.get();
        this.aPointX = this.aPointXSubsciber.get();
        this.aPointY = this.aPointYSubsciber.get();

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

        // reset if we couldn't find the vision target
        if (missedHeartbeatExceedsThreshold || this.vDistance < 0.0 || this.vAngle == TuningConstants.MAGIC_NULL_VALUE)
        {
            this.vDistance = null;
            this.vAngle = null;
        }

        // reset if we couldn't find the game piece
        if (missedHeartbeatExceedsThreshold || this.aPointX < 0.0 || this.aPointY < 0.0)
        {
            this.aPointX = null;
            this.aPointY = null;
        }

        this.logger.logNumber(LoggingKey.OffboardVisionTargetDistance, this.vDistance);
        this.logger.logNumber(LoggingKey.OffboardVisionTargetHorizontalAngle, this.vAngle);
        this.logger.logNumber(LoggingKey.OffboardVisionTagX, this.aPointX);
        this.logger.logNumber(LoggingKey.OffboardVisionTagY, this.aPointY);
    }

    @Override
    public void update()
    {
        boolean enableVision = !this.driver.getDigital(DigitalOperation.VisionForceDisable);
        boolean enableVideoStream = !this.driver.getDigital(DigitalOperation.VisionDisableStream);
        boolean enableGamePieceProcessing = this.driver.getDigital(DigitalOperation.VisionEnableGamePieceProcessing);
        boolean enableRetroreflectiveProcessing = this.driver.getDigital(DigitalOperation.VisionEnableRetroreflectiveProcessing);

        double visionProcessingMode = 0.0;
        if (enableVision)
        {
            if (enableRetroreflectiveProcessing)
            {
                visionProcessingMode = 1.0;
            }
            else if (enableGamePieceProcessing)
            {
                visionProcessingMode = 2.0;
            }
        }

        this.logger.logBoolean(LoggingKey.OffboardVisionEnableVision, enableVision);
        this.logger.logBoolean(LoggingKey.OffboardVisionEnableStream, enableVideoStream);
        this.logger.logNumber(LoggingKey.OffboardVisionEnableProcessing, visionProcessingMode);
    }

    @Override
    public void stop()
    {
        this.logger.logBoolean(LoggingKey.OffboardVisionEnableVision, false);
        this.logger.logBoolean(LoggingKey.OffboardVisionEnableStream, false);
        this.logger.logNumber(LoggingKey.OffboardVisionEnableProcessing, 0.0);
    }

    public Double getVisionTargetHorizontalAngle()
    {
        return this.vAngle;
    }

    public Double getVisionTargetDistance()
    {
        return this.vDistance;
    }

    public Double getGamePieceDistance()
    {
        return 0.0;
    }

    public Double getGamePieceHorizontalAngle()
    {
        return 0.0;
    }

    public Double getTagX()
    {
        return this.aPointX;
    }

    public Double getTagY()
    {
        return this.aPointY;
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
