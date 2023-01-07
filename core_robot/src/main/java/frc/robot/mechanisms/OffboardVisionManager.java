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
    private IDoubleSubscriber gDistanceSubsciber;
    private IDoubleSubscriber gAngleSubsciber;
    private IIntegerSubscriber heartbeatSubscriber;

    private Double vDistance;
    private Double vAngle;
    private Double gDistance;
    private Double gAngle;

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
        this.gDistanceSubsciber = this.networkTable.getDoubleSubscriber("g.distance");
        this.gAngleSubsciber = this.networkTable.getDoubleSubscriber("g.horizontalAngle");
        this.heartbeatSubscriber = this.networkTable.getIntegerSubscriber("v.heartbeat");

        this.vDistance = null;
        this.vAngle = null;
        this.gDistance = null;
        this.gAngle = null;

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
        this.gDistance = this.gDistanceSubsciber.get();
        this.gAngle = this.gAngleSubsciber.get();

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
        if (missedHeartbeatExceedsThreshold || this.gDistance < 0.0 || this.gAngle == TuningConstants.MAGIC_NULL_VALUE)
        {
            this.gDistance = null;
            this.gAngle = null;
        }

        this.logger.logNumber(LoggingKey.OffboardVisionTargetDistance, this.vDistance);
        this.logger.logNumber(LoggingKey.OffboardVisionTargetHorizontalAngle, this.vAngle);
        this.logger.logNumber(LoggingKey.OffboardVisionGamePieceDistance, this.gDistance);
        this.logger.logNumber(LoggingKey.OffboardVisionGamePieceHorizontalAngle, this.gAngle);
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
                if (this.isRedTeam())
                {
                    visionProcessingMode = 2.0; // 2.0 means red team
                }
                else
                {
                    visionProcessingMode = 3.0; // 3.0 means blue team
                }
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

    public Double getGamePieceHorizontalAngle()
    {
        return this.gAngle;
    }

    public Double getGamePieceDistance()
    {
        return this.gDistance;
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
