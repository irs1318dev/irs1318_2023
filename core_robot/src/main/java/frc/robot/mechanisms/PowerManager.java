package frc.robot.mechanisms;

import frc.robot.*;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.DigitalOperation;
import frc.robot.driver.common.*;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * Power manager.
 * 
 * @author Will
 *
 */
@Singleton
public class PowerManager implements IMechanism
{
    public enum CurrentLimiting
    {
        Normal,
        OverCurrent,
        OverCurrentHigh,
    }

    private final IDriver driver;
    private final ITimer timer;
    private final LoggingManager logger;
    private final IPowerDistribution powerDistribution;

    private ComplementaryFilter batteryVoltageFilter;
    private double prevTime;
    private double currentFloatingAverage;
    private double[] currentSamples;

    /**
     * Initializes a new PowerManager
     * @param provider for obtaining electronics objects
     */
    @Inject
    public PowerManager(IDriver driver, ITimer timer, LoggingManager logger, IRobotProvider provider)
    {
        this.driver = driver;
        this.timer = timer;
        this.logger = logger;
        this.powerDistribution = provider.getPowerDistribution(ElectronicsConstants.POWER_DISTRIBUTION_CAN_ID, ElectronicsConstants.POWER_DISTRIBUTION_TYPE);
        this.batteryVoltageFilter = new ComplementaryFilter(0.4, 0.6, this.powerDistribution.getBatteryVoltage());

        this.prevTime = 0.0;
        this.currentFloatingAverage = 0.0;
        if (TuningConstants.POWER_TRACK_CURRENT)
        {
            this.currentSamples = new double[TuningConstants.POWER_OVERCURRENT_SAMPLES];
        }
    }

    public CurrentLimiting getCurrentLimitingValue()
    {
        if (this.currentFloatingAverage <= TuningConstants.POWER_OVERCURRENT_THRESHOLD)
        {
            return CurrentLimiting.Normal;
        }
        else if (this.currentFloatingAverage <= TuningConstants.POWER_OVERCURREHT_HIGH_THRESHOLD)
        {
            return CurrentLimiting.OverCurrent;
        }
        else
        {
            return CurrentLimiting.OverCurrentHigh;
        }
    }

    public double getCurrent(int pdpChannel)
    {
        return this.powerDistribution.getCurrent(pdpChannel);
    }

    @Override
    public void readSensors()
    {
        double currTime = this.timer.get();

        this.batteryVoltageFilter.update(this.powerDistribution.getBatteryVoltage());
        this.logger.logNumber(LoggingKey.PowerBatteryVoltage, this.batteryVoltageFilter.getValue());

        if (TuningConstants.POWER_TRACK_CURRENT)
        {
            int prevIndex = (int)(this.prevTime * TuningConstants.SAMPLES_PER_SECOND) % TuningConstants.POWER_OVERCURRENT_SAMPLES;
            int currIndex = (int)(currTime * TuningConstants.SAMPLES_PER_SECOND) % TuningConstants.POWER_OVERCURRENT_SAMPLES;

            int slots = currIndex - prevIndex + 1;
            if (slots < 0)
            {
                slots += TuningConstants.POWER_OVERCURRENT_SAMPLES;
            }

            double currCurrent = this.powerDistribution.getTotalCurrent();
            for (int i = 1; i < slots; i++)
            {
                int index = (prevIndex + i) % TuningConstants.POWER_OVERCURRENT_SAMPLES;
                this.currentFloatingAverage += ((currCurrent - this.currentSamples[index]) * TuningConstants.SAMPLE_DURATION) / TuningConstants.POWER_OVERCURRENT_TRACKING_DURATION;
                this.currentSamples[index] = currCurrent;
            }

            this.logger.logNumber(LoggingKey.PowerCurrent, currCurrent);
            this.logger.logNumber(LoggingKey.PowerCurrentFloatingAverage, this.currentFloatingAverage);
        }

        this.prevTime = currTime;
    }

    @Override
    public void update()
    {
        boolean enableVision = !this.driver.getDigital(DigitalOperation.VisionForceDisable);
        boolean enableGamePieceProcessing = this.driver.getDigital(DigitalOperation.VisionEnableGamePieceProcessing);
        boolean enableRetroreflectiveProcessing = this.driver.getDigital(DigitalOperation.VisionEnableRetroreflectiveProcessing);
        this.powerDistribution.setSwitchableChannel(enableVision && (enableGamePieceProcessing || enableRetroreflectiveProcessing));
    }

    @Override
    public void stop()
    {
        this.prevTime = 0.0;
        this.currentFloatingAverage = 0.0;
        if (TuningConstants.POWER_TRACK_CURRENT)
        {
            for (int i = 0; i < this.currentSamples.length; i++)
            {
                this.currentSamples[i] = 0.0;
            }
        }

        this.powerDistribution.setSwitchableChannel(false);
        this.batteryVoltageFilter.reset();
    }
}
