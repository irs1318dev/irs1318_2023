package frc.robot.mechanisms;

import frc.robot.*;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.common.Driver;
import frc.robot.vision.common.VisionProcessingState;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * Indicator Light manager
 * 
 * This class manages indicator lights on the robot.
 * 
 */
@Singleton
public class IndicatorLightManager implements IMechanism
{
    private static final double FlashingFrequency = 0.2;
    private static final double FlashingComparisonFrequency = IndicatorLightManager.FlashingFrequency / 2.0;

    private final VisionManager visionManager;
    private final ITimer timer;
    private final IDashboardLogger logger;

    private final IRelay visionIndicator;

    private LightMode visionMode;

    /**
     * Initializes a new IndicatorLightManager
     * @param provider for obtaining electronics objects
     * @param timer to use
     * @param grabberMechanism for grabber reference
     * @param visionManager for vision reference
     */
    @Inject
    public IndicatorLightManager(
        IRobotProvider provider,
        IDashboardLogger logger,
        ITimer timer,
        VisionManager visionManager)
    {
        this.visionManager = visionManager;
        this.timer = timer;
        this.logger = logger;

        this.visionIndicator = provider.getRelay(ElectronicsConstants.INDICATOR_VISION_RELAY_CHANNEL);

        this.visionMode = LightMode.Off;
    }

    /**
     * set the driver that the mechanism should use
     * @param driver to use
     */
    @Override
    public void setDriver(Driver driver)
    {
    }

    @Override
    public void readSensors()
    {
    }

    /**
     * calculate the various outputs to use based on the inputs and apply them to the outputs for the relevant mechanism
     */
    @Override
    public void update()
    {
        if (this.visionManager != null &&
            this.visionManager.getState() != VisionProcessingState.Disabled &&
            this.visionManager.getCenter() != null)
        {
            if (this.visionManager.getMeasuredDistance() > TuningConstants.INDICATOR_LIGHT_VISION_CONSIDERATION_DISTANCE_RANGE)
            {
                this.visionMode = LightMode.Off;
            }
            else if (Math.abs(this.visionManager.getMeasuredAngle() - this.visionManager.getDesiredAngle()) < TuningConstants.INDICATOR_LIGHT_VISION_ACCEPTABLE_ANGLE_RANGE)
            {
                this.visionMode = LightMode.On;
            }
            else
            {
                this.visionMode = LightMode.Flashing;
            }
        }
        else
        {
            this.visionMode = LightMode.Off;
        }

        this.logger.logBoolean("ind", "vision", this.visionMode != LightMode.Off);
        this.controlLight(this.visionIndicator, this.visionMode);
    }

    /**
     * stop the relevant component
     */
    @Override
    public void stop()
    {
        this.visionIndicator.set(RelayValue.Off);
    }

    private void controlLight(IRelay indicatorLight, LightMode mode)
    {
        if (mode == LightMode.On)
        {
            indicatorLight.set(RelayValue.Forward);
        }
        else if (mode == LightMode.Off)
        {
            indicatorLight.set(RelayValue.Off);
        }
        else
        {
            double currentTime = this.timer.get();
            if (currentTime % IndicatorLightManager.FlashingFrequency >= IndicatorLightManager.FlashingComparisonFrequency)
            {
                indicatorLight.set(RelayValue.Forward);
            }
            else
            {
                indicatorLight.set(RelayValue.Off);
            }
        }
    }

    private enum LightMode
    {
        Off,
        Flashing,
        On,
    }
}
