package frc.robot.mechanisms;

import frc.robot.*;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;

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
    private final ICANdle candle;

    private enum LightTransition
    {
        NoChange,
        TurnOn,
        TurnOff;
    }

    private boolean wasDisabled;

    @Inject
    public IndicatorLightManager(
        IRobotProvider provider)
    {
        this.candle = provider.getCANdle(ElectronicsConstants.INDICATOR_LIGHT_CANDLE_CAN_ID);
        this.candle.configLEDType(CANdleLEDStripType.GRB);
        this.candle.configVBatOutput(CANdleVBatOutputMode.Off);

        this.wasDisabled = true;
    }

    @Override
    public void readSensors()
    {
    }

    @Override
    public void update()
    {
        this.wasDisabled = false;
    }

    @Override
    public void stop()
    {
        this.wasDisabled = true;

        this.candle.startRainbowAnimation(1.0, 0.5, TuningConstants.CANDLE_TOTAL_NUMBER_LEDS);
    }

    /**
     * Check whether there is a transition required given the current state and the new state
     * @param needsUpdate
     * @param currentState the current state of the lights (on or off)
     * @param newState the new desired state of the lights (on or off)
     * @return NoChange if no state change is required, TurnOn if we need to turn the lights on, TurnOff if we need to turn the lights off
     */
    private LightTransition checkTransitionRequired(boolean needsUpdate, boolean currentState, boolean newState)
    {
        if (!needsUpdate &&
            currentState == newState)
        {
            return LightTransition.NoChange;
        }

        if (newState)
        {
            return LightTransition.TurnOn;
        }

        return LightTransition.TurnOff;
    }

    /**
     * Update a pair of light ranges to a certiain color (if TurnOn) or to the off (if TurnOff)
     * @param updateType whether to turn the lights to the on color, or the off color (NoChange not supported!!)
     * @param onColorRed the on color's red content [0, 255]
     * @param onColorGreen the on color's green content [0, 255]
     * @param onColorBlue the on color's blue content [0, 255]
     * @param onColorWhite the on color's white content [0, 255]
     * @param range1Start the beginning of the first range to modify
     * @param range1Count the size of the first range to modify
     * @param range2Start the beginning of the second range to modify
     * @param range2Count the size of the second range to modify
     * @return true if we turned on the light ranges, false if we turned off the light ranges
     */
    private boolean updateLightRanges(
        LightTransition updateType,
        int onColorRed,
        int onColorGreen,
        int onColorBlue,
        int onColorWhite,
        int range1Start,
        int range1Count,
        int range2Start,
        int range2Count)
    {
        boolean result;

        int r;
        int g;
        int b;
        int w;
        if (updateType == LightTransition.TurnOn)
        {
            result = true;
            r = onColorRed;
            g = onColorGreen;
            b = onColorBlue;
            w = onColorWhite;
        }
        else // if (updateType == LightTransition.TurnOff)
        {
            result = false;
            r = TuningConstants.INDICATOR_OFF_COLOR_RED;
            g = TuningConstants.INDICATOR_OFF_COLOR_GREEN;
            b = TuningConstants.INDICATOR_OFF_COLOR_BLUE;
            w = TuningConstants.INDICATOR_OFF_COLOR_WHITE;
        }

        this.candle.setLEDs(r, g, b, w, range1Start, range1Count);
        this.candle.setLEDs(r, g, b, w, range2Start, range2Count);

        return result;
    }
}
