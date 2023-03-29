package frc.robot.mechanisms;

import frc.robot.*;
import frc.lib.driver.IDriver;
import frc.lib.mechanisms.IMechanism;
import frc.lib.robotprovider.*;
import frc.robot.driver.DigitalOperation;
import frc.lib.driver.descriptions.UserInputDevice;
import frc.robot.mechanisms.PowerManager.CurrentLimiting;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * Driver feedback manager
 *
 * This class manages things like controller rumbler and indicator lights on the robot.
 *
 */
@Singleton
public class DriverFeedbackManager implements IMechanism
{
    private final ICANdle candle;
    private final IDriverStation ds;
    private final IDriver driver;

    private final PowerManager powerMan;

    private LightMode currentStripMode;
    private LightMode currentCandleMode;

    private enum LightMode
    {
        Off,
        Purple,
        Yellow,
        Green,
        Red,
        Blue,
        Orange,
        Rainbow,
        YellowFlashing,
        PurpleTwinkling,
        GreenTwinkling,
        PurpleStrobe,
    }

    @Inject
    public DriverFeedbackManager(
        IDriver driver,
        IRobotProvider provider,
        PowerManager powerMan)
    {
        this.driver = driver;

        this.candle = provider.getCANdle(ElectronicsConstants.INDICATOR_LIGHT_CANDLE_CAN_ID);
        this.candle.configLEDType(CANdleLEDStripType.GRB);
        this.candle.configVBatOutput(CANdleVBatOutputMode.On);

        this.ds = provider.getDriverStation();
        this.powerMan = powerMan;

        this.currentStripMode = LightMode.Off;
        this.currentCandleMode = LightMode.Off;
    }

    @Override
    public void readSensors()
    {
    }

    @Override
    public void update()
    {
        RobotMode currentMode = this.ds.getMode();

        boolean isCurrentLimiting = this.powerMan.getCurrentLimitingValue() != CurrentLimiting.Normal;

        LightMode newStripMode;
        if (currentMode == RobotMode.Autonomous)
        {
            newStripMode = LightMode.Rainbow;
        }
        else if (this.driver.getDigital(DigitalOperation.CubeWantedFromSubstation) ||
            this.driver.getDigital(DigitalOperation.IntakeCube))
        {
            newStripMode = LightMode.Purple;
        }
        else if (this.driver.getDigital(DigitalOperation.ConeWantedFromSubstation) ||
            this.driver.getDigital(DigitalOperation.IntakeCone))
        {
            newStripMode = LightMode.Yellow;
        }
        else if (this.driver.getDigital(DigitalOperation.ForcePurpleStrobe))
        {
            newStripMode = LightMode.PurpleStrobe;
        }
        else if (isCurrentLimiting)
        {
            newStripMode = LightMode.Blue;
        }
        else
        {
            newStripMode = LightMode.Off;
        }

        LightMode newCandleMode;
        if (isCurrentLimiting)
        {
            newCandleMode = LightMode.Blue;
        }
        else
        {
            newCandleMode = LightMode.Off;
        }

        if (newStripMode != this.currentStripMode)
        {
            this.updateLightRange(
                this.currentStripMode,
                newStripMode,
                TuningConstants.LED_STRIP_LED_START,
                TuningConstants.LED_STRIP_LED_COUNT,
                TuningConstants.CANDLE_ANIMATION_SLOT_1);

            this.currentStripMode = newStripMode;
        }

        if (newCandleMode != this.currentCandleMode)
        {
            this.updateLightRange(
                this.currentCandleMode,
                newCandleMode,
                TuningConstants.CANDLE_LED_START,
                TuningConstants.CANDLE_LED_COUNT,
                TuningConstants.CANDLE_ANIMATION_SLOT_2);

            this.currentCandleMode = newCandleMode;
        }

        if (this.driver.getDigital(DigitalOperation.ForceLightDriverRumble))
        {
            this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Left, 0.5);
            this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Right, 0.5);
        }
        else
        {
            this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Left, 0.0);
            this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Right, 0.0);
        }
    }

    @Override
    public void stop()
    {
        this.updateLightRange(
            this.currentStripMode,
            LightMode.Off,
            TuningConstants.LED_STRIP_LED_START,
            TuningConstants.LED_STRIP_LED_COUNT,
            TuningConstants.CANDLE_ANIMATION_SLOT_1);

        this.updateLightRange(
            this.currentCandleMode,
            LightMode.Off,
            TuningConstants.CANDLE_LED_START,
            TuningConstants.CANDLE_LED_COUNT,
            TuningConstants.CANDLE_ANIMATION_SLOT_2);

        this.currentStripMode = LightMode.Off;
        this.currentCandleMode = LightMode.Off;

        this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Left, 0.0);
        this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Right, 0.0);
    }

    /**
     * Update a light range to a certiain mode
     * @param previousMode the previous mode for the lights
     * @param desiredMode the desired mode for the lights
     * @param rangeStart the beginning of the range to modify
     * @param rangeCount the size of the range to modify
     */
    private void updateLightRange(
        LightMode previousMode,
        LightMode desiredMode,
        int rangeStart,
        int rangeCount,
        int animationSlot)
    {
        // if we were in an animation previously, stop it:
        switch (previousMode)
        {
            case Rainbow:
            case YellowFlashing:
            case GreenTwinkling:
            case PurpleTwinkling:
            case PurpleStrobe:
                this.candle.stopAnimation(animationSlot);
                break;

            default:
                break;
        }

        // set based on our new mode:
        switch (desiredMode)
        {
            case Purple:
                this.candle.setLEDs(
                    TuningConstants.INDICATOR_PURPLE_COLOR_RED,
                    TuningConstants.INDICATOR_PURPLE_COLOR_GREEN,
                    TuningConstants.INDICATOR_PURPLE_COLOR_BLUE,
                    TuningConstants.INDICATOR_PURPLE_COLOR_WHITE,
                    rangeStart,
                    rangeCount);
                break;

            case Yellow:
                this.candle.setLEDs(
                    TuningConstants.INDICATOR_YELLOW_COLOR_RED,
                    TuningConstants.INDICATOR_YELLOW_COLOR_GREEN,
                    TuningConstants.INDICATOR_YELLOW_COLOR_BLUE,
                    TuningConstants.INDICATOR_YELLOW_COLOR_WHITE,
                    rangeStart,
                    rangeCount);
                break;

            case Green:
                this.candle.setLEDs(
                    TuningConstants.INDICATOR_GREEN_COLOR_RED,
                    TuningConstants.INDICATOR_GREEN_COLOR_GREEN,
                    TuningConstants.INDICATOR_GREEN_COLOR_BLUE,
                    TuningConstants.INDICATOR_GREEN_COLOR_WHITE,
                    rangeStart,
                    rangeCount);
                break;

            case Red:
                this.candle.setLEDs(
                    TuningConstants.INDICATOR_RED_COLOR_RED,
                    TuningConstants.INDICATOR_RED_COLOR_GREEN,
                    TuningConstants.INDICATOR_RED_COLOR_BLUE,
                    TuningConstants.INDICATOR_RED_COLOR_WHITE,
                    rangeStart,
                    rangeCount);
                break;

            case Blue:
                this.candle.setLEDs(
                    TuningConstants.INDICATOR_BLUE_COLOR_RED,
                    TuningConstants.INDICATOR_BLUE_COLOR_GREEN,
                    TuningConstants.INDICATOR_BLUE_COLOR_BLUE,
                    TuningConstants.INDICATOR_BLUE_COLOR_WHITE,
                    rangeStart,
                    rangeCount);
                break;

            case Orange:
                this.candle.setLEDs(
                    TuningConstants.INDICATOR_ORANGE_COLOR_RED,
                    TuningConstants.INDICATOR_ORANGE_COLOR_GREEN,
                    TuningConstants.INDICATOR_ORANGE_COLOR_BLUE,
                    TuningConstants.INDICATOR_ORANGE_COLOR_WHITE,
                    rangeStart,
                    rangeCount);
                break;

            case Rainbow:
                this.candle.startRainbowAnimation(
                    animationSlot,
                    TuningConstants.INDICATOR_RAINBOW_BRIGHTNESS,
                    TuningConstants.INDICATOR_RAINBOW_SPEED,
                    rangeCount,
                    TuningConstants.INDICATOR_RAINBOW_REVERSE_DIRECTION,
                    rangeStart);
                break;

            case YellowFlashing:
                this.candle.startStrobeAnimation(
                    animationSlot,
                    TuningConstants.INDICATOR_YELLOW_COLOR_RED,
                    TuningConstants.INDICATOR_YELLOW_COLOR_GREEN,
                    TuningConstants.INDICATOR_YELLOW_COLOR_BLUE,
                    TuningConstants.INDICATOR_YELLOW_COLOR_WHITE,
                    0.75,
                    rangeCount,
                    rangeStart);
                break;

            case PurpleStrobe:
                this.candle.startStrobeAnimation(
                    animationSlot,
                    TuningConstants.INDICATOR_PURPLE_COLOR_RED,
                    TuningConstants.INDICATOR_PURPLE_COLOR_GREEN,
                    TuningConstants.INDICATOR_PURPLE_COLOR_BLUE,
                    TuningConstants.INDICATOR_PURPLE_COLOR_WHITE,
                    0.75,
                    rangeCount,
                    rangeStart);

            case GreenTwinkling:
                this.candle.startTwinkleAnimation(
                    animationSlot,
                    TuningConstants.INDICATOR_GREEN_COLOR_RED,
                    TuningConstants.INDICATOR_GREEN_COLOR_GREEN,
                    TuningConstants.INDICATOR_GREEN_COLOR_BLUE,
                    TuningConstants.INDICATOR_GREEN_COLOR_WHITE,
                    0.75,
                    rangeCount,
                    CANdleTwinklePercent.Percent88,
                    rangeStart);
                break;

            case PurpleTwinkling:
                this.candle.startTwinkleAnimation(
                    animationSlot,
                    TuningConstants.INDICATOR_PURPLE_COLOR_RED,
                    TuningConstants.INDICATOR_PURPLE_COLOR_GREEN,
                    TuningConstants.INDICATOR_PURPLE_COLOR_BLUE,
                    TuningConstants.INDICATOR_PURPLE_COLOR_WHITE,
                    0.75,
                    rangeCount,
                    CANdleTwinklePercent.Percent88,
                    rangeStart);
                break;

            default:
            case Off:
                this.candle.setLEDs(
                    TuningConstants.INDICATOR_OFF_COLOR_RED,
                    TuningConstants.INDICATOR_OFF_COLOR_GREEN,
                    TuningConstants.INDICATOR_OFF_COLOR_BLUE,
                    TuningConstants.INDICATOR_OFF_COLOR_WHITE,
                    rangeStart,
                    rangeCount);
                break;
        }
    }
}
