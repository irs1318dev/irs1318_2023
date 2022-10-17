package frc.robot.mechanisms;

import frc.robot.*;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.DigitalOperation;
import frc.robot.driver.common.*;
import frc.robot.driver.common.descriptions.UserInputDevice;

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
    // private final ICANdle candle;
    // private final IDriverStation ds;
    private final IDriver driver;

    // private LightMode currentMode;

    // private enum LightMode
    // {
    //     Off,
    //     Red,
    //     Yellow,
    //     YellowFlashing,
    //     Green,
    //     Rainbow,
    //     PurpleTwinkling,
    // }

    @Inject
    public DriverFeedbackManager(
        IDriver driver,
        IRobotProvider provider)
    {
        this.driver = driver;

        // this.candle = provider.getCANdle(ElectronicsConstants.INDICATOR_LIGHT_CANDLE_CAN_ID, ElectronicsConstants.CANIVORE_NAME);
        // this.candle.configLEDType(CANdleLEDStripType.GRB);
        // this.candle.configVBatOutput(CANdleVBatOutputMode.Off);

        // this.ds = provider.getDriverStation();

        // this.currentMode = LightMode.Off;
    }

    @Override
    public void readSensors()
    {
    }

    @Override
    public void update()
    {
        // LightMode newLightMode;
        // if (this.ds.getMode() == RobotMode.Autonomous) 
        // {
        //     newLightMode = LightMode.PurpleTwinkling;
        // }
        // else
        // {
        //     newLightMode = LightMode.Green;
        // }

        // if (newLightMode != this.currentMode)
        // {
        //     this.updateLightRange(
        //         this.currentMode,
        //         newLightMode,
        //         0,
        //         TuningConstants.CANDLE_TOTAL_NUMBER_LEDS,
        //         TuningConstants.CANDLE_ANIMATION_SLOT_1);

        //     this.currentMode = newLightMode;
        // }

        if (this.driver.getDigital(DigitalOperation.ForceLightDriverRumble))
        {
            this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Left, 0.25);
            this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Right, 0.25);
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
        // this.updateLightRange(
        //     this.currentMode,
        //     LightMode.Rainbow,
        //     0,
        //     TuningConstants.CANDLE_TOTAL_NUMBER_LEDS,
        //     TuningConstants.CANDLE_ANIMATION_SLOT_1);

        // this.currentMode = LightMode.Rainbow;
        this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Left, 0.0);
        this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Right, 0.0);
    }

    // /**
    //  * Update a light range to a certiain mode
    //  * @param previousMode the previous mode for the lights
    //  * @param desiredMode the desired mode for the lights
    //  * @param rangeStart the beginning of the range to modify
    //  * @param rangeCount the size of the range to modify
    //  */
    // private void updateLightRange(
    //     LightMode previousMode,
    //     LightMode desiredMode,
    //     int rangeStart,
    //     int rangeCount,
    //     int animationSlot)
    // {
    //     // if we were in an animation previously, stop it:
    //     switch (previousMode)
    //     {
    //         case Rainbow:
    //         case YellowFlashing:
    //         case PurpleTwinkling:
    //             this.candle.stopAnimation(animationSlot);
    //             break;

    //         default:
    //             break;
    //     }

    //     // set based on our new mode:
    //     switch (desiredMode)
    //     {
    //         case Rainbow:
    //             this.candle.startRainbowAnimation(
    //                 animationSlot,
    //                 1.0,
    //                 0.25,
    //                 rangeCount,
    //                 false,
    //                 rangeStart);
    //             break;

    //         case YellowFlashing:
    //             this.candle.startStrobeAnimation(
    //                 animationSlot,
    //                 TuningConstants.INDICATOR_YELLOW_COLOR_RED,
    //                 TuningConstants.INDICATOR_YELLOW_COLOR_GREEN,
    //                 TuningConstants.INDICATOR_YELLOW_COLOR_BLUE,
    //                 TuningConstants.INDICATOR_YELLOW_COLOR_WHITE,
    //                 0.75,
    //                 rangeStart,
    //                 rangeCount);

    //         case Green:
    //             this.candle.setLEDs(
    //                 TuningConstants.INDICATOR_GREEN_COLOR_RED,
    //                 TuningConstants.INDICATOR_GREEN_COLOR_GREEN,
    //                 TuningConstants.INDICATOR_GREEN_COLOR_BLUE,
    //                 TuningConstants.INDICATOR_GREEN_COLOR_WHITE,
    //                 rangeStart,
    //                 rangeCount);
    //             break;

    //         case Yellow:
    //             this.candle.setLEDs(
    //                 TuningConstants.INDICATOR_YELLOW_COLOR_RED,
    //                 TuningConstants.INDICATOR_YELLOW_COLOR_GREEN,
    //                 TuningConstants.INDICATOR_YELLOW_COLOR_BLUE,
    //                 TuningConstants.INDICATOR_YELLOW_COLOR_WHITE,
    //                 rangeStart,
    //                 rangeCount);
    //             break;

    //         case Red:
    //             this.candle.setLEDs(
    //                 TuningConstants.INDICATOR_RED_COLOR_RED,
    //                 TuningConstants.INDICATOR_RED_COLOR_GREEN,
    //                 TuningConstants.INDICATOR_RED_COLOR_BLUE,
    //                 TuningConstants.INDICATOR_RED_COLOR_WHITE,
    //                 rangeStart,
    //                 rangeCount);
    //             break;

    //         case PurpleTwinkling:
    //             this.candle.startTwinkleAnimation(
    //                 animationSlot,
    //                 TuningConstants.INDICATOR_PURPLE_RED,
    //                 TuningConstants.INDICATOR_PURPLE_GREEN,
    //                 TuningConstants.INDICATOR_PURPLE_BLUE,
    //                 TuningConstants.INDICATOR_PURPLE_WHITE,
    //                 0.75,
    //                 rangeCount,
    //                 CANdleTwinklePercent.Percent88,
    //                 rangeStart);
    //             break;

    //         default:
    //         case Off:
    //             this.candle.setLEDs(
    //                 TuningConstants.INDICATOR_OFF_COLOR_RED,
    //                 TuningConstants.INDICATOR_OFF_COLOR_GREEN,
    //                 TuningConstants.INDICATOR_OFF_COLOR_BLUE,
    //                 TuningConstants.INDICATOR_OFF_COLOR_WHITE,
    //                 rangeStart,
    //                 rangeCount);
    //             break;
    //     }
    // }
}
