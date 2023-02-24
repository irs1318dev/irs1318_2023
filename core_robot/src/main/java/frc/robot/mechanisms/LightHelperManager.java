package frc.robot.mechanisms;

import frc.robot.*;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.DigitalOperation;
import frc.robot.driver.common.IDriver;
import frc.robot.driver.controltasks.ChargeStationTask;
import frc.robot.driver.controltasks.VisionTurningTask;
import frc.robot.driver.controltasks.ChargeStationTask.State;

import com.google.inject.Inject;
import com.google.inject.Singleton;

@Singleton
public class LightHelperManager implements IMechanism{

    private final ArmMechanism arm;
    private final ChargeStationTask chargeStationTask; 
    private final VisionTurningTask visionTurningTask;
    private final ICANdle candle;
    private final IDriver driver;

    private LightMode currentMode;
    
    private enum LightMode
    {
        Off,
        Purple,
        Yellow,
        Green,
        Red,
        Blue,
        Orange,
        Rainbow
    }

    public LightHelperManager(IRobotProvider provider, IDriver driver, ArmMechanism arm, ChargeStationTask chargeStationTask, VisionTurningTask visionTurningTask)
    {
        this.candle = provider.getCANdle(ElectronicsConstants.INDICATOR_LIGHT_CANDLE_CAN_ID, ElectronicsConstants.CANIVORE_NAME);
        this.candle.configLEDType(CANdleLEDStripType.GRB);
        this.candle.configVBatOutput(CANdleVBatOutputMode.Off);

        this.driver = driver;
        
        this.arm = arm;
        this.chargeStationTask = chargeStationTask;
        this.visionTurningTask = visionTurningTask;

        this.currentMode = LightMode.Off;
    }
    
    @Override
    public void readSensors() {
        
    }

    @Override
    public void update() {
        LightMode newLightMode;
        
        if (this.driver.getDigital(DigitalOperation.CubeInSubstation))
        {
            newLightMode = LightMode.Purple;
        }

        else if (this.driver.getDigital(DigitalOperation.ConeInSubstation))
        {
            newLightMode = LightMode.Yellow;
        }

        else if ((this.driver.getDigital(DigitalOperation.IntakeIn) || this.driver.getDigital(DigitalOperation.IntakeGrab)) && arm.isThroughBeamBroken())
        {
            newLightMode = LightMode.Blue;
        }

        else if ((this.driver.getDigital(DigitalOperation.IntakeIn) || this.driver.getDigital(DigitalOperation.IntakeGrab)) && !arm.isThroughBeamBroken())
        {
            newLightMode = LightMode.Orange;
        }

        else if (visionTurningTask.isAprilTag())
        {
            newLightMode = LightMode.Green; //Can See AprilTag
        }

        else if (!visionTurningTask.isAprilTag())
        {
            newLightMode = LightMode.Red; //Can not see AprilTag
        }

        else if (chargeStationTask.currentState == State.Completed)
        {
            newLightMode = LightMode.Rainbow; //Balanced on Charge Station
        }

        else
        {
            newLightMode = LightMode.Off;
        }

        if (newLightMode != this.currentMode)
        {
            this.updateLightRange(
                this.currentMode,
                newLightMode,
                0,
                TuningConstants.CANDLE_TOTAL_NUMBER_LEDS,
                TuningConstants.CANDLE_ANIMATION_SLOT_1);

            this.currentMode = newLightMode;
        }
    }

    @Override
    public void stop() {
        this.updateLightRange(
            this.currentMode,
            LightMode.Off,
            0,
            TuningConstants.CANDLE_TOTAL_NUMBER_LEDS,
            TuningConstants.CANDLE_ANIMATION_SLOT_1);

        this.currentMode = LightMode.Off;
    }

    private void updateLightRange(
        LightMode previousMode,
        LightMode desiredMode,
        int rangeStart,
        int rangeCount,
        int animationSlot)
    {
    
    switch (previousMode)
    {
        case Rainbow:
            this.candle.stopAnimation(animationSlot);
            break;
        
        default:
            break;
    }
    
    switch (desiredMode)
    {
        case Purple:
            this.candle.setLEDs(
                TuningConstants.INDICATOR_PURPLE_COLOR_RED, 
                TuningConstants.INDICATOR_PURPLE_COLOR_GREEN,
                TuningConstants.INDICATOR_PURPLE_COLOR_BLUE,
                TuningConstants.INDICATOR_PURPLE_COLOR_WHITE,
                rangeCount, 
                animationSlot);
        break;

        case Yellow:
            this.candle.setLEDs(
                TuningConstants.INDICATOR_YELLOW_COLOR_RED,
                TuningConstants.INDICATOR_YELLOW_COLOR_GREEN,
                TuningConstants.INDICATOR_YELLOW_COLOR_BLUE,
                TuningConstants.INDICATOR_YELLOW_COLOR_WHITE,
                rangeCount, animationSlot);
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
                rangeCount, 
                animationSlot);
        break;

        case Blue:
            this.candle.setLEDs(
                TuningConstants.INDICATOR_BLUE_COLOR_RED,
                TuningConstants.INDICATOR_BLUE_COLOR_GREEN,
                TuningConstants.INDICATOR_BLUE_COLOR_BLUE,
                TuningConstants.INDICATOR_GREEN_COLOR_WHITE, 
                rangeCount, 
                animationSlot);
        break;

        case Orange:
            this.candle.setLEDs(
                TuningConstants.INDICATOR_ORANGE_COLOR_RED, 
                TuningConstants.INDICATOR_ORANGE_COLOR_GREEN, 
                TuningConstants.INDICATOR_ORANGE_COLOR_BLUE,
                TuningConstants.INDICATOR_ORANGE_COLOR_WHITE,
                rangeCount,
                animationSlot);
        break;

        case Rainbow:
            this.candle.startRainbowAnimation(
                ElectronicsConstants.animSlot, 
                TuningConstants.brightness, 
                TuningConstants.speed, 
                TuningConstants.numLed, 
                TuningConstants.reverseDirection, 
                TuningConstants.ledOffset);
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