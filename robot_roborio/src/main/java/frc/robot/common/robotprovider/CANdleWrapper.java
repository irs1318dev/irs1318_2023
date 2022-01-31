package frc.robot.common.robotprovider;

import com.ctre.phoenix.led.*;

public class CANdleWrapper implements ICANdle
{
    private final CANdle wrappedObject;

    public CANdleWrapper(int deviceNumber)
    {
        this.wrappedObject = new CANdle(deviceNumber);
    }

    public void configBrightnessScalar(double brightness)
    {
        this.wrappedObject.configBrightnessScalar(brightness);
    }

    public void configLEDType(CANdleLEDStripType type)
    {
        CANdle.LEDStripType stripType = CANdle.LEDStripType.GRB;
        switch (type)
        {
            case GRB:
                stripType = CANdle.LEDStripType.GRB;
                break;

            case RGB:
                stripType = CANdle.LEDStripType.RGB;
                break;

            case BRG:
                stripType = CANdle.LEDStripType.BRG;
                break;

            case GRBW:
                stripType = CANdle.LEDStripType.GRBW;
                break;

            case RGBW:
                stripType = CANdle.LEDStripType.RGBW;
                break;

            case BRGW:
                stripType = CANdle.LEDStripType.BRGW;
                break;
        }

        this.wrappedObject.configLEDType(stripType);
    }

    public void configLOSBehavior(boolean disableWhenLOS)
    {
        this.wrappedObject.configLOSBehavior(disableWhenLOS);
    }

    public void configStatusLedState(boolean disableWhenRunning)
    {
        this.wrappedObject.configStatusLedState(disableWhenRunning);
    }

    public void configVBatOutput(CANdleVBatOutputMode mode)
    {
        CANdle.VBatOutputMode outputMode = CANdle.VBatOutputMode.Off;
        switch (mode)
        {
            case On:
                outputMode = CANdle.VBatOutputMode.On;
                break;

            case Off:
                outputMode = CANdle.VBatOutputMode.Off;
                break;

            case Modulated:
                outputMode = CANdle.VBatOutputMode.Modulated;
                break;
        }

        this.wrappedObject.configVBatOutput(outputMode);
    }

    public void modulateVBatOutput(double dutyCyclePercent)
    {
        this.wrappedObject.modulateVBatOutput(dutyCyclePercent);
    }

    public void setLEDs(int r, int g, int b)
    {
        this.wrappedObject.setLEDs(r, g, b);
    }

    public void setLEDs(int r, int g, int b, int w, int startIdx, int count)
    {
        this.wrappedObject.setLEDs(r, g, b, w, startIdx, count);
    }

    public void startTwinkleAnimation(int r, int g, int b, int w, double speed, int numLed, CANdleTwinklePercent divider)
    {
        TwinkleAnimation.TwinklePercent twinkleDivider = TwinkleAnimation.TwinklePercent.Percent100;
        switch (divider)
        {
            case Percent100:
                twinkleDivider = TwinkleAnimation.TwinklePercent.Percent100;
                break;

            case Percent88:
                twinkleDivider = TwinkleAnimation.TwinklePercent.Percent88;
                break;

            case Percent76:
                twinkleDivider = TwinkleAnimation.TwinklePercent.Percent76;
                break;

            case Percent64:
                twinkleDivider = TwinkleAnimation.TwinklePercent.Percent64;
                break;

            case Percent42:
                twinkleDivider = TwinkleAnimation.TwinklePercent.Percent42;
                break;

            case Percent30:
                twinkleDivider = TwinkleAnimation.TwinklePercent.Percent30;
                break;

            case Percent18:
                twinkleDivider = TwinkleAnimation.TwinklePercent.Percent18;
                break;

            case Percent6:
                twinkleDivider = TwinkleAnimation.TwinklePercent.Percent6;
                break;
        }

        this.wrappedObject.animate(new TwinkleAnimation(r, g, b, w, speed, numLed, twinkleDivider));
    }

    public void startTwinkleOffAnimation(int r, int g, int b, int w, double speed, int numLed, CANdleTwinklePercent divider)
    {
        TwinkleOffAnimation.TwinkleOffPercent twinkleDivider = TwinkleOffAnimation.TwinkleOffPercent.Percent100;
        switch (divider)
        {
            case Percent100:
                twinkleDivider = TwinkleOffAnimation.TwinkleOffPercent.Percent100;
                break;

            case Percent88:
                twinkleDivider = TwinkleOffAnimation.TwinkleOffPercent.Percent88;
                break;

            case Percent76:
                twinkleDivider = TwinkleOffAnimation.TwinkleOffPercent.Percent76;
                break;

            case Percent64:
                twinkleDivider = TwinkleOffAnimation.TwinkleOffPercent.Percent64;
                break;

            case Percent42:
                twinkleDivider = TwinkleOffAnimation.TwinkleOffPercent.Percent42;
                break;

            case Percent30:
                twinkleDivider = TwinkleOffAnimation.TwinkleOffPercent.Percent30;
                break;

            case Percent18:
                twinkleDivider = TwinkleOffAnimation.TwinkleOffPercent.Percent18;
                break;

            case Percent6:
                twinkleDivider = TwinkleOffAnimation.TwinkleOffPercent.Percent6;
                break;
        }

        this.wrappedObject.animate(new TwinkleOffAnimation(r, g, b, w, speed, numLed, twinkleDivider));
    }

    public void startStrobeAnimation(int r, int g, int b, int w, double speed, int numLed)
    {
        this.wrappedObject.animate(new StrobeAnimation(r, g, b, w, speed, numLed));
    }

    public void startSingleFadeAnimation(int r, int g, int b, int w, double speed, int numLed)
    {
        this.wrappedObject.animate(new SingleFadeAnimation(r, g, b, w, speed, numLed));
    }

    public void startRgbFadeAnimation(double brightness, double speed, int numLed)
    {
        this.wrappedObject.animate(new RgbFadeAnimation(brightness, speed, numLed));
    }

    public void startRainbowAnimation(double brightness, double speed, int numLed)
    {
        this.wrappedObject.animate(new RainbowAnimation(brightness, speed, numLed));
    }

    public void startLarsonAnimation(int r, int g, int b, int w, double speed, int numLed, CANdleLarsonBounceMode mode, int size)
    {
        LarsonAnimation.BounceMode bounceMode = LarsonAnimation.BounceMode.Front;
        switch (mode)
        {
            case Front:
                bounceMode = LarsonAnimation.BounceMode.Front;
                break;

            case Center:
                bounceMode = LarsonAnimation.BounceMode.Center;
                break;

            case Back:
                bounceMode = LarsonAnimation.BounceMode.Back;
                break;
        }

        this.wrappedObject.animate(new LarsonAnimation(r, g, b, w, speed, numLed, bounceMode, size));
    }

    public void startFireAnimation(double brightness, double speed, int numLed, double sparking, double cooling)
    {
        this.wrappedObject.animate(new FireAnimation(brightness, speed, numLed, sparking, cooling));
    }

    public void startColorFlowAnimation(int r, int g, int b, int w, double speed, int numLed, boolean forward)
    {
        this.wrappedObject.animate(new ColorFlowAnimation(r, g, b, w, speed, numLed, forward ? ColorFlowAnimation.Direction.Forward : ColorFlowAnimation.Direction.Backward));
    }
}