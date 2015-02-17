package org.usfirst.frc.team1318.robot.Elevator;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;
import org.usfirst.frc.team1318.robot.HardwareConstants;
import org.usfirst.frc.team1318.robot.Common.SmartDashboardLogger;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Talon;

public class ElevatorComponent
{
    private final AnalogInput throughBeamSensor;
    private final Talon motor;
    private final Encoder encoder;
    private final DigitalInput topLimitSwtich;
    private final DigitalInput bottomLimitSwitch;
    private final Relay limitSwitchRelay;
    private final Relay throughBeamRelay;

    public static final String MOTOR_POWER_LOG_KEY = "e.motorPower";
    public static final String ENCODER_DISTANCE_LOG_KEY = "e.encoderDistance";
    public static final String ENCODER_TICKS_LOG_KEY = "e.encoderTicks";
    public static final String ENCODER_VELOCITY_LOG_KEY = "e.encoderVelocity";
    public static final String TOP_LIMIT_SWITCH_LOG_KEY = "e.topLimitSwitch";
    public static final String BOTTOM_LIMIT_SWITCH_LOG_KEY = "e.bottomLimitSwitch";
    public static final String THROUGH_BEAM_SENSOR_ANALOG_LOG_KEY = "e.throughBeamAnalog";
    public static final String THROUGH_BEAM_SENSOR_BOOLEAN_LOG_KEY = "e.throughBeamSensorBoolean";
    public static final String THROUGH_BEAM_RELAY_LOG_KEY = "e.throughBeamRelay";
    public static final String LIMIT_SWITCH_RELAY_LOG_KEY = "e.limitSwitchRelay";

    public ElevatorComponent()
    {
        this.motor = new Talon(ElectronicsConstants.ELEVATOR_TALON_CHANNEL);

        this.encoder = new Encoder(
            ElectronicsConstants.ELEVATOR_ENCODER_CHANNEL_A,
            ElectronicsConstants.ELEVATOR_ENCODER_CHANNEL_B);

        this.encoder.setDistancePerPulse(HardwareConstants.ELEVATOR_PULSE_DISTANCE);

        this.throughBeamSensor = new AnalogInput(ElectronicsConstants.ELEVATOR_THROUGH_BEAM_SENSOR_CHANNEL);

        this.topLimitSwtich = new DigitalInput(ElectronicsConstants.ELEVATOR_TOP_LIMIT_SWITCH_CHANNEL);
        this.bottomLimitSwitch = new DigitalInput(ElectronicsConstants.ELEVATOR_BOTTOM_LIMIT_SWITCH_CHANNEL);

        this.throughBeamRelay = new Relay(ElectronicsConstants.ELEVATOR_THROUGH_BEAM_RELAY_CHANNEL, Relay.Direction.kReverse);
        this.limitSwitchRelay = new Relay(ElectronicsConstants.ELEVATOR_LIMIT_SWITCH_RELAY_CHANNEL, Relay.Direction.kReverse);
    }

    /**
     * Gets the net distance that the encoder has rotated in (TODO) 
     * @return the net distance the encoder has rotated 
     */
    public double getEncoderDistance()
    {
        double value = this.encoder.getDistance();
        SmartDashboardLogger.putNumber(ElevatorComponent.ENCODER_DISTANCE_LOG_KEY, value);
        return value;
    }

    /**
     * Gets the net number of ticks that the encoder has rotated 
     * @return the net number of ticks the encoder has rotated 
     */
    public int getEncoderTicks()
    {
        int value = this.encoder.get();
        SmartDashboardLogger.putNumber(ElevatorComponent.ENCODER_TICKS_LOG_KEY, value);
        return value;
    }

    /**
     * Get the velocity that the encoder is rotating at in (TODO) 
     * @return the velocity the encoder is rotating at 
     */
    public double getEncoderVelocity()
    {
        double value = this.encoder.getRate();
        SmartDashboardLogger.putNumber(ElevatorComponent.ENCODER_VELOCITY_LOG_KEY, value);
        return value;
    }

    /**
     * Sets the power level of the elevator motor 
     * @param powerLevel value to set the power level to 
     */
    public void setMotorPowerLevel(double powerLevel)
    {
        this.motor.set(powerLevel);
        SmartDashboardLogger.putNumber(ElevatorComponent.MOTOR_POWER_LOG_KEY, powerLevel);
    }

    /**
     * Gets the current value of the through beam sensor 
     * @return current voltage of the through beam sensor 
     */
    public double getThroughBeamVoltage()
    {
        //        double value = throughBeamSensor.getVoltage();
        double value = 0;
        SmartDashboardLogger.putNumber(THROUGH_BEAM_SENSOR_ANALOG_LOG_KEY, value);
        return value;
    }

    /**
     * Gets a boolean representing whether or not the through beam sensor is broken 
     * @return true for broken, otherwise false 
     */
    public boolean getThroughBeamBroken()
    {
        boolean valueBool = (throughBeamSensor.getVoltage() < 2.5);
        //        boolean valueBool = false;
        SmartDashboardLogger.putBoolean(THROUGH_BEAM_SENSOR_BOOLEAN_LOG_KEY, valueBool);
        return valueBool;
    }

    /**
     * Gets the current value of the Limit Switch at the top of the elevator 
     * @return true for pressed, otherwise false 
     */
    public boolean getTopLimitSwitchValue()
    {
        boolean value = this.topLimitSwtich.get();
        SmartDashboardLogger.putBoolean(ElevatorComponent.TOP_LIMIT_SWITCH_LOG_KEY, value);
        return value;
    }

    /**
     * Gets the current value of the LimitSwitch at the bottom of the elevator 
     * @return true for pressed, otherwise false 
     */
    public boolean getBottomLimitSwitchValue()
    {
        boolean value = this.bottomLimitSwitch.get();
        SmartDashboardLogger.putBoolean(ElevatorComponent.BOTTOM_LIMIT_SWITCH_LOG_KEY, value);
        return value;
    }

    public void setThroughBeamRelayValue(boolean value)
    {
        if (value)
        {
            throughBeamRelay.setDirection(Relay.Direction.kForward);
        }
        else
        {
            throughBeamRelay.setDirection(Relay.Direction.kReverse);
        }
        SmartDashboardLogger.putBoolean(ElevatorComponent.THROUGH_BEAM_RELAY_LOG_KEY, value);
    }

    public void setLimitSwitchRelayValue(boolean value)
    {
        if (value)
        {
            //limitSwitchRelay.setDirection(Relay.Direction.kForward);
            limitSwitchRelay.set(Relay.Value.kOn);
        }
        else
        {
            //            limitSwitchRelay.setDirection(Relay.Direction.kReverse);
            limitSwitchRelay.set(Relay.Value.kOff);
        }
        SmartDashboardLogger.putBoolean(ElevatorComponent.LIMIT_SWITCH_RELAY_LOG_KEY, value);
    }
}
