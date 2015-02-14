package org.usfirst.frc.team1318.robot.Elevator;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;
import org.usfirst.frc.team1318.robot.HardwareConstants;
import org.usfirst.frc.team1318.robot.Common.SmartDashboardLogger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;

public class ElevatorComponent
{
    //private final DigitalInput throughBeamSensor;
    private final Talon motor;
    private final Encoder encoder;
    private final DigitalInput topLimitSwtich;
    private final DigitalInput bottomLimitSwitch;

    public static final String THROUGH_BEAM_SENSOR_LOG_KEY = "e.throughBeamSensor";
    public static final String MOTOR_POWER_LOG_KEY = "e.motorPower";
    public static final String ENCODER_DISTANCE_LOG_KEY = "e.encoderDistance";
    public static final String ENCODER_TICKS_LOG_KEY = "e.encoderTicks";
    public static final String ENCODER_VELOCITY_LOG_KEY = "e.encoderVelocity";
    public static final String TOP_LIMIT_SWITCH_LOG_KEY = "e.topHallEffectSensor";
    public static final String BOTTOM_LIMIT_SWITCH_LOG_KEY = "e.bottomHallEffectSendor";

    public ElevatorComponent()
    {
        this.motor = new Talon(ElectronicsConstants.ELEVATOR_TALON_CHANNEL);

        this.encoder = new Encoder(
            ElectronicsConstants.ELEVATOR_ENCODER_CHANNEL_A,
            ElectronicsConstants.ELEVATOR_ENCODER_CHANNEL_B);

        this.encoder.setDistancePerPulse(HardwareConstants.ELEVATOR_PULSE_DISTANCE);

        //this.throughBeamSensor = new DigitalInput(ElectronicsConstants.ELEVATOR_THROUGH_BEAM_SENSOR_CHANNEL);

        this.topLimitSwtich = new DigitalInput(ElectronicsConstants.ELEVATOR_TOP_LIMIT_SWITCH_CHANNEL);
        this.bottomLimitSwitch = new DigitalInput(ElectronicsConstants.ELEVATOR_BOTTOM_LIMIT_SWITCH_CHANNEL);
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
     * Gets the current value of the Through Beam Sensor 
     * @return true for (TODO), otherwise false 
     */
    public boolean getThroughBeamSensor()
    {
        //boolean value = this.throughBeamSensor.get();
        //SmartDashboardLogger.putBoolean(ElevatorComponent.THROUGH_BEAM_SENSOR_LOG_KEY, value);
        return false;//value;
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
}
