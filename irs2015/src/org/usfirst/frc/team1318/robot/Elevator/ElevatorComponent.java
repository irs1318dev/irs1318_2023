package org.usfirst.frc.team1318.robot.Elevator;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;
import org.usfirst.frc.team1318.robot.HardwareConstants;
import org.usfirst.frc.team1318.robot.Common.SmartDashboardLogger;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;

public class ElevatorComponent
{
    private static final String MOTOR_POWER_LOG_KEY = "e.motorPower";
    private static final String ENCODER_DISTANCE_LOG_KEY = "e.encoderDistance";
    private static final String ENCODER_TICKS_LOG_KEY = "e.encoderTicks";
    private static final String ENCODER_VELOCITY_LOG_KEY = "e.encoderVelocity";
    private static final String ENCODER_ZERO_OFFSET_LOG_KEY = "e.encoderZeroOffset";
    private static final String TOP_LIMIT_SWITCH_LOG_KEY = "e.topLimitSwitch";
    private static final String BOTTOM_LIMIT_SWITCH_LOG_KEY = "e.bottomLimitSwitch";
    private static final String THROUGH_BEAM_SENSOR_ANALOG_LOG_KEY = "e.throughBeamAnalog";
    private static final String THROUGH_BEAM_SENSOR_BOOLEAN_LOG_KEY = "e.throughBeamSensorBoolean";
    private static final String THROUGH_BEAM_RELAY_LOG_KEY = "e.throughBeamRelay";
    private static final String LIMIT_SWITCH_RELAY_LOG_KEY = "e.limitSwitchRelay";

    private final AnalogInput throughBeamSensor;
    private final Talon motor;
    private final Encoder encoder;
    private final DoubleSolenoid canStabilizer;
    private final DigitalInput topLimitSwtich;
    private final DigitalInput bottomLimitSwitch;
    private final Solenoid limitSwitchLight;
    private final Solenoid throughBeamLightUpper;
    private final Solenoid throughBeamLightLeft;
    private final Solenoid throughBeamLightRight;

    private double encoderZeroOffset;

    public ElevatorComponent()
    {
        this.motor = new Talon(ElectronicsConstants.ELEVATOR_TALON_CHANNEL);

        this.encoder = new Encoder(
            ElectronicsConstants.ELEVATOR_ENCODER_CHANNEL_A,
            ElectronicsConstants.ELEVATOR_ENCODER_CHANNEL_B);

        this.canStabilizer = new DoubleSolenoid(ElectronicsConstants.PCM_B_MODULE, ElectronicsConstants.ELEVATOR_CAN_STABILIZER_EXTEND,
            ElectronicsConstants.ELEVATOR_CAN_STABILIZER_RETRACT);

        this.encoder.setDistancePerPulse(HardwareConstants.ELEVATOR_PULSE_DISTANCE);

        this.throughBeamSensor = new AnalogInput(ElectronicsConstants.ELEVATOR_THROUGH_BEAM_SENSOR_CHANNEL);

        this.topLimitSwtich = new DigitalInput(ElectronicsConstants.ELEVATOR_TOP_LIMIT_SWITCH_CHANNEL);
        this.bottomLimitSwitch = new DigitalInput(ElectronicsConstants.ELEVATOR_BOTTOM_LIMIT_SWITCH_CHANNEL);

        this.limitSwitchLight = new Solenoid(ElectronicsConstants.PCM_B_MODULE, ElectronicsConstants.ELEVATOR_LIMIT_SWITCH_LIGHT_CHANNEL);
        this.throughBeamLightUpper = new Solenoid(ElectronicsConstants.PCM_B_MODULE,
            ElectronicsConstants.ELEVATOR_THROUGH_BEAM_LIGHT_CHANNEL_UPPER);
        this.throughBeamLightLeft = new Solenoid(ElectronicsConstants.PCM_B_MODULE,
            ElectronicsConstants.ELEVATOR_THROUGH_BEAM_LIGHT_CHANNEL_LEFT);
        this.throughBeamLightRight = new Solenoid(ElectronicsConstants.PCM_B_MODULE,
            ElectronicsConstants.ELEVATOR_THROUGH_BEAM_LIGHT_CHANNEL_RIGHT);

        this.encoderZeroOffset = 0.0;
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
     * Sets the encoder zero offset
     * @param encoderZeroOffset to apply
     */
    public void setEncoderZeroOffset(double encoderZeroOffset)
    {
        this.encoderZeroOffset = encoderZeroOffset;
        SmartDashboardLogger.putNumber(ElevatorComponent.ENCODER_ZERO_OFFSET_LOG_KEY, encoderZeroOffset);
    }

    /**
     * Gets the currently set encoder zero offset
     * @return the encoder zero offset
     */
    public double getEncoderZeroOffset()
    {
        return this.encoderZeroOffset;
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

    public void setCanStabilizer(boolean value)
    {
        if (value)
        {
            this.canStabilizer.set(DoubleSolenoid.Value.kForward);
        }
        else
        {
            this.canStabilizer.set(DoubleSolenoid.Value.kReverse);
        }
    }

    /**
     * Gets the current value of the through beam sensor 
     * @return current voltage of the through beam sensor 
     */
    public double getThroughBeamVoltage()
    {
        double value = this.throughBeamSensor.getVoltage();
        SmartDashboardLogger.putNumber(ElevatorComponent.THROUGH_BEAM_SENSOR_ANALOG_LOG_KEY, value);
        return value;
    }

    /**
     * Gets a boolean representing whether or not the through beam sensor is broken 
     * @return true for broken, otherwise false 
     */
    public boolean getThroughBeamBroken()
    {
        boolean valueBool = (this.throughBeamSensor.getVoltage() < 2.5);
        SmartDashboardLogger.putBoolean(ElevatorComponent.THROUGH_BEAM_SENSOR_BOOLEAN_LOG_KEY, valueBool);
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

        this.throughBeamLightUpper.set(value);
        this.throughBeamLightLeft.set(value);
        this.throughBeamLightRight.set(value);
        SmartDashboardLogger.putBoolean(ElevatorComponent.THROUGH_BEAM_RELAY_LOG_KEY, value);
    }

    public void setLimitSwitchRelayValue(boolean value)
    {
        this.limitSwitchLight.set(value);
        SmartDashboardLogger.putBoolean(ElevatorComponent.LIMIT_SWITCH_RELAY_LOG_KEY, value);
    }
}
