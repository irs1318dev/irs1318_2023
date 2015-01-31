package org.usfirst.frc.team1318.robot.Elevator;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;

public class ElevatorComponent
{
    private final DigitalInput throughBeamSensor;
    private final Talon motor;
    private final Encoder encoder;

    public ElevatorComponent()
    {
        this.motor = new Talon(ElectronicsConstants.ELEVATOR_TALON_CHANNEL);
        this.encoder = new Encoder(ElectronicsConstants.ELEVATOR_ENCODER_CHANNELA,
            ElectronicsConstants.ELEVATOR_ENCODER_CHANNELB);
        this.throughBeamSensor = new DigitalInput(ElectronicsConstants.ELEVATOR_DINPUT_CHANNEL);

        this.encoder.setDistancePerPulse(ElevatorTurningConstants.PULSE_DISTANCE);
    }

    /**
     * 
     * @return Value of Encoder Distance
     */
    public double getEncoderDistance()
    {
        return this.encoder.getDistance();
    }

    public double getEncoderVelocity()
    {
        return this.encoder.getRate();
    }

    public void setMotorVelocity(double motorVelocity)
    {
        this.motor.set(motorVelocity);
    }

    /**
     * 
     * @return Through Beam Sensor output
     */
    public boolean getThroughBeamSensor()
    {
        return this.throughBeamSensor.get();
    }
}
