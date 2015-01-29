package org.usfirst.frc.team1318.robot.Elevator;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;

public class ElevatorComponent
{
    private DigitalInput throughBeamSensor;
    private Talon motor;
    private Encoder encoder;

    public ElevatorComponent()
    {
        motor = new Talon(ElectronicsConstants.ELEVATOR_TALON_CHANNEL);
        encoder = new Encoder(ElectronicsConstants.ELEVATOR_ENCODER_CHANNELA,
            ElectronicsConstants.ELEVATOR_ENCODER_CHANNELB);
        throughBeamSensor = new DigitalInput(ElectronicsConstants.ELEVATOR_DINPUT_CHANNEL);

        encoder.setDistancePerPulse(ElevatorTurningConstants.PULSE_DISTANCE);
    }

    /**
     * 
     * @return Value of Encoder Distance
     */
    public double getEncoderDistance()
    {
        return encoder.getDistance();
    }

    public double getEncoderVelocity()
    {
        return encoder.getRate();
    }

    public void setMotorVelocity(double motorVelocity)
    {
        motor.set(motorVelocity);

    }

    /**
     * 
     * @return Through Beam Sensor output
     */
    public boolean getThroughBeamSensor()
    {
        return throughBeamSensor.get();

    }

}
