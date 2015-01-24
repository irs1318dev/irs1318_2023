package org.usfirst.frc.team1318.robot.Elevator;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;

public class ElevatorComponent
{
    private Talon motor;
    private Encoder encoder;

    public ElevatorComponent()
    {
        motor = new Talon(ElectronicsConstants.ELEVATOR_TALON_CHANNEL);
        encoder = new Encoder(ElectronicsConstants.ELEVATOR_ENCODER_CHANNELA,
            ElectronicsConstants.ELEVATOR_ENCODER_CHANNELB);
    }

    public double getEncoderDistance()
    {
        return encoder.getDistance();
    }

    public void setMotorVelocity(double motorVelocity)
    {
        motor.set(motorVelocity);
    }

}
