package frc.robot.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.TuningConstants;

public class ArmMechanismTests
{
    @Test
    public void verifyIKFullyRetracted()
    {
        //ArmPositionSetpoint setpoint = ArmMechanism.calculateIK(0.0, 0.0);
        // assertEquals(TuningConstants.ARM_LOWER_FULL_EXTENTION_LENGTH * TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH, setpoint.lowerPosition, "lower position");
        // assertEquals(TuningConstants.ARM_UPPER_FULL_RETRACTED_LENGTH * TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH, setpoint.upperPosition, "upper position");
    }

    @Test
    public void calculateAngle()
    {
        ArmAngleSetpoint setpoint = ArmMechanism.calculaterIKAngles(46.0, 37.0);
        assertEquals( 90, setpoint.lowerAngle, "lower position");
        assertEquals( 90, setpoint.upperAngle, "upper position");
        
    }
}
