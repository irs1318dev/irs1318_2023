package frc.robot.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.HardwareConstants;
import frc.robot.common.Helpers;

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
    public void calculateAngleStraightUpAndDown()
    {
        ArmAngleSetpoint setpoint = ArmMechanism.calculateIKAngles(0.0, Math.abs(HardwareConstants.ARM_LOWER_ARM_LENGTH - HardwareConstants.ARM_UPPER_ARM_LENGTH));
        assertEquals(90.0, setpoint.lowerAngle, 0.01, "lower angle");
        assertEquals(0.0, setpoint.upperAngle, 0.01, "upper angle");
    }

    @Test
    public void calculateAngleStraightUp()
    {
        ArmAngleSetpoint setpoint = ArmMechanism.calculateIKAngles(0.0, HardwareConstants.ARM_UPPER_ARM_LENGTH + HardwareConstants.ARM_LOWER_ARM_LENGTH);
        assertEquals(90.0, setpoint.lowerAngle, 0.01, "lower angle");
        assertEquals(180.0, setpoint.upperAngle, 0.01, "upper angle");
    }

    @Test
    public void calculateAngleStraightOut()
    {
        ArmAngleSetpoint setpoint = ArmMechanism.calculateIKAngles(HardwareConstants.ARM_UPPER_ARM_LENGTH + HardwareConstants.ARM_LOWER_ARM_LENGTH, 0.0);
        assertEquals(0.0, setpoint.lowerAngle, 0.01, "lower angle");
        assertEquals(180.0, setpoint.upperAngle, 0.01, "upper angle");
    }

    @Test
    public void calculateAngleStraightUpAndOver()
    {
        ArmAngleSetpoint setpoint = ArmMechanism.calculateIKAngles(HardwareConstants.ARM_UPPER_ARM_LENGTH, HardwareConstants.ARM_LOWER_ARM_LENGTH);
        assertEquals(90.0, setpoint.lowerAngle, 0.01, "lower angle");
        assertEquals(90.0, setpoint.upperAngle, 0.01, "upper angle");
    }

    @Test
    public void calculateAngleStraightOverAndDown()
    {
        ArmAngleSetpoint setpoint = ArmMechanism.calculateIKAngles(HardwareConstants.ARM_LOWER_ARM_LENGTH, -HardwareConstants.ARM_UPPER_ARM_LENGTH);
        assertEquals(0.0, setpoint.lowerAngle, 0.01, "lower angle");
        assertEquals(90.0, setpoint.upperAngle, 0.01, "upper angle");
    }

    @Test
    public void calculateAngleHalfUpAndOut()
    {
        ArmAngleSetpoint setpoint = ArmMechanism.calculateIKAngles(HardwareConstants.ARM_LOWER_ARM_LENGTH * Helpers.sind(45.0) + HardwareConstants.ARM_UPPER_ARM_LENGTH, HardwareConstants.ARM_LOWER_ARM_LENGTH * Helpers.cosd(45.0));
        assertEquals(45.0, setpoint.lowerAngle, 0.01, "lower angle");
        assertEquals(135.0, setpoint.upperAngle, 0.01, "upper angle");
    }

    @Test
    public void calculateAngleOutOfRangeOrigin()
    {
        ArmAngleSetpoint setpoint = ArmMechanism.calculateIKAngles(0.0, 0.0);
        assertEquals(null, setpoint, "setpoint");
    }

    @Test
    public void calculateAngleOutOfRangeTooFarOut()
    {
        ArmAngleSetpoint setpoint = ArmMechanism.calculateIKAngles(HardwareConstants.ARM_LOWER_ARM_LENGTH + HardwareConstants.ARM_UPPER_ARM_LENGTH, HardwareConstants.ARM_LOWER_ARM_LENGTH + HardwareConstants.ARM_UPPER_ARM_LENGTH);
        assertEquals(null, setpoint, "setpoint");
    }
}
