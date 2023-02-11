package frc.robot.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.HardwareConstants;
import frc.robot.common.Helpers;

public class ArmMechanismTests
{
    @Test
    public void calculateAngleStraightUpAndDown()
    {
        DoubleTuple setpoint = ArmMechanism.calculateIKAngles(0.0, Math.abs(HardwareConstants.ARM_LOWER_ARM_LENGTH - HardwareConstants.ARM_UPPER_ARM_LENGTH));
        assertEquals(90.0, setpoint.first, 0.01, "lower angle");
        assertEquals(0.0, setpoint.second, 0.01, "upper angle");
    }

    @Test
    public void calculateAngleStraightUp()
    {
        DoubleTuple setpoint = ArmMechanism.calculateIKAngles(0.0, HardwareConstants.ARM_UPPER_ARM_LENGTH + HardwareConstants.ARM_LOWER_ARM_LENGTH);
        assertEquals(90.0, setpoint.first, 0.01, "lower angle");
        assertEquals(180.0, setpoint.second, 0.01, "upper angle");
    }

    @Test
    public void calculateAngleStraightOut()
    {
        DoubleTuple setpoint = ArmMechanism.calculateIKAngles(HardwareConstants.ARM_UPPER_ARM_LENGTH + HardwareConstants.ARM_LOWER_ARM_LENGTH, 0.0);
        assertEquals(0.0, setpoint.first, 0.01, "lower angle");
        assertEquals(180.0, setpoint.second, 0.01, "upper angle");
    }

    @Test
    public void calculateAngleStraightUpAndOver()
    {
        DoubleTuple setpoint = ArmMechanism.calculateIKAngles(HardwareConstants.ARM_UPPER_ARM_LENGTH, HardwareConstants.ARM_LOWER_ARM_LENGTH);
        assertEquals(90.0, setpoint.first, 0.01, "lower angle");
        assertEquals(90.0, setpoint.second, 0.01, "upper angle");
    }

    @Test
    public void calculateAngleStraightOverAndDown()
    {
        DoubleTuple setpoint = ArmMechanism.calculateIKAngles(HardwareConstants.ARM_LOWER_ARM_LENGTH, -HardwareConstants.ARM_UPPER_ARM_LENGTH);
        assertEquals(0.0, setpoint.first, 0.01, "lower angle");
        assertEquals(90.0, setpoint.second, 0.01, "upper angle");
    }

    @Test
    public void calculateAngleHalfUpAndOut()
    {
        DoubleTuple setpoint = ArmMechanism.calculateIKAngles(HardwareConstants.ARM_LOWER_ARM_LENGTH * Helpers.sind(45.0) + HardwareConstants.ARM_UPPER_ARM_LENGTH, HardwareConstants.ARM_LOWER_ARM_LENGTH * Helpers.cosd(45.0));
        assertEquals(45.0, setpoint.first, 0.01, "lower angle");
        assertEquals(135.0, setpoint.second, 0.01, "upper angle");
    }

    @Test
    public void calculateAngleOutOfRangeOrigin()
    {
        DoubleTuple setpoint = ArmMechanism.calculateIKAngles(0.0, 0.0);
        assertEquals(null, setpoint, "setpoint");
    }

    @Test
    public void calculateAngleOutOfRangeTooFarOut()
    {
        DoubleTuple setpoint = ArmMechanism.calculateIKAngles(HardwareConstants.ARM_LOWER_ARM_LENGTH + HardwareConstants.ARM_UPPER_ARM_LENGTH, HardwareConstants.ARM_LOWER_ARM_LENGTH + HardwareConstants.ARM_UPPER_ARM_LENGTH);
        assertEquals(null, setpoint, "setpoint");
    }

    @Test
    public void verifyIKEqualsFK()
    {
        final double theoreticalFullExtension = HardwareConstants.ARM_LOWER_ARM_LENGTH + HardwareConstants.ARM_UPPER_ARM_LENGTH;
        for (double x = -theoreticalFullExtension; x <= theoreticalFullExtension; x += 0.1)
        {
            for (double z = -theoreticalFullExtension; z <= theoreticalFullExtension; z += 0.1)
            {
                DoubleTuple setpoint = ArmMechanism.calculateIKAngles(HardwareConstants.ARM_LOWER_ARM_LENGTH + HardwareConstants.ARM_UPPER_ARM_LENGTH, HardwareConstants.ARM_LOWER_ARM_LENGTH + HardwareConstants.ARM_UPPER_ARM_LENGTH);
                if (setpoint != null)
                {
                    DoubleTuple result = ArmMechanism.calculateFKPositions(setpoint.first, setpoint.second);
                    assertEquals(x, result.first, 0.01, "xPosition");
                    assertEquals(z, result.second, 0.01, "zPosition");
                }
            }
        }
    }
}
