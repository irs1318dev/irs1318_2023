package frc.robot.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.Test;

import frc.robot.HardwareConstants;
import frc.robot.TuningConstants;
import frc.robot.common.Helpers;

public class ArmMechanismTests
{
    @Test
    public void calculateAngleStraightUpAndDown()
    {
        DoubleTuple setpoint = ArmMechanism.calculateIKAnglesFromPosition(0.0, Math.abs(HardwareConstants.ARM_LOWER_ARM_LENGTH - HardwareConstants.ARM_UPPER_ARM_LENGTH));
        assertEquals(90.0, setpoint.first, 0.01, "lower angle");
        assertEquals(0.0, setpoint.second, 0.01, "upper angle");
    }
    @Test
    public void verifycalculateAngleStraightUpAndDownFK()
    {
        DoubleTuple setpoint = ArmMechanism.calculateFKPositionsFromAngles(90.0, 0.0);
        // assertEquals(0.0, Math.abs(HardwareConstants.ARM_LOWER_ARM_LENGTH - setpoint.first));
        // assertEquals(Math.abs(HardwareConstants.ARM_LOWER_ARM_LENGTH - HardwareConstants.ARM_UPPER_ARM_LENGTH), setpoint.second);
    }


    @Test
    public void calculateAngleStraightOut()
    {
        DoubleTuple setpoint = ArmMechanism.calculateIKAnglesFromPosition(HardwareConstants.ARM_UPPER_ARM_LENGTH + HardwareConstants.ARM_LOWER_ARM_LENGTH, 0.0);
        assertEquals(0.0, setpoint.first, 0.01, "lower angle");
        assertEquals(180.0, setpoint.second, 0.01, "upper angle");
    }

    @Test
    public void calculateAngleStraightUpAndOver()
    {
        DoubleTuple setpoint = ArmMechanism.calculateIKAnglesFromPosition(HardwareConstants.ARM_UPPER_ARM_LENGTH, HardwareConstants.ARM_LOWER_ARM_LENGTH);
        assertEquals(90.0, setpoint.first, 0.01, "lower angle");
        assertEquals(90.0, setpoint.second, 0.01, "upper angle");
    }

    @Test
    public void calculateAngleStraightOverAndDown()
    {
        DoubleTuple setpoint = ArmMechanism.calculateIKAnglesFromPosition(HardwareConstants.ARM_LOWER_ARM_LENGTH, -HardwareConstants.ARM_UPPER_ARM_LENGTH);
        assertEquals(0.0, setpoint.first, 0.01, "lower angle");
        assertEquals(90.0, setpoint.second, 0.01, "upper angle");
    }

    @Test
    public void calculateAngleHalfUpAndOut()
    {
        DoubleTuple setpoint = ArmMechanism.calculateIKAnglesFromPosition(HardwareConstants.ARM_LOWER_ARM_LENGTH * Helpers.sind(45.0) + HardwareConstants.ARM_UPPER_ARM_LENGTH, HardwareConstants.ARM_LOWER_ARM_LENGTH * Helpers.cosd(45.0));
        assertEquals(45.0, setpoint.first, 0.01, "lower angle");
        assertEquals(135.0, setpoint.second, 0.01, "upper angle");
    }

    @Test
    public void calculateAngleOutOfRangeOrigin()
    {
        DoubleTuple setpoint = ArmMechanism.calculateIKAnglesFromPosition(0.0, 0.0);
        assertEquals(null, setpoint, "setpoint");
    }

    @Test
    public void calculateAngleOutOfRangeTooFarOut()
    {
        DoubleTuple setpoint = ArmMechanism.calculateIKAnglesFromPosition(HardwareConstants.ARM_LOWER_ARM_LENGTH + HardwareConstants.ARM_UPPER_ARM_LENGTH, HardwareConstants.ARM_LOWER_ARM_LENGTH + HardwareConstants.ARM_UPPER_ARM_LENGTH);
        assertEquals(null, setpoint, "setpoint");
    }

    @Test
    public void verifyIKAnglesEqualsFKAngles()
    {
        final double theoreticalFullExtension = HardwareConstants.ARM_LOWER_ARM_LENGTH + HardwareConstants.ARM_UPPER_ARM_LENGTH;
        for (double x = -theoreticalFullExtension; x <= theoreticalFullExtension; x += 0.1)
        {
            for (double z = -theoreticalFullExtension; z <= theoreticalFullExtension; z += 0.1)
            {
                DoubleTuple setpoint = ArmMechanism.calculateIKAnglesFromPosition(HardwareConstants.ARM_LOWER_ARM_LENGTH + HardwareConstants.ARM_UPPER_ARM_LENGTH, HardwareConstants.ARM_LOWER_ARM_LENGTH + HardwareConstants.ARM_UPPER_ARM_LENGTH);
                if (setpoint != null)
                {
                    DoubleTuple result = ArmMechanism.calculateFKPositionsFromAngles(setpoint.first, setpoint.second);
                    assertEquals(x, result.first, 0.01, "xPosition");
                    assertEquals(z, result.second, 0.01, "zPosition");
                }
            }
        }
    }

    @Test
    public void verifyFKLengthsEqualsIKLengths()
    {
        final double maxExtension = (HardwareConstants.ARM_EXTENTION_LENGTH - 0.25) * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH;
        for (double lowerExtension = 0.5 * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH; lowerExtension <= maxExtension; lowerExtension += 10.0)
        {
            for (double upperExtension = 0.5 * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH; upperExtension <= maxExtension; upperExtension += 10.0)
            {
                DoubleTuple angles = ArmMechanism.calculateFKAnglesFromExtensions(lowerExtension, upperExtension);
                if (angles != null)
                {
                    DoubleTuple extensions = ArmMechanism.calculateIKExtensionsFromAngles(angles.first, angles.second);
                    assertNotNull(extensions);
                    assertEquals(lowerExtension, extensions.first, 0.01, "lowerExtension");
                    assertEquals(upperExtension, extensions.second, 0.01, "upperExtension");
                }
            }
        }
    }


    @Test
    public void verifyLinearActuatorLengthsStraightUp()
    {
        DoubleTuple setpoint = ArmMechanism.calculateIKExtensionsFromAngles(90.0, 135.0);
        // assertEquals(8.0 * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH, setpoint.second, "Upper LA Length");
        // assertEquals(8.0 * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH, setpoint.first, "Lower LA Length");
    }

    @Test
    public void FullUnitTestForStraightOut()
    {
        DoubleTuple IK = ArmMechanism.calculateIK(HardwareConstants.ARM_LOWER_ARM_LENGTH, HardwareConstants.ARM_UPPER_ARM_LENGTH);
        DoubleTuple FK = ArmMechanism.calculateFK(IK.first, IK.second);
        // assertEquals(HardwareConstants.ARM_LOWER_ARM_LENGTH, FK.first);
        // assertEquals(HardwareConstants.ARM_UPPER_ARM_LENGTH, FK.second);
        

    }

    @Test
    public void verifyLAFKForFullyRetracted() //StraightOverAnd Down
    {
        DoubleTuple LA_to_Angles_out = ArmMechanism.calculateFKAnglesFromExtensions(16.93, 16.93);
        // assertEquals(40, LA_to_Angles_out.first, "Lower Arm Angle Out");
        // assertEquals(20, LA_to_Angles_out.second, "Upper Arm Angle Out");
    }
}
