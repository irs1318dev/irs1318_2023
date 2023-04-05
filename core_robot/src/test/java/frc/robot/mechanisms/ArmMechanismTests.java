package frc.robot.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.nio.charset.StandardCharsets;

import org.junit.jupiter.api.Test;

import de.siegmar.fastcsv.writer.*;

import frc.lib.helpers.Helpers;
import frc.robot.HardwareConstants;
import frc.robot.TuningConstants;

public class ArmMechanismTests
{
    @Test
    public void calculateAngleStraightUpAndOver()
    {
        DoubleTuple setpoint = ArmMechanism.calculateIKAnglesFromPosition(HardwareConstants.ARM_UPPER_ARM_LENGTH, HardwareConstants.ARM_LOWER_ARM_LENGTH);
        assertEquals(90.0, setpoint.first, 0.01, "lower angle");
        assertEquals(90.0, setpoint.second, 0.01, "upper angle");
    }

    // @Test
    public void checkIKEqualsPositions()
    {
        DoubleTuple setpoint;

        setpoint = ArmMechanism.calculateFK(TuningConstants.ARM_LOWER_POSITION_STOWED, TuningConstants.ARM_UPPER_POSITION_STOWED);
        assertEquals(TuningConstants.ARM_IKX_POSITION_STOWED, setpoint.first, 0.01, "stowed x");
        assertEquals(TuningConstants.ARM_IKZ_POSITION_STOWED, setpoint.second, 0.01, "stowed z");

        setpoint = ArmMechanism.calculateFK(TuningConstants.ARM_LOWER_POSITION_MIDDLE_CONE, TuningConstants.ARM_UPPER_POSITION_MIDDLE_CONE);
        assertEquals(TuningConstants.ARM_IKX_POSITION_MIDDLE_CONE, setpoint.first, 0.01, "middle cone x");
        assertEquals(TuningConstants.ARM_IKZ_POSITION_MIDDLE_CONE, setpoint.second, 0.01, "middle cone z");

        setpoint = ArmMechanism.calculateFK(TuningConstants.ARM_LOWER_POSITION_MIDDLE_CUBE, TuningConstants.ARM_UPPER_POSITION_MIDDLE_CUBE);
        assertEquals(TuningConstants.ARM_IKX_POSITION_MIDDLE_CUBE, setpoint.first, 0.01, "middle cube x");
        assertEquals(TuningConstants.ARM_IKZ_POSITION_MIDDLE_CUBE, setpoint.second, 0.01, "middle cube z");

        setpoint = ArmMechanism.calculateFK(TuningConstants.ARM_LOWER_POSITION_HIGH_CONE_DOWN, TuningConstants.ARM_UPPER_POSITION_HIGH_CONE_DOWN);
        assertEquals(TuningConstants.ARM_IKX_POSITION_HIGH_CONE_DOWN, setpoint.first, 0.01, "high cone down x");
        assertEquals(TuningConstants.ARM_IKZ_POSITION_HIGH_CONE_DOWN, setpoint.second, 0.01, "high cone down z");

        setpoint = ArmMechanism.calculateFK(TuningConstants.ARM_LOWER_POSITION_HIGH_CONE_UP, TuningConstants.ARM_UPPER_POSITION_HIGH_CONE_UP);
        assertEquals(TuningConstants.ARM_IKX_POSITION_HIGH_CONE_UP, setpoint.first, 0.01, "high cone up x");
        assertEquals(TuningConstants.ARM_IKZ_POSITION_HIGH_CONE_UP, setpoint.second, 0.01, "high cone up z");

        setpoint = ArmMechanism.calculateFK(TuningConstants.ARM_LOWER_POSITION_HIGH_CUBE, TuningConstants.ARM_UPPER_POSITION_HIGH_CUBE);
        assertEquals(TuningConstants.ARM_IKX_POSITION_HIGH_CUBE, setpoint.first, 0.01, "high cube x");
        assertEquals(TuningConstants.ARM_IKZ_POSITION_HIGH_CUBE, setpoint.second, 0.01, "high cube z");

        setpoint = ArmMechanism.calculateFK(TuningConstants.ARM_LOWER_POSITION_CONE_GROUND_PICKUP, TuningConstants.ARM_UPPER_POSITION_CONE_GROUND_PICKUP);
        assertEquals(TuningConstants.ARM_IKX_POSITION_CONE_GROUND_PICKUP, setpoint.first, 0.01, "cone ground x");
        assertEquals(TuningConstants.ARM_IKZ_POSITION_CONE_GROUND_PICKUP, setpoint.second, 0.01, "cone ground z");

        setpoint = ArmMechanism.calculateFK(TuningConstants.ARM_LOWER_POSITION_CUBE_GROUND_PICKUP, TuningConstants.ARM_UPPER_POSITION_CUBE_GROUND_PICKUP);
        assertEquals(TuningConstants.ARM_IKX_POSITION_CUBE_GROUND_PICKUP, setpoint.first, 0.01, "cube ground x");
        assertEquals(TuningConstants.ARM_IKZ_POSITION_CUBE_GROUND_PICKUP, setpoint.second, 0.01, "cube ground z");

        setpoint = ArmMechanism.calculateFK(TuningConstants.ARM_LOWER_POSITION_CONE_SUBSTATION_PICKUP_APPROACH, TuningConstants.ARM_UPPER_POSITION_CONE_SUBSTATION_PICKUP_APPROACH);
        assertEquals(TuningConstants.ARM_IKX_POSITION_CONE_SUBSTATION_PICKUP_APPROACH, setpoint.first, 0.01, "cone substation approach x");
        assertEquals(TuningConstants.ARM_IKZ_POSITION_CONE_SUBSTATION_PICKUP_APPROACH, setpoint.second, 0.01, "cone substation approach z");

        setpoint = ArmMechanism.calculateFK(TuningConstants.ARM_LOWER_POSITION_CONE_SUBSTATION_PICKUP, TuningConstants.ARM_UPPER_POSITION_CONE_SUBSTATION_PICKUP);
        assertEquals(TuningConstants.ARM_IKX_POSITION_CONE_SUBSTATION_PICKUP, setpoint.first, 0.01, "cone substation x");
        assertEquals(TuningConstants.ARM_IKZ_POSITION_CONE_SUBSTATION_PICKUP, setpoint.second, 0.01, "cone substation z");

        setpoint = ArmMechanism.calculateFK(TuningConstants.ARM_LOWER_POSITION_CUBE_SINGLE_SUBSTATION_PICKUP, TuningConstants.ARM_UPPER_POSITION_CUBE_SINGLE_SUBSTATION_PICKUP);
        assertEquals(TuningConstants.ARM_IKX_POSITION_CUBE_SINGLE_SUBSTATION_PICKUP, setpoint.first, 0.01, "cube single substation x");
        assertEquals(TuningConstants.ARM_IKZ_POSITION_CUBE_SINGLE_SUBSTATION_PICKUP, setpoint.second, 0.01, "cube single substation z");

        setpoint = ArmMechanism.calculateFK(TuningConstants.ARM_LOWER_POSITION_APPROACH, TuningConstants.ARM_UPPER_POSITION_APPROACH);
        assertEquals(TuningConstants.ARM_IKX_POSITION_APPROACH, setpoint.first, 0.01, "approach x");
        assertEquals(TuningConstants.ARM_IKZ_POSITION_APPROACH, setpoint.second, 0.01, "approach z");

        setpoint = ArmMechanism.calculateFK(TuningConstants.ARM_LOWER_POSITION_CONE_UPRIGHTING_MACRO, TuningConstants.ARM_UPPER_POSITION_CONE_UPRIGHTING_MACRO);
        assertEquals(TuningConstants.ARM_IKX_POSITION_CONE_UPRIGHTING_MACRO, setpoint.first, 0.01, "uprighting x");
        assertEquals(TuningConstants.ARM_IKZ_POSITION_CONE_UPRIGHTING_MACRO, setpoint.second, 0.01, "uprighting z");
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
                DoubleTuple setpoint = ArmMechanism.calculateIKAnglesFromPosition(x, z);
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
        final double maxExtension = (HardwareConstants.ARM_EXTENTION_LENGTH) * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH;
        for (double lowerExtension = 0.0 * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH; lowerExtension <= maxExtension; lowerExtension += 10.0)
        {
            for (double upperExtension = 0.0 * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH; upperExtension <= maxExtension; upperExtension += 10.0)
            {
                DoubleTuple angles = ArmMechanism.calculateFKAnglesFromExtensions(lowerExtension, upperExtension);
                if (angles != null)
                {
                    DoubleTuple extensions = ArmMechanism.calculateIKExtensionsFromAngles(angles.first, angles.second);
                    if (extensions != null)
                    {
                        assertEquals(lowerExtension, extensions.first, 0.01, "lowerExtension");
                        assertEquals(upperExtension, extensions.second, 0.01, "upperExtension");
                    }
                }
            }
        }
    }

    public static void main(String[] args)
    {
        // Generate CSV file containing valid positions for the arm given the IK/FK (and that they agree)
        try (CsvWriter validCsvWriter = CsvWriter.builder().build(java.nio.file.Path.of("validPositions-rangeConstrained.csv"), StandardCharsets.UTF_8))
        {
            validCsvWriter.writeRow("x", "z", "lowerExtension", "upperExtension");

            final double theoreticalFullExtension = HardwareConstants.ARM_LOWER_ARM_LENGTH + HardwareConstants.ARM_UPPER_ARM_LENGTH;
            for (double x = -theoreticalFullExtension; x <= theoreticalFullExtension; x += 0.1)
            {
                for (double z = -theoreticalFullExtension; z <= theoreticalFullExtension; z += 0.1)
                {
                    DoubleTuple setpoint = ArmMechanism.calculateIK(x, z);
                    if (setpoint != null)
                    {
                        DoubleTuple result = ArmMechanism.calculateFK(setpoint.first, setpoint.second);
                        if (result != null &&
                            Helpers.RoughEquals(x, result.first, 0.01) &&
                            Helpers.RoughEquals(z, result.second, 0.01))
                        {
                            validCsvWriter.writeRow(String.valueOf(x), String.valueOf(z), String.valueOf(setpoint.first), String.valueOf(setpoint.second));
                        }
                        else
                        {
                            throw new RuntimeException("Unsupported position " + x + ", " + z);
                        }
                    }
                }
            }
        }
        catch (IOException e)
        {
            e.printStackTrace(System.err);
        }

        // try (CsvWriter csvWriter = CsvWriter.builder().build(java.nio.file.Path.of("theoreticalPositions.csv"), StandardCharsets.UTF_8))
        // {
        //     csvWriter.writeRow("x", "z", "lowerExtension", "upperExtension");

        //     final double maxExtension = (HardwareConstants.ARM_EXTENTION_LENGTH) * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH;
        //     for (double lowerExtension = 0.0 * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH; lowerExtension <= maxExtension; lowerExtension += 10.0)
        //     {
        //         for (double upperExtension = 0.0 * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH; upperExtension <= maxExtension; upperExtension += 10.0)
        //         {
        //             DoubleTuple position = ArmMechanism.calculateFK(lowerExtension, upperExtension);
        //             if (position != null)
        //             {
        //                 csvWriter.writeRow(String.valueOf(position.first), String.valueOf(position.second), String.valueOf(lowerExtension), String.valueOf(upperExtension));
        //             }
        //         }
        //     }
        // }
        // catch (IOException e)
        // {
        //     e.printStackTrace(System.err);
        // }
    }
}
