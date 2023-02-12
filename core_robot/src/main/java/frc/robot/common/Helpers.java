package frc.robot.common;

public class Helpers
{
    // Conversion constants...
    public static final double DEGREES_TO_RADIANS = (Math.PI / 180.0f);
    public static final double RADIANS_TO_DEGREES = (180.0f / Math.PI);
    public static final double INCHES_PER_METER = 39.37;
    public static final double METERS_PER_INCH = 0.0254;
    public static final double GRAVITY_INCH_PER_SQ_SECOND = 386.22047244094; // in/s^2

    public static double EnforceRange(double value, double minValue, double maxValue)
    {
        if (value > maxValue)
        {
            return maxValue;
        }
        else if (value < minValue)
        {
            return minValue;
        }

        return value;
    }

    public static boolean WithinRange(double value, double minValue, double maxValue)
    {
        return value >= minValue && value <= maxValue;
    }

    public static boolean WithinDelta(double actualValue, double expectedValue, double acceptableDelta)
    {
        return Helpers.WithinRange(actualValue, expectedValue - acceptableDelta, expectedValue + acceptableDelta);
    }

    /**
     * Returns the trigonometric tangent of an angle.
     * @param angle in degrees.
     * @return the tangent of the argument (opposite length over adjacent length).
     */
    public static double tand(double angle)
    {
        return Math.tan(angle * Helpers.DEGREES_TO_RADIANS);
    }

    /**
     * Returns the trigonometric arctangent.
     * @param ratio of the opposite side over the adjacent side
     * @return the arctangent angle in degrees between -90 and 90
     */
    public static double atand(double ratio)
    {
        return Math.atan(ratio) * Helpers.RADIANS_TO_DEGREES;
    }

    /**
     * Returns the trigonometric arctangent.
     * @param opposite side
     * @param adjacent side
     * @return the arctangent angle in degrees between -180 and 180
     */
    public static double atan2d(double opposite, double adjacent)
    {
        return Math.atan2(opposite, adjacent) * Helpers.RADIANS_TO_DEGREES;
    }

    /**
     * Returns the trigonometric sine of an angle.
     * @param angle in degrees.
     * @return the sine of the argument (opposite length over hypotenuse length).
     */
    public static double sind(double angle)
    {
        return Math.sin(angle * Helpers.DEGREES_TO_RADIANS);
    }

    /**
     * Returns the trigonometric arcsine.
     * @param ratio of the opposite side over the hypotenuse side
     * @return the arcsine angle in degrees between -90 and 90
     */
    public static double asind(double ratio)
    {
        return Math.asin(ratio) * Helpers.RADIANS_TO_DEGREES;
    }

    /**
     * Returns the trigonometric cosine of an angle.
     * @param angle in degrees.
     * @return the cosine of the argument (adjacent length over hypotenuse length).
     */
    public static double cosd(double angle)
    {
        return Math.cos(angle * Helpers.DEGREES_TO_RADIANS);
    }

    /**
     * Returns the trigonometric arccosine.
     * @param ratio of the adjacent side over the hypotenuse side
     * @return the arccosine angle in degrees between -90 and 90
     */
    public static double acosd(double ratio)
    {
        return Math.acos(ratio) * Helpers.RADIANS_TO_DEGREES;
    }

    /**
     * Returns angle between -180 and 180
     * @param angle in some large range
     * @return angle capped between -180 and 180
     */
    public static double updateAngleRange(double angle)
    {
        // get the difference in degrees between -360 and 360
        double twoLoopAngle = angle % 360.0;

        // change the range from -180 to 180
        if (twoLoopAngle < -180.0)
        {
            return twoLoopAngle + 360.0;
        }
        else if (twoLoopAngle > 180.0)
        {
            return twoLoopAngle - 360.0;
        }

        return twoLoopAngle;
    }

    public static boolean AnglePairWithinDelta(double value1, boolean isSwapped1, double value2, boolean isSwapped2, double acceptableDelta)
    {
        if (isSwapped1)
        {
            value1 = Helpers.updateAngleRange(value1 + 180.0);
        }
        
        if (isSwapped2)
        {
            value2 = Helpers.updateAngleRange(value2 + 180.0);
        }

        return Helpers.WithinDelta(value1, value2, acceptableDelta);
    }

    /**
     * Convert from cartesian coordinates (x, y) to polar angle (along positive x-axis is 0, increasing counter-clockwise)
     * @param x coordinate
     * @param y coordinate
     * @return polar angle (theta), or -1 if at (0, 0)
     */
    public static double convertToPolarAngle(double x, double y)
    {
        // special case for (0, 0)
        if (x == 0.0 && y == 0.0)
        {
            return -1.0;
        }

        // handle horizontal lines
        if (x == 0.0)
        {
            if (y > 0.0)
            {
                return 90.0;
            }
            else
            {
                return 270.0;
            }
        }

        // handle vertical lines
        if (y == 0.0)
        {
            if (x > 0.0)
            {
                return 0.0;
            }
            else
            {
                return 180.0;
            }
        }

        if (x > 0.0 && y > 0.0)
        {
            // quadrant I
            return Helpers.atand(y / x);
        }
        else if (x < 0.0 && y > 0.0)
        {
            // quadrant II
            return Helpers.atand(y / x) + 180.0;
        }
        else if (x < 0.0 && y < 0.0)
        {
            // quadrant III
            return Helpers.atand(y / x) + 180.0;
        }
        else // if (x > 0.0 && y < 0.0)
        {
            // quadrant IV
            return Helpers.atand(y / x) + 360.0;
        }
    }

    /**
     * Calculate the distance of the link in a triange opposite the provided angle, with the two adjacent lengths known
     * @param adjacent1 first length adjacent to the provided angle
     * @param adjacent2 second length adjacent to the provided angle
     * @param angle in degrees
     * @return length of the link opposite the provided angle
     */
    public static double calculateLawOfCosinesDistance(double adjacent1, double adjacent2, double angle)
    {
        double opposite;
        opposite = Math.sqrt(adjacent1 * adjacent1 + adjacent2 * adjacent2 - 2.0 * adjacent1 * adjacent2 * Helpers.cosd(angle));
        return opposite;
    }

    /**
     * Calculate the angle between a pair of adjacent lines with the length in the triangle known 
     * @param adjacent1 first length adjacent to the desired angle
     * @param adjacent2 second length adjacent to the desired angle
     * @param opposite length of the line opposite to the desired angle
     * @return angle of the value in degrees
     */
    public static double calculateLawOfCosinesAngle(double adjacent1, double adjacent2, double opposite)
    {
        double angle;
        angle = Helpers.acosd((adjacent1 * adjacent1 + adjacent2 * adjacent2 - opposite * opposite) / (2.0 * adjacent1 * adjacent2));
        return angle;
    }
}
