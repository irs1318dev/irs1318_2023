package frc.robot;

/**
 * All constants describing the physical structure of the robot (distances and sizes of things).
 * 
 * @author Will
 * 
 */
public class HardwareConstants
{
    //================================================== DriveTrain ==============================================================
    // Note: Pulse Distance is the distance moved per tick

    public static final double DRIVETRAIN_LEFT_ENCODER_PULSES_PER_REVOLUTION = 4096.0;
    public static final double DRIVETRAIN_LEFT_WHEEL_DIAMETER = 5.0; // (in inches)
    public static final double DRIVETRAIN_LEFT_WHEEL_CIRCUMFERENCE = Math.PI * HardwareConstants.DRIVETRAIN_LEFT_WHEEL_DIAMETER;
    public static final double DRIVETRAIN_LEFT_PULSE_DISTANCE = HardwareConstants.DRIVETRAIN_LEFT_WHEEL_CIRCUMFERENCE / HardwareConstants.DRIVETRAIN_LEFT_ENCODER_PULSES_PER_REVOLUTION;
    public static final double DRIVETRAIN_LEFT_TICKS_PER_INCH = HardwareConstants.DRIVETRAIN_LEFT_ENCODER_PULSES_PER_REVOLUTION / HardwareConstants.DRIVETRAIN_LEFT_WHEEL_CIRCUMFERENCE;

    public static final double DRIVETRAIN_RIGHT_ENCODER_PULSES_PER_REVOLUTION = 4096.0;
    public static final double DRIVETRAIN_RIGHT_WHEEL_DIAMETER = 5.0; // (in inches)
    public static final double DRIVETRAIN_RIGHT_WHEEL_CIRCUMFERENCE = Math.PI * HardwareConstants.DRIVETRAIN_RIGHT_WHEEL_DIAMETER;
    public static final double DRIVETRAIN_RIGHT_PULSE_DISTANCE = HardwareConstants.DRIVETRAIN_RIGHT_WHEEL_CIRCUMFERENCE / HardwareConstants.DRIVETRAIN_RIGHT_ENCODER_PULSES_PER_REVOLUTION;
    public static final double DRIVETRAIN_RIGHT_TICKS_PER_INCH = HardwareConstants.DRIVETRAIN_RIGHT_ENCODER_PULSES_PER_REVOLUTION / HardwareConstants.DRIVETRAIN_RIGHT_WHEEL_CIRCUMFERENCE;

    // measure from outside of wheel:
    public static final double DRIVETRAIN_WHEEL_SEPARATION_DISTANCE = 24.5; // (in inches)

    // DriveTrain motor/sensor orientations
    public static final boolean DRIVETRAIN_LEFT_MASTER_INVERT_OUTPUT = true;
    public static final boolean DRIVETRAIN_LEFT_FOLLOWER1_INVERT_OUTPUT = false;
    public static final boolean DRIVETRAIN_LEFT_FOLLOWER2_INVERT_OUTPUT = false;
    public static final boolean DRIVETRAIN_LEFT_INVERT_SENSOR = false;
    public static final boolean DRIVETRAIN_RIGHT_MASTER_INVERT_OUTPUT = true;
    public static final boolean DRIVETRAIN_RIGHT_FOLLOWER1_INVERT_OUTPUT = true;
    public static final boolean DRIVETRAIN_RIGHT_FOLLOWER2_INVERT_OUTPUT = true;
    public static final boolean DRIVETRAIN_RIGHT_INVERT_SENSOR = true;
}
