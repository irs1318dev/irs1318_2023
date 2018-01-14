package org.usfirst.frc.team1318.robot;

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
    public static final double DRIVETRAIN_LEFT_WHEEL_DIAMETER = 4.0; // (in inches)
    public static final double DRIVETRAIN_LEFT_PULSE_DISTANCE = Math.PI
        * HardwareConstants.DRIVETRAIN_LEFT_WHEEL_DIAMETER / HardwareConstants.DRIVETRAIN_LEFT_ENCODER_PULSES_PER_REVOLUTION;

    public static final double DRIVETRAIN_RIGHT_ENCODER_PULSES_PER_REVOLUTION = 4096.0;
    public static final double DRIVETRAIN_RIGHT_WHEEL_DIAMETER = 4.0; // (in inches)
    public static final double DRIVETRAIN_RIGHT_PULSE_DISTANCE = Math.PI
        * HardwareConstants.DRIVETRAIN_RIGHT_WHEEL_DIAMETER / HardwareConstants.DRIVETRAIN_RIGHT_ENCODER_PULSES_PER_REVOLUTION;

    // measure from outside of wheel:
    public static final double DRIVETRAIN_WHEEL_SEPARATION_DISTANCE = TuningConstants.COMPETITION_ROBOT ? 25.0 : 24.0; // (in inches)

    // DriveTrain motor/sensor orientations
    public static final boolean DRIVETRAIN_LEFT_INVERT_OUTPUT = false;
    public static final boolean DRIVETRAIN_LEFT_INVERT_SENSOR = true;
    public static final boolean DRIVETRAIN_RIGHT_INVERT_OUTPUT = true;
    public static final boolean DRIVETRAIN_RIGHT_INVERT_SENSOR = true;
}
