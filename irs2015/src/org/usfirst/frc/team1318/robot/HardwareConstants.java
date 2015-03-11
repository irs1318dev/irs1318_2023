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

    public static final double DRIVETRAIN_LEFT_ENCODER_PULSES_PER_REVOLUTION = 360.0;
    public static final double DRIVETRAIN_LEFT_WHEEL_DIAMETER = 6.0 * 2.54; // (in centimeters)
    public static final double DRIVETRAIN_LEFT_PULSE_DISTANCE = Math.PI
        * HardwareConstants.DRIVETRAIN_LEFT_WHEEL_DIAMETER / HardwareConstants.DRIVETRAIN_LEFT_ENCODER_PULSES_PER_REVOLUTION;

    public static final double DRIVETRAIN_RIGHT_ENCODER_PULSES_PER_REVOLUTION = 360.0;
    public static final double DRIVETRAIN_RIGHT_WHEEL_DIAMETER = 6.0 * 2.54; // (in centimeters)
    public static final double DRIVETRAIN_RIGHT_PULSE_DISTANCE = Math.PI
        * HardwareConstants.DRIVETRAIN_RIGHT_WHEEL_DIAMETER / HardwareConstants.DRIVETRAIN_RIGHT_ENCODER_PULSES_PER_REVOLUTION;

    public static final double DRIVETRAIN_WHEEL_SEPARATION_DISTANCE = 23.75 * 2.54; // (in centimeters)

    public static final double DRIVETRAIN_WHEEL_DISTANCE = 9.0 * 2.54; // in centimeters

    //================================================== Elevator ==============================================================

    public static final double ELEVATOR_FLOOR_HEIGHT = 0;
    //    public static final double ELEVATOR_PLATFORM_HEIGHT = 2 * 2.54;
    public static final double ELEVATOR_STEP_HEIGHT = 6.5 * 2.54;

    public static final double ELEVATOR_0_TOTE_HEIGHT = 0;
    public static final double ELEVATOR_1_TOTE_HEIGHT = 16 * 2.54;
    public static final double ELEVATOR_2_TOTE_HEIGHT = 27.5 * 2.54;
    public static final double ELEVATOR_3_TOTE_HEIGHT = 39 * 2.54;

    public static final double ELEVATOR_MIN_HEIGHT = 0.0;
    public static final double ELEVATOR_MAX_HEIGHT = 101.147; // (approximately 40 inches - we rely on the actual height from the encoder here...) 40 * 2.54;
    public static final double ELEVATOR_MAX_POWER_POSITIONAL_NON_PID = 0.2; // max power level (positional, non-PID)

    public static final double ELEVATOR_PULSE_DISTANCE = 4.5 * 2.54 / 360.0;
}
