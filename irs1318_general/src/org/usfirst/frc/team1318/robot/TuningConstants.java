package org.usfirst.frc.team1318.robot;

/**
 * All constants related to tuning the operation of the robot.
 * 
 * @author Will
 * 
 */
public class TuningConstants
{
    public static final boolean THROW_EXCEPTIONS = true;

    //================================================== DriveTrain ==============================================================

    // Drivetrain PID keys/default values:
    public static final boolean DRIVETRAIN_USE_PID_DEFAULT = true;

    // Velocity PID (right)
    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KP_DEFAULT = 0.0275;
    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KI_DEFAULT = 0.0;
    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KD_DEFAULT = 0.0;
    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KF_DEFAULT = 0.4;
    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KS_DEFAULT = 100.0;

    // Velocity PID (left)
    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KP_DEFAULT = 0.0275;
    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KI_DEFAULT = 0.0;
    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KD_DEFAULT = 0.0;
    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KF_DEFAULT = 0.4;
    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KS_DEFAULT = 100.0;

    // Position PID (right)
    public static final double DRIVETRAIN_POSITION_PID_RIGHT_KP_DEFAULT = 0.15;
    public static final double DRIVETRAIN_POSITION_PID_RIGHT_KI_DEFAULT = 0.0;
    public static final double DRIVETRAIN_POSITION_PID_RIGHT_KD_DEFAULT = 0.0;
    public static final double DRIVETRAIN_POSITION_PID_RIGHT_KF_DEFAULT = 0.0;

    // Position PID (left)
    public static final double DRIVETRAIN_POSITION_PID_LEFT_KP_DEFAULT = 0.15;
    public static final double DRIVETRAIN_POSITION_PID_LEFT_KI_DEFAULT = 0.0;
    public static final double DRIVETRAIN_POSITION_PID_LEFT_KD_DEFAULT = 0.0;
    public static final double DRIVETRAIN_POSITION_PID_LEFT_KF_DEFAULT = 0.0;

    // Drivetrain choices for one-stick drive
    public static final double DRIVETRAIN_K1 = 1.4;
    public static final double DRIVETRAIN_K2 = 0.5;

    public static final double DRIVETRAIN_A = 0.4;// "a" coefficient (advancing turn)
    public static final double DRIVETRAIN_B = 0.4;// "b" coefficient (in-place turn)

    // Drivetrain deadzone/max power levels
    public static final double DRIVETRAIN_X_DEAD_ZONE = .05;
    public static final double DRIVETRAIN_Y_DEAD_ZONE = .1;
    public static final double DRIVETRAIN_MAX_POWER_LEVEL = 0.775;// max power level (velocity)
    public static final double DRIVETRAIN_MAX_POWER_POSITIONAL_NON_PID = 0.2;// max power level (positional, non-PID)

    public static final double DRIVETRAIN_POSITIONAL_MAX_POWER_LEVEL = 0.6;
    public static final double DRIVETRAIN_VELOCITY_MAX_POWER_LEVEL = 1.0;

    public static final double DRIVETRAIN_REVERSE_RIGHT_SCALE_FACTOR = 1.15;//moving forwards
    public static final double DRIVETRAIN_REVERSE_LEFT_SCALE_FACTOR = 1.17;//moving backwards

    public static final double DRIVETRAIN_ENCODER_ODOMETRY_ANGLE_CORRECTION = 0.979858464888405; // account for turning weirdness (7 degree offset in the angle)

    //================================================== Autonomous ==============================================================

    public static final double MACRO_CANCEL_DEAD_ZONE = 0.1;

    public static final double DRIVETRAIN_POSITIONAL_ACCEPTABLE_DELTA = 1.0;

    // Drivetrain autonomous velocity values
    public static final double DRIVETRAIN_AUTONOMOUS_SLOW_VELOCITY = .3;
    public static final double DRIVETRAIN_AUTONOMOUS_FAST_VELOCITY = .45;

    public static final double AUTONOMOUS_TIME_SLOW = 4.0;
    public static final double AUTONOMOUS_TIME_FAST = 3.0;

    // Acceptable vision centering range values in degrees
    public static final double MAX_VISION_CENTERING_OUTPUT = 0.08;

    // Acceptable vision centering range values in degrees
    public static final double MAX_VISION_CENTERING_RANGE_DEGREES = 1.0;
}
