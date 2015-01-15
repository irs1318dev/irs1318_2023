package org.usfirst.frc.team1318.robot;

/**
 * All constants related to tuning the operation of the robot.
 * 
 * @author Will
 * 
 */
public class TuningConstants
{
    // Drivetrain PID keys/default values:
    public static final String DRIVETRAIN_USE_PID_KEY = "DRIVETRAIN_USE_PID";
    public static final boolean DRIVETRAIN_USE_PID_DEFAULT = true;

    // Velocity PID (right)
    public static final String DRIVETRAIN_VELOCITY_PID_RIGHT_KP_KEY = "DRIVETRAIN_VELOCITY_PID_RIGHT_KP";
    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KP_DEFAULT = 0.0005;

    public static final String DRIVETRAIN_VELOCITY_PID_RIGHT_KI_KEY = "DRIVETRAIN_VELOCITY_PID_RIGHT_KI";
    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KI_DEFAULT = 0.0;

    public static final String DRIVETRAIN_VELOCITY_PID_RIGHT_KD_KEY = "DRIVETRAIN_VELOCITY_PID_RIGHT_KD";
    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KD_DEFAULT = 0.0;

    public static final String DRIVETRAIN_VELOCITY_PID_RIGHT_KF_KEY = "DRIVETRAIN_VELOCITY_PID_RIGHT_KF";
    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KF_DEFAULT = 0.5;

    // Velocity PID (left)
    public static final String DRIVETRAIN_VELOCITY_PID_LEFT_KP_KEY = "DRIVETRAIN_VELOCITY_PID_LEFT_KP";
    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KP_DEFAULT = 0.0005;

    public static final String DRIVETRAIN_VELOCITY_PID_LEFT_KI_KEY = "DRIVETRAIN_VELOCITY_PID_LEFT_KI";
    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KI_DEFAULT = 0.0;

    public static final String DRIVETRAIN_VELOCITY_PID_LEFT_KD_KEY = "DRIVETRAIN_VELOCITY_PID_LEFT_KD";
    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KD_DEFAULT = 0.0;

    public static final String DRIVETRAIN_VELOCITY_PID_LEFT_KF_KEY = "DRIVETRAIN_VELOCITY_PID_LEFT_KF";
    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KF_DEFAULT = 0.5;

    // Position PID (right)
    public static final String DRIVETRAIN_POSITION_PID_RIGHT_KP_KEY = "DRIVETRAIN_POSITION_PID_RIGHT_KP";
    public static final double DRIVETRAIN_POSITION_PID_RIGHT_KP_DEFAULT = 0.0005;

    public static final String DRIVETRAIN_POSITION_PID_RIGHT_KI_KEY = "DRIVETRAIN_POSITION_PID_RIGHT_KI";
    public static final double DRIVETRAIN_POSITION_PID_RIGHT_KI_DEFAULT = 0.0;

    public static final String DRIVETRAIN_POSITION_PID_RIGHT_KD_KEY = "DRIVETRAIN_POSITION_PID_RIGHT_KD";
    public static final double DRIVETRAIN_POSITION_PID_RIGHT_KD_DEFAULT = 0.5;

    public static final String DRIVETRAIN_POSITION_PID_RIGHT_KF_KEY = "DRIVETRAIN_POSITION_PID_RIGHT_KF";
    public static final double DRIVETRAIN_POSITION_PID_RIGHT_KF_DEFAULT = 0.0;

    // Position PID (left)
    public static final String DRIVETRAIN_POSITION_PID_LEFT_KP_KEY = "DRIVETRAIN_POSITION_PID_LEFT_KP";
    public static final double DRIVETRAIN_POSITION_PID_LEFT_KP_DEFAULT = 0.0005;

    public static final String DRIVETRAIN_POSITION_PID_LEFT_KI_KEY = "DRIVETRAIN_POSITION_PID_LEFT_KI";
    public static final double DRIVETRAIN_POSITION_PID_LEFT_KI_DEFAULT = 0.0;

    public static final String DRIVETRAIN_POSITION_PID_LEFT_KD_KEY = "DRIVETRAIN_POSITION_PID_LEFT_KD";
    public static final double DRIVETRAIN_POSITION_PID_LEFT_KD_DEFAULT = 0.5;

    public static final String DRIVETRAIN_POSITION_PID_LEFT_KF_KEY = "DRIVETRAIN_POSITION_PID_LEFT_KF";
    public static final double DRIVETRAIN_POSITION_PID_LEFT_KF_DEFAULT = 0.0;

    // Drivetrain max speeds from encoder
    public static final double DRIVETRAIN_LEFT_ENCODER_MAX_SPEED = 170.0; // max speed we expect to detect from the left encoder
    public static final double DRIVETRAIN_RIGHT_ENCODER_MAX_SPEED = 170.0; // max speed we expect to detect from the right encoder

    // Drivetrain kinematics choices for advanced one-stick drive
    public static final double DRIVETRAIN_A = 0.4; // "a" coefficient (advancing turn)
    public static final double DRIVETRAIN_B = 1.0; // "b" coefficient (in-place turn)

    // Drivetrain deadzone/max power levels
    public static final double DRIVETRAIN_DEAD_ZONE = 0.1;
    public static final double DRIVETRAIN_MAX_POWER_LEVEL = 0.8; // max power level (velocity)
    public static final double DRIVETRAIN_MAX_POWER_POSITIONAL_NON_PID = 0.2; // max power level (positional, non-PID)
}
