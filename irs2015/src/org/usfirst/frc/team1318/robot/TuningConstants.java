package org.usfirst.frc.team1318.robot;

/**
 * All constants related to tuning the operation of the robot.
 * 
 * @author Will
 * 
 */
public class TuningConstants
{
    public static final double COLLECTOR_SPEED = 0.8;

    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KP = 0.0005;
    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KI = 0.0;
    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KD = 0.0;
    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KF = 0.5;

    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KP = 0.0005;
    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KI = 0.0;
    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KD = 0.0;
    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KF = 0.5;

    public static final double DRIVETRAIN_POSITION_PID_RIGHT_KP = 0.0005;
    public static final double DRIVETRAIN_POSITION_PID_RIGHT_KI = 0.0;
    public static final double DRIVETRAIN_POSITION_PID_RIGHT_KD = 0.5;
    public static final double DRIVETRAIN_POSITION_PID_RIGHT_KF = 0.0;

    public static final double DRIVETRAIN_POSITION_PID_LEFT_KP = 0.0005;
    public static final double DRIVETRAIN_POSITION_PID_LEFT_KI = 0.0;
    public static final double DRIVETRAIN_POSITION_PID_LEFT_KD = 0.5;
    public static final double DRIVETRAIN_POSITION_PID_LEFT_KF = 0.0;

    public static final double DRIVETRAIN_DEAD_ZONE = 0.1;
    public static final double DRIVETRAIN_MAX_POWER_LEVEL = 0.8; // max power level

    public static final double DRIVETRAIN_LEFT_ENCODER_MAX_SPEED = 1.0; // max speed we expect to detect from the left encoder
    public static final double DRIVETRAIN_RIGHT_ENCODER_MAX_SPEED = 1.0; // max speed we expect to detect from the right encoder

    public static final double DRIVETRAIN_A = 0.2; // "a" coefficient (advancing turn)
    public static final double DRIVETRAIN_B = 1.0; // "b" coefficient (in-place turn)

    public static final double SHOOTER_TOGGLE_DURATION = 2.0;
}
