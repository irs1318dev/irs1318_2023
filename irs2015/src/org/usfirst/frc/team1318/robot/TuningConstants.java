package org.usfirst.frc.team1318.robot;

/**
 * All constants related to tuning the operation of the robot.
 * 
 * @author Will
 * 
 */
public class TuningConstants
{
    //================================================== DriveTrain ==============================================================

    // Drivetrain PID keys/default values:
    public static final boolean DRIVETRAIN_USE_PID_DEFAULT = true;

    // Velocity PID (right)
    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KP_DEFAULT = 0.03; // 3.0;
    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KI_DEFAULT = 0.0;
    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KD_DEFAULT = 0.0;
    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KF_DEFAULT = 0.5; // 0.5;
    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KS_DEFAULT = 100.0;

    // Velocity PID (left)
    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KP_DEFAULT = 0.03; // 3.0;
    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KI_DEFAULT = 0.0;
    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KD_DEFAULT = 0.0;
    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KF_DEFAULT = 0.5; // 0.5;
    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KS_DEFAULT = 100.0;

    // Position PID (right)
    public static final double DRIVETRAIN_POSITION_PID_RIGHT_KP_DEFAULT = 0.2;
    public static final double DRIVETRAIN_POSITION_PID_RIGHT_KI_DEFAULT = 0.0;
    public static final double DRIVETRAIN_POSITION_PID_RIGHT_KD_DEFAULT = 0.0;
    public static final double DRIVETRAIN_POSITION_PID_RIGHT_KF_DEFAULT = 0.0;

    // Position PID (left)
    public static final double DRIVETRAIN_POSITION_PID_LEFT_KP_DEFAULT = 0.2;
    public static final double DRIVETRAIN_POSITION_PID_LEFT_KI_DEFAULT = 0.0;
    public static final double DRIVETRAIN_POSITION_PID_LEFT_KD_DEFAULT = 0.0;
    public static final double DRIVETRAIN_POSITION_PID_LEFT_KF_DEFAULT = 0.0;

    // Drivetrain max speeds from encoder
    public static final double DRIVETRAIN_LEFT_ENCODER_MAX_SPEED = 1.0; // max speed we expect to detect from the left encoder
    public static final double DRIVETRAIN_RIGHT_ENCODER_MAX_SPEED = 1.0; // max speed we expect to detect from the right encoder

    // Drivetrain kinematics choices for advanced one-stick drive
    public static final double DRIVETRAIN_A = 0.4; // "a" coefficient (advancing turn)
    public static final double DRIVETRAIN_B = 0.4; // "b" coefficient (in-place turn)

    // Drivetrain deadzone/max power levels
    public static final double DRIVETRAIN_X_DEAD_ZONE = 0.1;//.04
    public static final double DRIVETRAIN_Y_DEAD_ZONE = 0.1;//.05
    public static final double DRIVETRAIN_MAX_POWER_LEVEL = 0.775; // max power level (velocity)
    public static final double DRIVETRAIN_MAX_POWER_POSITIONAL_NON_PID = 0.2; // max power level (positional, non-PID)

    public static final double DRIVETRAIN_POSITIONAL_MAX_POWER_LEVEL = 0.6;
    public static final double DRIVETRAIN_VELOCITY_MAX_POWER_LEVEL = 1.0;

    public static final double DRIVETRAIN_REVERSE_RIGHT_SCALE_FACTOR = 1.15; //moving forwards
    public static final double DRIVETRAIN_REVERSE_LEFT_SCALE_FACTOR = 1.17; //moving backwards

    //================================================== Elevator ==============================================================

    // Position PID
    public static final double ELEVATOR_POSITION_PID_KP_DEFAULT = 0.2;
    public static final double ELEVATOR_POSITION_PID_KI_DEFAULT = 0.0;
    public static final double ELEVATOR_POSITION_PID_KD_DEFAULT = 0.0;
    public static final double ELEVATOR_POSITION_PID_KF_DEFAULT = 0.0;

    public static final String ELEVATOR_POSITION_PID_KO_KEY = "ELEVATOR_POSITION_PID_KO";
    public static final double ELEVATOR_POSITION_PID_KO_DEFAULT = 0.0;

    public static final String ELEVATOR_POSITION_PID_KN_KEY = "ELEVATOR_POSITION_PID_KN";
    public static final double ELEVATOR_POSITION_PID_KN_DEFAULT = 1.0;

    // Elevator max speeds from encoder
    public static final double ELEVATOR_ENCODER_MAX_SPEED = 170.0; // max speed we expect to detect from the left encoder

    // Elevator deadzone/max power levels
    public static final double ELEVATOR_DEAD_ZONE = 0.1;
    public static final double ELEVATOR_MAX_POWER_LEVEL = 0.75; // max power level (velocity)
    public static final double ELEVATOR_SLOW_MODE_MAX_POWER_LEVEL = 0.675;//0.375; // max power level (velocity) for slow mode
    public static final double ELEVATOR_FAST_MODE_MAX_POWER_LEVEL = 1.0; // max power level (velocity) for fast mode

    public static final double ELEVATOR_OVERRIDE_POWER_LEVEL = 0.5;
    public static final double ELEVATOR_BELOW_MINIMUM_POSITION = -10000;
    public static final double ELEVATOR_MAX_VELOCITY = 32;

    //================================================== Arm ==============================================================

    public static final double ARM_TILT_EXTEND_WAIT_TIME = 1;
    public static final double ARM_EXTENDOR_EXTEND_WAIT_TIME = 1;//2;
    public static final double ARM_TROMBONE_EXTEND_WAIT_TIME = 1;

    public static final double ARM_TILT_RETRACT_WAIT_TIME = 1;
    public static final double ARM_TROMBONE_RETRACT_WAIT_TIME = 1;

    public static final double ARM_SAFETY_WAIT = 1;

    //================================================== Intake ==============================================================

    public static final double INTAKE_MOTOR_SPEED = .7;
}
