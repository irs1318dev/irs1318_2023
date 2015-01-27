package org.usfirst.frc.team1318.robot.Elevator;

public class ElevatorTurningConstants
{
    // Velocity PID (right)
    public static final String ELEVATOR_VELOCITY_PID_KP_KEY = "ELEVATOR_VELOCITY_PID_KP";
    public static final double ELEVATOR_VELOCITY_PID_KP_DEFAULT = 0.0005;

    public static final String ELEVATOR_VELOCITY_PID_KI_KEY = "ELEVATOR_VELOCITY_PID_KI";
    public static final double ELEVATOR_VELOCITY_PID_KI_DEFAULT = 0.0;

    public static final String ELEVATOR_VELOCITY_PID_KD_KEY = "ELEVATOR_VELOCITY_PID_KD";
    public static final double ELEVATOR_VELOCITY_PID_KD_DEFAULT = 0.0;

    public static final String ELEVATOR_VELOCITY_PID_KF_KEY = "ELEVATOR_VELOCITY_PID_KF";
    public static final double ELEVATOR_VELOCITY_PID_KF_DEFAULT = 0.5;

    // Position PID (right)
    public static final String ELEVATOR_POSITION_PID_KP_KEY = "ELEVATOR_POSITION_PID_KP";
    public static final double ELEVATOR_POSITION_PID_KP_DEFAULT = 0.0005;

    public static final String ELEVATOR_POSITION_PID_KI_KEY = "ELEVATOR_POSITION_PID_KI";
    public static final double ELEVATOR_POSITION_PID_KI_DEFAULT = 0.0;

    public static final String ELEVATOR_POSITION_PID_KD_KEY = "ELEVATOR_POSITION_PID_KD";
    public static final double ELEVATOR_POSITION_PID_KD_DEFAULT = 0.5;

    public static final String ELEVATOR_POSITION_PID_KF_KEY = "ELEVATOR_POSITION_PID_KF";
    public static final double ELEVATOR_POSITION_PID_KF_DEFAULT = 0.0;

    // Elevator max speeds from encoder
    public static final double ELEVATOR_ENCODER_MAX_SPEED = 170.0; // max speed we expect to detect from the left encoder

    // Drivetrain deadzone/max power levels
    public static final double ELEVATOR_DEAD_ZONE = 0.1;
    public static final double ELEVATOR_MAX_POWER_LEVEL = 0.8; // max power level (velocity)
    public static final double ELEVATOR_MAX_POWER_POSITIONAL_NON_PID = 0.2; // max power level (positional, non-PID)

    public static final double PULSE_DISTANCE = -1;
}
