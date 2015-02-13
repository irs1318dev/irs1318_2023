package org.usfirst.frc.team1318.robot;

/**
 * All constants describing how the electronics are plugged together.
 * 
 * @author Will
 * 
 */
public class ElectronicsConstants
{
    public static final int DRIVETRAIN_LEFT_TALON_CHANNEL = 3; // kitbot - 9
    public static final int DRIVETRAIN_RIGHT_TALON_CHANNEL = 2; // kitbot - 8

    public static final int DRIVETRAIN_RIGHT_ENCODER_CHANNEL_A = 4; // kitbot - 6
    public static final int DRIVETRAIN_RIGHT_ENCODER_CHANNEL_B = 5; // kitbot - 7

    public static final int DRIVETRAIN_LEFT_ENCODER_CHANNEL_A = 2; // kitbot - 8
    public static final int DRIVETRAIN_LEFT_ENCODER_CHANNEL_B = 3; // kitbot - 9

    public static final int ELEVATOR_TALON_CHANNEL = 4;
    public static final int ELEVATOR_ENCODER_CHANNEL_A = 1;
    public static final int ELEVATOR_ENCODER_CHANNEL_B = 0;

    public static final int ELEVATOR_LOWER_LIMIT_SWITCH = 6;
    public static final int ELEVATOR_UPPER_LIMIT_SWITCH = 7;

    public static final int ELEVATOR_THROUGH_BEAM_SENSOR_CHANNEL = -1;
    public static final int ELEVATOR_HALL_EFFECT_TOP_CHANNEL = -1;
    public static final int ELEVATOR_HALL_EFFECT_BOTTOM_CHANNEL = -1;

    public static final int INTAKE_LEFT_TALON_CHANNEL = 1;
    public static final int INTAKE_RIGHT_TALON_CHANNEL = 0;

    // All in PCM "A":
    public static final int PCM_A_MODULE = 0;
    public static final int ARM_TROMBONE_SOLANOID_EXTEND = 7;
    public static final int ARM_TROMBONE_SOLANOID_RETRACT = 0;

    public static final int ARM_TILT_LINK_SOLANOID_EXTEND = 6;
    public static final int ARM_TILT_LINK_SOLANOID_RETRACT = 1;

    public static final int ARM_EXTEND_LINK_SOLANOID_EXTEND = 5;
    public static final int ARM_EXTEND_LINK_SOLANOID_RETRACT = 2;

    public static final double MAX_POWER_LEVEL = 1.0;
}
