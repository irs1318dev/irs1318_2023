package org.usfirst.frc.team1318.robot;

/**
 * All constants describing how the electronics are plugged together.
 * 
 * @author Will
 * 
 */
public class ElectronicsConstants
{
    public static final double MAX_POWER_LEVEL = 1.0;

    public static final int PCM_A_MODULE = 1;
    public static final int PCM_B_MODULE = 0;

    //================================================== DriveTrain ==============================================================

    public static final int DRIVETRAIN_LEFT_TALON_CHANNEL = 3; // kitbot - 9
    public static final int DRIVETRAIN_RIGHT_TALON_CHANNEL = 2; // kitbot - 8

    public static final int DRIVETRAIN_RIGHT_ENCODER_CHANNEL_A = 4; // kitbot - 6
    public static final int DRIVETRAIN_RIGHT_ENCODER_CHANNEL_B = 5; // kitbot - 7

    public static final int DRIVETRAIN_LEFT_ENCODER_CHANNEL_A = 2; // kitbot - 8
    public static final int DRIVETRAIN_LEFT_ENCODER_CHANNEL_B = 3; // kitbot - 9

    public static final int TEST_PROXIMITY_SENSOR_BACK_PORT = 1;
    public static final int TEST_PROXIMITY_SENSOR_FRONT_PORT = 2;

    //================================================== Elevator ==============================================================

    public static final int ELEVATOR_TALON_CHANNEL = 4;
    public static final int ELEVATOR_ENCODER_CHANNEL_A = 1;
    public static final int ELEVATOR_ENCODER_CHANNEL_B = 0;

    public static final int ELEVATOR_BOTTOM_LIMIT_SWITCH_CHANNEL = 6;
    public static final int ELEVATOR_TOP_LIMIT_SWITCH_CHANNEL = 7;

    public static final int ELEVATOR_THROUGH_BEAM_SENSOR_CHANNEL = 0;

    //================================================== Intake ==============================================================

    public static final int INTAKE_LEFT_TALON_CHANNEL = 1;
    public static final int INTAKE_RIGHT_TALON_CHANNEL = 0;

    public static final int INTAKE_LEFT_ARM_CHANNEL_A = 6;
    public static final int INTAKE_LEFT_ARM_CHANNEL_B = 1;

    public static final int INTAKE_RIGHT_ARM_CHANNEL_A = 7;
    public static final int INTAKE_RIGHT_ARM_CHANNEL_B = 0;

    //================================================== Arm ==============================================================

    public static final int ARM_TROMBONE_SOLANOID_EXTEND = 7;
    public static final int ARM_TROMBONE_SOLANOID_RETRACT = 0;

    public static final int ARM_TILT_LINK_SOLANOID_EXTEND = 6;
    public static final int ARM_TILT_LINK_SOLANOID_RETRACT = 1;

    public static final int ARM_EXTEND_LINK_SOLANOID_EXTEND = 2;
    public static final int ARM_EXTEND_LINK_SOLANOID_RETRACT = 5;
}
