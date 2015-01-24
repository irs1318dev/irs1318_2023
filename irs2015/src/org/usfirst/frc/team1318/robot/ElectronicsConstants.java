package org.usfirst.frc.team1318.robot;

/**
 * All constants describing how the electronics are plugged together.
 * 
 * @author Will
 * 
 */
public class ElectronicsConstants
{
    public static final int SOLENOID_MODULE_PORT_1 = 3;
    public static final int SOLENOID_MODULE_PORT_2 = 2;

    public static final int DRIVETRAIN_LEFT_TALON_CHANNEL = 9;
    public static final int DRIVETRAIN_RIGHT_TALON_CHANNEL = 8;

    public static final int DRIVETRAIN_RIGHT_ENCODER_CHANNEL_A = 6;
    public static final int DRIVETRAIN_RIGHT_ENCODER_CHANNEL_B = 7;

    public static final int DRIVETRAIN_LEFT_ENCODER_CHANNEL_A = 8;
    public static final int DRIVETRAIN_LEFT_ENCODER_CHANNEL_B = 9;

    public static final int DRIVETRAIN_SHIFTER_MODE_EXTENDER_PORT = 1;
    public static final int DRIVETRAIN_SHIFTER_MODE_RETRACTER_PORT = 0;

    public static final int COMPRESSOR_PRESSURE_SWITCH_CHANNEL = 6;
    public static final int COMPRESSOR_RELAY_CHANNEL = 1;

    public static final int COMPRESSOR_ANALOG_PRESSURE_SENSOR = 4;

    public static final double COMPRESSOR_MAX_PSI = 150.0;
    public static final double COMPRESSOR_MAX_VOLTAGE = 10.0;

    public static final int TROMBONE_SOLANOID_EXTEND = -1;
    public static final int TROMBONE_SOLANOID_RETRACT = -1;
    public static final int MOVE_LINK_SOLANOID_EXTEND = -1;
    public static final int MOVE_LINK_SOLANOID_RETRACT = -1;
    public static final int EXTEND_LINK_SOLANOID_EXTEND = -1;
    public static final int EXTEND_LINK_SOLANOID_RETRACT = -1;
}
