package org.usfirst.frc.team1318.robot;

/**
 * All constants describing how the electronics are plugged together.
 * 
 * @author Will
 * 
 */
public class ElectronicsConstants
{
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

    public static final int INTAKE_LEFT_TALON_CHANNEL = -1; // TODO
    public static final int INTAKE_RIGHT_TALON_CHANNEL = -1; // TODO

    public static final int INTAKE_LEFT_ARM_CHANNEL_A = -1; // TODO
    public static final int INTAKE_LEFT_ARM_CHANNEL_B = -1; // TODO

    public static final int INTAKE_RIGHT_ARM_CHANNEL_A = -1; // TODO
    public static final int INTAKE_RIGHT_ARM_CHANNEL_B = -1; // TODO

    public static final double COMPRESSOR_MAX_PSI = 150.0;
    public static final double COMPRESSOR_MAX_VOLTAGE = 10.0;
}
