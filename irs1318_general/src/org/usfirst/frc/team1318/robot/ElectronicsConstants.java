package org.usfirst.frc.team1318.robot;

/**
 * All constants describing how the electronics are plugged together.
 * 
 * @author Will
 * 
 */
public class ElectronicsConstants
{
    // change INVERT_X_AXIS to true if positive on the joystick isn't to the right, and negative isn't to the left
    public static final boolean INVERT_X_AXIS = false;

    // change INVERT_Y_AXIS to true if positive on the joystick isn't forward, and negative isn't backwards.
    public static final boolean INVERT_Y_AXIS = true;

    public static final double MAX_POWER_LEVEL = 1.0;

    public static final int PCM_A_MODULE = 1;
    public static final int PCM_B_MODULE = 0;

    public static final int JOYSTICK_DRIVER_PORT = 0;
    public static final int JOYSTICK_CO_DRIVER_PORT = 1;

    //================================================== DriveTrain ==============================================================

    public static final int DRIVETRAIN_LEFT_TALON_CHANNEL = 2;
    public static final int DRIVETRAIN_RIGHT_TALON_CHANNEL = 1;

    public static final int DRIVETRAIN_RIGHT_ENCODER_CHANNEL_A = 2;
    public static final int DRIVETRAIN_RIGHT_ENCODER_CHANNEL_B = 3;

    public static final int DRIVETRAIN_LEFT_ENCODER_CHANNEL_A = 4;
    public static final int DRIVETRAIN_LEFT_ENCODER_CHANNEL_B = 5;

    public static final int DRIVETRAIN_PROXIMITY_SENSOR_BACK_PORT = 1;
    public static final int DRIVETRAIN_PROXIMITY_SENSOR_FRONT_PORT = 2;

    //================================================== Autonomous ==============================================================

    public static final int AUTONOMOUS_DIP_SWITCH_A = 8;
    public static final int AUTONOMOUS_DIP_SWITCH_B = 9;
}
