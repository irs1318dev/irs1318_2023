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

    public static final int PCM_A_MODULE = 1;//1;
    public static final int PCM_B_MODULE = 0;//0;

    //================================================== DriveTrain ==============================================================

    public static final int DRIVETRAIN_LEFT_TALON_CHANNEL = 2;// kitbot - 9
    public static final int DRIVETRAIN_RIGHT_TALON_CHANNEL = 1;// kitbot - 8

    public static final int DRIVETRAIN_RIGHT_ENCODER_CHANNEL_A = 2;// kitbot - 6
    public static final int DRIVETRAIN_RIGHT_ENCODER_CHANNEL_B = 3;// kitbot - 7

    public static final int DRIVETRAIN_LEFT_ENCODER_CHANNEL_A = 4;// kitbot - 8
    public static final int DRIVETRAIN_LEFT_ENCODER_CHANNEL_B = 5;// kitbot - 9

    public static final int DRIVETRAIN_PROXIMITY_SENSOR_BACK_PORT = 1;
    public static final int DRIVETRAIN_PROXIMITY_SENSOR_FRONT_PORT = 2;

    //================================================== Autonomous ==============================================================

    public static final int AUTONOMOUS_DIP_SWITCH_A = 8;
    public static final int AUTONOMOUS_DIP_SWITCH_B = 9;
}
