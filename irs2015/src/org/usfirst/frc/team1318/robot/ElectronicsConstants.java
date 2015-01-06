package org.usfirst.frc.team1318.robot;

import edu.wpi.first.wpilibj.SensorBase;

/**
 * All constants describing how the electronics are plugged together.
 * 
 * @author Will
 * 
 */
public class ElectronicsConstants
{
    public static final int SIDECAR_SLOT = 0; //SensorBase.getDefaultDigitalModule();
    public static final int DIGITAL_IO = SensorBase.getDefaultSolenoidModule();
    public static final int ANALOG_MODULE = 0; //SensorBase.getDefaultAnalogModule();

    public static final int SOLENOID_MODULE_PORT_1 = 1;
    public static final int SOLENOID_MODULE_PORT_2 = 2;

    public static final int DRIVETRAIN_LEFT_TALON_CHANNEL = 1;
    public static final int DRIVETRAIN_RIGHT_TALON_CHANNEL = 2;

    public static final int DRIVETRAIN_RIGHT_ENCODER_CHANNEL_A = 1; 
    public static final int DRIVETRAIN_RIGHT_ENCODER_CHANNEL_B = 5; 

    public static final int DRIVETRAIN_LEFT_ENCODER_CHANNEL_A = 2;
    public static final int DRIVETRAIN_LEFT_ENCODER_CHANNEL_B = 3;

    public static final int COLLECTOR_MOTOR_CHANNEL = 3;

    public static final int COLLECTOR_SOLENOID_MODULE_PORT = 2;

    public static final int COLLECTOR_EXTENDER_SOLENOID_CHANNEL = 4;
    public static final int COLLECTOR_RETRACTOR_SOLENOID_CHANNEL = 3;
    
    public static final int COLLECTOR_LIMIT_SWITCH_PORT = 0;

    public static final int COMPRESSOR_PRESSURE_SWITCH_CHANNEL = 6;
    public static final int COMPRESSOR_RELAY_CHANNEL = 1;
    
    public static final int COMPRESSOR_ANALOG_PRESSURE_SENSOR = 4;
    
    public static final double COMPRESSOR_MAX_PSI = 150.0;
    public static final double COMPRESSOR_MAX_VOLTAGE = 10.0;
    
    public static final int SHOOTER_ANGLE_EXTENDER_SOLENOID_PORT = 4;
    public static final int SHOOTER_ANGLE_RETRACTOR_SOLENOID_PORT = 3;

    public static final int SHOOTER_MIDDLE_SOLENOID_EXTENDER_PORT = 5;
    public static final int SHOOTER_MIDDLE_SOLENOID_RETRACTOR_PORT = 6;
    
    public static final int SHOOTER_INNER_LEFT_SOLENOID_EXTENDER_PORT = 3;
    public static final int SHOOTER_INNER_LEFT_SOLENOID_RETRACTOR_PORT = 4;
    
    public static final int SHOOTER_INNER_RIGHT_SOLENOID_EXTENDER_PORT = 7;
    public static final int SHOOTER_INNER_RIGHT_SOLENOID_RETRACTOR_PORT = 8;
    
    public static final int SHOOTER_OUTER_LEFT_SOLENOID_EXTENDER_PORT = 7;
    public static final int SHOOTER_OUTER_LEFT_SOLENOID_RETRACTOR_PORT = 8; 
    
    public static final int SHOOTER_OUTER_RIGHT_SOLENOID_EXTENDER_PORT = 1;
    public static final int SHOOTER_OUTER_RIGHT_SOLENOID_RETRACTOR_PORT = 2; 
}
