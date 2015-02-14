package org.usfirst.frc.team1318.robot;

/**
 * All constants that describe which button on the joystick maps to which action in the user driver.
 * 
 * Button guide:
 * -----------------------
 * Logitech Xtreme 3D Pro
 * 1 - stick trigger
 * 2 - stick thumb button
 * 3 - stick bottom left
 * 4 - stick bottom right
 * 5 - stick top left
 * 6 - stick top right
 * 7 - base top left
 * 8 - base top right
 * 9 - base middle left
 * 10 - base middle right
 * 11 - base bottom left
 * 12 - base bottom right
 * -----------------------
 * 
 * @author Will
 * 
 */
public class JoystickButtonConstants
{
    public static final int JOYSTICK_DRIVER_PORT = 0;
    public static final int JOYSTICK_CO_DRIVER_PORT = 1;

    // change INVERT_X_AXIS to true if positive on the joystick isn't to the right, and negative isn't to the left
    public static final boolean INVERT_X_AXIS = false;

    // change INVERT_Y_AXIS to true if positive on the joystick isn't forward, and negative isn't backwards.
    public static final boolean INVERT_Y_AXIS = true;

    //================================================== DriveTrain ==============================================================

    public static final int DRIVETRAIN_SIMPLE_BUTTON = 3;

    //================================================== Elevator ==============================================================

    public static final int ELEVATOR_CONTAINER_MACRO_BUTTON = -1;

    public static final int ELEVATOR_SET_STAE_TO_FLOOR_BUTTON = 12; //1
    public static final int ELEVATOR_SET_STATE_TO_PLATFORM_BUTTON = 10; //1
    public static final int ELEVATOR_SET_STATE_TO_STEP_BUTTON = 8;  //1
    public static final int ELEVATOR_MOVE_TO_0_TOTES_BUTTON = 2;    //1
    public static final int ELEVATOR_MOVE_TO_1_TOTE_BUTTON = 11;    //1
    public static final int ELEVATOR_MOVE_TO_2_TOTES_BUTTON = 9;    //1
    public static final int ELEVATOR_MOVE_TO_3_TOTES_BUTTON = 7;    //1
    public static final int ELEVATOR_PID_ON = 6;    //2
    public static final int ELEVATOR_PID_OFF = 4;   //2
    public static final int ELEVATOR_STOP_BUTTON = 1;   //2
    public static final int ELEVATOR_UP_BUTTON = 5; //1
    public static final int ELEVATOR_DOWN_BUTTON = 3;   //1
    public static final int ELEVATOR_MOVE_TO_BOTTOM = 2;    //2
    public static final int ELEVATOR_IGNORE_SENSORS_BUTTON = 5; //2
    public static final int ELEVATOR_USE_SENSORS_BUTTON = 3;    //2
    public static final int ELEVATOR_ZERO_ENCODERS = 12;    //2

    //================================================== Arm ==============================================================

    public static final int ARM_MACRO_EXTEND_BUTTON = -1;
    public static final int ARM_MACRO_RETRACT_BUTTON = -1;
    public static final int ARM_EXTENDER_EXTEND_BUTTON = -1;
    public static final int ARM_EXTENDER_RETRACT_BUTTON = -1;
    public static final int ARM_TILT_EXTEND_BUTTON = -1;
    public static final int ARM_TILT_RETRACT_BUTTON = -1;
    public static final int ARM_TROMBONE_EXTEND_BUTTON = -1;
    public static final int ARM_TROMBONE_RETRACT_BUTTON = -1;

    //================================================== Intake ==============================================================

    public static final int INTAKE_UP_BUTTON = -1;
    public static final int INTAKE_DOWN_BUTTON = -1;
    public static final int INTAKE_RIGHT_EXTEND_OVERRIDE = -1;
    public static final int INTAKE_RIGHT_RETRACT_OVERRIDE = -1;
    public static final int INTAKE_LEFT_EXTEND_OVERRIDE = -1;
    public static final int INTAKE_LEFT_RETRACT_OVERRIDE = -1;
    public static final int INTAKE_FORWARD_BUTTON = -1;
    public static final int INTAKE_BACKWARD_BUTTON = -1;
}
