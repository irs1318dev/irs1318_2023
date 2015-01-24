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
    public static final int JOYSTICK_PORT = 0;

    public static final int DRIVETRAIN_SIMPLE_BUTTON = 3;

    public static final int DRIVETRAIN_SHIFTER_BUTTON = 4;

    // change INVERT_X_AXIS to true if positive on the joystick isn't to the right, and negative isn't to the left
    public static final boolean INVERT_X_AXIS = false;

    // change INVERT_Y_AXIS to true if positive on the joystick isn't forward, and negative isn't backwards.
    public static final boolean INVERT_Y_AXIS = true;
}
