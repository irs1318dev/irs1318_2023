package frc.robot;

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

    // change INVERT_THROTTLE_AXIS to true if positive on the joystick isn't forward, and negative isn't backwards.
    public static final boolean INVERT_THROTTLE_AXIS = true;

    public static final int PCM_A_MODULE = 0;
    public static final int PCM_B_MODULE = 1;

    public static final int JOYSTICK_DRIVER_PORT = 0;
    public static final int JOYSTICK_CO_DRIVER_PORT = 1;

    //================================================== Auto ==============================================================

    public static final int AUTO_DIP_SWITCH_A_DIGITAL_CHANNEL = -1;

    //================================================== Vision ==============================================================

    public static final int VISION_RING_LIGHT_PCM_CHANNEL = 2;

    //================================================== DriveTrain ==============================================================

    public static final int DRIVETRAIN_LEFT_MASTER_CAN_ID = 1;
    public static final int DRIVETRAIN_LEFT_FOLLOWER1_CAN_ID = 2;
    public static final int DRIVETRAIN_LEFT_FOLLOWER2_CAN_ID = 3;
    public static final int DRIVETRAIN_RIGHT_MASTER_CAN_ID = 4;
    public static final int DRIVETRAIN_RIGHT_FOLLOWER1_CAN_ID = 5;
    public static final int DRIVETRAIN_RIGHT_FOLLOWER2_CAN_ID = 6;
}
