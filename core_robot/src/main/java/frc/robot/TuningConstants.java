package frc.robot;

/**
 * All constants related to tuning the operation of the robot.
 * 
 * @author Will
 * 
 */
public class TuningConstants
{
    public static final boolean COMPETITION_ROBOT = false;
    public static boolean THROW_EXCEPTIONS = !TuningConstants.COMPETITION_ROBOT;
    public static boolean LOG_EXCEPTIONS = true;
    public static double LOOP_DURATION = 0.02; // we expect the robot's main loop to run at roughly ~50 Hz, or 1 update per 20ms (0.02s)
    public static int LOOPS_PER_SECOND = 50; // we expect the robot's main loop to run at roughly ~50 Hz, or 1 update per 20ms (0.02s)

    public static final boolean EXPECT_UNUSED_JOYSTICKS = true;

    //================================================== Magic Values ==============================================================

    public static final double MAGIC_NULL_VALUE = -1318.0;
    public static final double ZERO = 0.0;
    public static final double ENDGAME_START_TIME = 30.0;
    public static final double ENDGAME_CLIMB_TIME = 5.0;

    //================================================== Logging  ==============================================================

    public static final int CALENDAR_YEAR = 2022;
    public static final boolean LOG_TO_FILE = true; // TuningConstants.COMPETITION_ROBOT;
    public static final boolean LOG_FILE_ONLY_COMPETITION_MATCHES = false;
    public static final long LOG_FILE_REQUIRED_FREE_SPACE = 50 * 1024 * 1024; // require at least 50 MB of space
    public static final int LOG_FLUSH_THRESHOLD = 25;

    //================================================= Power ======================================================

    public static final double POWER_OVERCURRENT_TRACKING_DURATION = 5.0; // duration of time to keep track of the average current
    public static final double POWER_OVERCURRENT_SAMPLES_PER_LOOP = 1.0; // we may want to increase this if we find our update loop duration isn't very consistent...
    public static final double POWER_OVERCURRENT_SAMPLES_PER_SECOND = TuningConstants.LOOPS_PER_SECOND * TuningConstants.POWER_OVERCURRENT_SAMPLES_PER_LOOP;
    public static final double POWER_OVERCURRENT_THRESHOLD = 120.0;
    public static final double POWER_OVERCURREHT_HIGH_THRESHOLD = 160.0;

    //================================================= Vision ======================================================

    // Acceptable vision centering range values in degrees
    public static final double MAX_VISION_CENTERING_RANGE_DEGREES = 5.0;

    // How long the robot system must remain centered on the target when using time
    public static final double VISION_CENTERING_DURATION = 0.75;

    // Acceptable vision distance from tape in inches (as measured by vision system)
    public static final double MAX_VISION_ACCEPTABLE_FORWARD_DISTANCE = 1.75;

    // PID settings for Centering the robot on a vision target from one stationary place
    public static final double VISION_STATIONARY_CENTERING_PID_KP = 0.025;
    public static final double VISION_STATIONARY_CENTERING_PID_KI = 0.0;
    public static final double VISION_STATIONARY_CENTERING_PID_KD = 0.01;
    public static final double VISION_STATIONARY_CENTERING_PID_KF = 0.0;
    public static final double VISION_STATIONARY_CENTERING_PID_KS = 1.0;
    public static final double VISION_STATIONARY_CENTERING_PID_MIN = -0.4;
    public static final double VISION_STATIONARY_CENTERING_PID_MAX = 0.4;

    // PID settings for Centering the robot on a vision target
    public static final double VISION_MOVING_CENTERING_PID_KP = 0.012;
    public static final double VISION_MOVING_CENTERING_PID_KI = 0.0;
    public static final double VISION_MOVING_CENTERING_PID_KD = 0.0;
    public static final double VISION_MOVING_CENTERING_PID_KF = 0.0;
    public static final double VISION_MOVING_CENTERING_PID_KS = 1.0;
    public static final double VISION_MOVING_CENTERING_PID_MIN = -0.3;
    public static final double VISION_MOVING_CENTERING_PID_MAX = 0.3;

    // PID settings for Advancing the robot towards a vision target
    public static final double VISION_ADVANCING_PID_KP = 0.015;
    public static final double VISION_ADVANCING_PID_KI = 0.0;
    public static final double VISION_ADVANCING_PID_KD = 0.0;
    public static final double VISION_ADVANCING_PID_KF = 0.0;
    public static final double VISION_ADVANCING_PID_KS = 1.0;
    public static final double VISION_ADVANCING_PID_MIN = -0.3;
    public static final double VISION_ADVANCING_PID_MAX = 0.3;

    // PID settings for Advancing the robot quickly towards a vision target
    public static final double VISION_FAST_ADVANCING_PID_KP = 0.15;
    public static final double VISION_FAST_ADVANCING_PID_KI = 0.0;
    public static final double VISION_FAST_ADVANCING_PID_KD = 0.0;
    public static final double VISION_FAST_ADVANCING_PID_KF = 0.0;
    public static final double VISION_FAST_ADVANCING_PID_KS = 1.0;
    public static final double VISION_FAST_ADVANCING_PID_MIN = -0.45;
    public static final double VISION_FAST_ADVANCING_PID_MAX = 0.45;

    public static final int VISION_MISSED_HEARTBEAT_THRESHOLD = 500;

    //================================================== Indicator Lights ========================================================

    public static final double INDICATOR_LIGHT_VISION_ACCEPTABLE_ANGLE_RANGE = 3.0;

    public static final int CANDLE_LED_COUNT = 8;
    public static final int LED_STRIP_LED_COUNT = 60; // 60 LEDs per meter-long strip from CTRE
    public static final int CANDLE_TOTAL_NUMBER_LEDS = TuningConstants.CANDLE_LED_COUNT; //+ TuningConstants.LED_STRIP_LED_COUNT; // * 2;

    public static final int CANDLE_ANIMATION_SLOT_1 = 0;
    public static final int CANDLE_ANIMATION_SLOT_2 = 1;

    // IRS1318 Purple color
    public static final int INDICATOR_PURPLE_RED = 101;
    public static final int INDICATOR_PURPLE_GREEN = 34;
    public static final int INDICATOR_PURPLE_BLUE = 129;
    public static final int INDICATOR_PURPLE_WHITE = 0;

    // No color
    public static final int INDICATOR_OFF_COLOR_RED = 0;
    public static final int INDICATOR_OFF_COLOR_GREEN = 0;
    public static final int INDICATOR_OFF_COLOR_BLUE = 0;
    public static final int INDICATOR_OFF_COLOR_WHITE = 0;

    // Bright Red color
    public static final int INDICATOR_RED_COLOR_RED = 255;
    public static final int INDICATOR_RED_COLOR_GREEN = 0;
    public static final int INDICATOR_RED_COLOR_BLUE = 0;
    public static final int INDICATOR_RED_COLOR_WHITE = 0;

    // Bright Yellow color
    public static final int INDICATOR_YELLOW_COLOR_RED = 255;
    public static final int INDICATOR_YELLOW_COLOR_GREEN = 255;
    public static final int INDICATOR_YELLOW_COLOR_BLUE = 0;
    public static final int INDICATOR_YELLOW_COLOR_WHITE = 0;

    // Bright Green color
    public static final int INDICATOR_GREEN_COLOR_RED = 0;
    public static final int INDICATOR_GREEN_COLOR_GREEN = 255;
    public static final int INDICATOR_GREEN_COLOR_BLUE = 0;
    public static final int INDICATOR_GREEN_COLOR_WHITE = 0;

    public static final double COMPRESSOR_FILL_RATE = 10.0;
    public static final double COMPRESSOR_ENOUGH_PRESSURE = 110.0;

    //================================================== DriveTrain ==============================================================

    public static final boolean DRIVETRAIN_STEER_MOTORS_USE_MOTION_MAGIC = true;

    public static final boolean DRIVETRAIN_USE_ODOMETRY = true;
    public static final boolean DRIVETRAIN_RESET_ON_ROBOT_START = true;
    public static final boolean DRIVETRAIN_FIELD_ORIENTED_ON_ROBOT_START = true;
    public static final boolean DRIVETRAIN_MAINTAIN_ORIENTATION_ON_ROBOT_START = true;

    public static final double DRIVETRAIN_STEER_MOTOR1_ABSOLUTE_OFFSET = 128.056;
    public static final double DRIVETRAIN_STEER_MOTOR2_ABSOLUTE_OFFSET = -16.259;
    public static final double DRIVETRAIN_STEER_MOTOR3_ABSOLUTE_OFFSET = 2.725;
    public static final double DRIVETRAIN_STEER_MOTOR4_ABSOLUTE_OFFSET = -30.586;

    // Position PID (angle) per-module
    public static final double DRIVETRAIN_STEER_MOTOR_POSITION_PID_KS = HardwareConstants.DRIVETRAIN_STEER_TICKS_PER_DEGREE;

    public static final double DRIVETRAIN_STEER_MOTORS_POSITION_PID_KP = 0.5;
    public static final double DRIVETRAIN_STEER_MOTORS_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_STEER_MOTORS_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_STEER_MOTORS_POSITION_PID_KF = 0.0;

    public static final double DRIVETRAIN_STEER_MOTORS_MM_PID_KP = 0.5;
    public static final double DRIVETRAIN_STEER_MOTORS_MM_PID_KI = 0.0;
    public static final double DRIVETRAIN_STEER_MOTORS_MM_PID_KD = 0.0;
    public static final double DRIVETRAIN_STEER_MOTORS_MM_PID_KF = 0.34; // 1023 over max speed (3000 ticks per 100ms)
    public static final int DRIVETRAIN_STEER_MOTORS_MM_PID_CRUISE_VELOC = 48000;
    public static final int DRIVETRAIN_STEER_MOTORS_MM_PID_ACCEL = 48000;

    // Velocity PID (drive) per-module
    public static final double DRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS = 16000.0; // 20000 was highest speed at full throttle FF on blocks. this is #ticks / 100ms

    public static final double DRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KP = 0.1;
    public static final double DRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KI = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KD = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KF = 0.05115; // .05115 ==> ~ 1023 / 20000 (100% control authority)

    public static final double DRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KP = 1.0;
    public static final double DRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KF = 0.0;

    public static final double DRIVETRAIN_OMEGA_POSITION_PID_KP = 0.1;
    public static final double DRIVETRAIN_OMEGA_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_OMEGA_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_OMEGA_POSITION_PID_KF = 0.0;
    public static final double DRIVETRAIN_OMEGA_POSITION_PID_KS = 1.0;
    public static final double DRIVETRAIN_OMEGA_MAX_OUTPUT = 5.0;
    public static final double DRIVETRAIN_OMEGA_MIN_OUTPUT = -5.0;

    public static final double DRIVETRAIN_PATH_OMEGA_POSITION_PID_KP = 0.1;
    public static final double DRIVETRAIN_PATH_OMEGA_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_PATH_OMEGA_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_PATH_OMEGA_POSITION_PID_KF = 0.0;
    public static final double DRIVETRAIN_PATH_OMEGA_POSITION_PID_KS = 1.0;
    public static final double DRIVETRAIN_PATH_OMEGA_MAX_OUTPUT = 4.0;
    public static final double DRIVETRAIN_PATH_OMEGA_MIN_OUTPUT = -4.0;

    public static final double DRIVETRAIN_PATH_X_POSITION_PID_KP = 0.0; // 1.0;
    public static final double DRIVETRAIN_PATH_X_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_PATH_X_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_PATH_X_POSITION_PID_KF = 0.0;
    public static final double DRIVETRAIN_PATH_X_POSITION_PID_KS = 1.0;
    public static final double DRIVETRAIN_PATH_X_MAX_OUTPUT = 10.0;
    public static final double DRIVETRAIN_PATH_X_MIN_OUTPUT = -10.0;

    public static final double DRIVETRAIN_PATH_Y_POSITION_PID_KP = 0.0; // 1.0;
    public static final double DRIVETRAIN_PATH_Y_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_PATH_Y_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_PATH_Y_POSITION_PID_KF = 0.0;
    public static final double DRIVETRAIN_PATH_Y_POSITION_PID_KS = 1.0;
    public static final double DRIVETRAIN_PATH_Y_MAX_OUTPUT = 10.0;
    public static final double DRIVETRAIN_PATH_Y_MIN_OUTPUT = -10.0;

    public static final boolean DRIVETRAIN_USE_OVERCURRENT_ADJUSTMENT = true;
    public static final double DRIVETRAIN_OVERCURRENT_ADJUSTMENT = 0.75;
    public static final double DRIVETRAIN_OVERCURRENT_HIGH_ADJUSTMENT = 0.5;

    public static final boolean DRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED = true;
    public static final double DRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION = 11.0;
    public static final boolean DRIVETRAIN_DRIVE_SUPPLY_CURRENT_LIMITING_ENABLED = true;
    public static final double DRIVETRAIN_DRIVE_SUPPLY_CURRENT_MAX = 35.0;
    public static final double DRIVETRAIN_DRIVE_SUPPLY_TRIGGER_CURRENT = 35.0;
    public static final double DRIVETRAIN_DRIVE_SUPPLY_TRIGGER_DURATION = 0.25;

    public static final boolean DRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED = true;
    public static final double DRIVETRAIN_STEER_VOLTAGE_COMPENSATION = 11.0;
    public static final boolean DRIVETRAIN_STEER_SUPPLY_CURRENT_LIMITING_ENABLED = true;
    public static final double DRIVETRAIN_STEER_SUPPLY_CURRENT_MAX = 20.0;
    public static final double DRIVETRAIN_STEER_SUPPLY_TRIGGER_CURRENT = 30.0;
    public static final double DRIVETRAIN_STEER_SUPPLY_TRIGGER_DURATION = 0.1;

    public static final int DRIVETRAIN_SENSOR_FRAME_PERIOD_MS = 10;
    public static final int DRIVETRAIN_PID_FRAME_PERIOD_MS = 100;

    public static final boolean DRIVETRAIN_SKIP_ANGLE_ON_ZERO_VELOCITY = true;
    public static final double DRIVETRAIN_SKIP_ANGLE_ON_ZERO_DELTA = 0.001;
    public static final double DRIVETRAIN_SKIP_OMEGA_ON_ZERO_DELTA = 0.25;

    public static final double DRIVETRAIN_DEAD_ZONE_TURN = 0.1;
    public static final double DRIVETRAIN_DEAD_ZONE_VELOCITY = 0.15;
    public static final double DRIVETRAIN_DEAD_ZONE_TRIGGER_AB = 0.15;

    public static final double DRIVETRAIN_ROTATION_A_MULTIPLIER = HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE / 2.0;
    public static final double DRIVETRAIN_ROTATION_B_MULTIPLIER = HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE / 2.0;

    public static final double DRIVETRAIN_MAX_VELOCITY = TuningConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS * HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND; // max velocity in inches per second
    public static final double DRIVETRAIN_VELOCITY_TO_PERCENTAGE = 1.0 / TuningConstants.DRIVETRAIN_MAX_VELOCITY;
    public static final double DRIVETRAIN_TURN_GOAL_VELOCITY = 10.0; // degrees per second for turn goal
    public static final double DRIVETRAIN_TURN_SCALE = 4.0; // radians per second
    public static final double DRIVETRAIN_STATIONARY_VELOCITY = 0.1;
    public static final double DRIVETRAIN_TURN_APPROXIMATION_STATIONARY = 2.0; // number of degrees off at which point we give up trying to face an angle when uncommanded
    public static final double DRIVETRAIN_TURN_APPROXIMATION = 1.0; // number of degrees off at which point we give up trying to face an angle when uncommanded
    public static final double DRIVETRAIN_MAX_MODULE_PATH_VELOCITY = 0.85 * TuningConstants.DRIVETRAIN_MAX_VELOCITY; // up to x% of our max controllable speed
    public static final double DRIVETRAIN_MAX_PATH_TURN_VELOCITY = 180.0; // in degrees per second
    public static final double DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY = 0.80 * TuningConstants.DRIVETRAIN_MAX_VELOCITY; // in inches per second
    public static final double DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION = 0.75 * TuningConstants.DRIVETRAIN_MAX_VELOCITY; // in inches per second per second

    //================================================= ARM ================================================================================

    public static final double INTAKE_POWER = 0.6;

    public static final double LOWER_ARM_POSITION_MM_PID_KP = 0.1;
    public static final double LOWER_ARM_POSITION_MM_PID_KI = 0.0;
    public static final double LOWER_ARM_POSITION_MM_PID_KD = 0.0;
    public static final double LOWER_ARM_POSITION_MM_PID_KF = 0.05;
    public static final double LOWER_ARM_POSITION_MM_CRUISE_VELOCITY = 0.0;
    public static final double LOWER_ARM_POSITION_MM_ACCELERATION = 0.0;

    public static final double UPPER_ARM_POSITION_MM_PID_KP = 0.1;
    public static final double UPPER_ARM_POSITION_MM_PID_KI = 0.0;
    public static final double UPPER_ARM_POSITION_MM_PID_KD = 0.0;
    public static final double UPPER_ARM_POSITION_MM_PID_KF = 0.05;
    public static final double UPPER_ARM_POSITION_MM_CRUISE_VELOCITY = 0.0;
    public static final double UPPER_ARM_POSITION_MM_ACCELERATION = 0.0;

    public static final boolean LOWER_ARM_INVERT_OUTPUT = false;
    public static final boolean UPPER_ARM_INVERT_OUTPUT = false;

    public static final boolean LOWER_ARM_INVERT_SENSOR = false;
    public static final boolean UPPER_ARM_INVERT_SENSOR = false;

    public static final double LOWER_ARM_FULL_EXTENTION_LENGTH = 8.0;
    public static final double UPPER_ARM_FULL_RETRACTED_LENGTH = 0.0;
    public static final double LOWER_ARM_NEAR_FULL_EXTENSION_LENGTH = LOWER_ARM_FULL_EXTENTION_LENGTH * 0.9;
    public static final double UPPER_ARM_NEAR_FULL_RETRACTED_LENGTH = UPPER_ARM_FULL_RETRACTED_LENGTH + 0.1;

    public static final double ARM_RETRACTION_MAX_TIME = 0.7;
    

    public static final double ARM_MAX_VELOCITY = 50.0;
    public static final double ARM_MAX_ACCLERATION = 50.0;

    public static final double ARM_MAX_SIMPLE_VELOCITY = 0.8;
 
    public static final double ARM_STRING_ENCODER_TICKS_PER_INCH = (4096 / 100.0) * 25.4; // .1 mm per tick

    public static final double ARM_MAX_HEIGHT = 78.0;
    public static final double ARM_MAX_LENGTH = 48.0;

    public static final double LOWER_ARM_VELOCITY_DEAZONE = 0.1;

    //============================================= ChargeStation Macro ==================================================================
    public static final double CHARGE_STATION_STEP_ONE_GOAL_ANGLE = 15;
    public static final double CHARGE_STATION_STEP_TWO_GOAL_ANGLE = 9;
    public static final double CHARGE_STATION_STEP_THREE_GOAL_POSITION = 4.5;

    //ChargeStationTask constants (owned by Jamie and Calvin)
    public static final double CHARGE_STATION_PITCH_VARIATION = 0.3;
    public static final double CHARGE_STATION_ACCEPTABLE_PITCH_DIFF = 2.0;
    public static final double CHARGE_STATION_STARTING_SPEED = 0.5;
    public static final double CHARGE_STATION_CLIMBING_SPEED = 0.2;
    public static final double CHARGE_STATION_BALANCING_SPEED = 0.06;
    public static final double CHARGE_STATION_START_TRANSITION_PITCH = 5.0;
    public static final double CHARGE_STATION_CLIMBING_TRANSITION_PITCH = 13;
    public static final double CHARGE_STATION_CLIMBING_TRANSITION_ACCEPTABLE_VARIATION = 0.3;

}
