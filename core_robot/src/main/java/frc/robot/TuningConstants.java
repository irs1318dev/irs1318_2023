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

    public static final int CALENDAR_YEAR = 2023;
    public static final boolean LOG_TO_FILE = true; // TuningConstants.COMPETITION_ROBOT;
    public static final boolean LOG_FILE_ONLY_COMPETITION_MATCHES = false;
    public static final long LOG_FILE_REQUIRED_FREE_SPACE = 50 * 1024 * 1024; // require at least 50 MB of space
    public static final int LOG_FLUSH_THRESHOLD = 25;

    //================================================== Autonomous ==============================================================

    public static final boolean TRAJECTORY_FORCE_BUILD = false;

    // Y Values
    public static final double StartOneGridY = 196.19;
    public static final double StartTwoGridY = 174.19; // April Tag ID's 6 and 3
    public static final double StartThreeGridY = 152.19;
    public static final double StartFourGridY = 130.19;
    public static final double StartFiveGridY = 108.19; // April Tag ID's 7 and 2
    public static final double StartSixGridY = 86.19;
    public static final double StartSevenGridY = 64.19;
    public static final double StartEightGridY = 42.19; // April Tag ID's 8 and 1
    public static final double StartNineGridY = 20.19;
    public static final double ChargeStationY = 108.015;
    public static final double GroundOneY = 180.19; // Real: 180.19
    public static final double GroundTwoY = 132.19;
    public static final double GroundThreeY = 84.19;
    public static final double GroundFourY = 36.19;
    public static final double LoadEdgeY = 198.53; // Edge of grid - 17.5
    public static final double GuardEdgeY = 17.5;

    // X Values
    public static final double StartGridX = 253.86; // Edge of grid - Robot centering value
    public static final double CloseChargeStationX = 241.015; // 12.845 inches away from the charge station and grid + Robot centering value
    public static final double FarChargeStationX = 102.99; // 12 inches away from the charge station + Robot centering value
    public static final double FarChargeStationInBetweenX = TuningConstants.FarChargeStationX - 30.0; // 30 inches away from last point to allow for turning
    public static final double GroundPiecesX = 47.36; // On ground pieces
    public static final double LoadEdgeStartX = 214.86;
    public static final double GuardEdgeStartX = 193.61;

    // April tag array by ids
    // (xPosition, yPosition, orientation)
    public static final double[][] AprilTagLocations =
        {
            { 285.16,   42.19,   0.0 }, // ID 1
            { 285.16,  108.19,   0.0 }, // ID 2
            { 285.16,  174.19,   0.0 }, // ID 3
            { 311.35,  265.74,   0.0 }, // ID 4
            { -285.16,  42.19, 180.0 }, // ID 5
            { -285.16, 108.19, 180.0 }, // ID 6
            { -285.16, 174.19, 180.0 }, // ID 7
            { -311.35, 265.74, 180.0 }, // ID 8
        };

    //================================================= Power ======================================================

    public static final double POWER_OVERCURRENT_TRACKING_DURATION = 5.0; // duration of time to keep track of the average current
    public static final double POWER_OVERCURRENT_SAMPLES_PER_LOOP = 1.0; // we may want to increase this if we find our update loop duration isn't very consistent...
    public static final double POWER_OVERCURRENT_SAMPLES_PER_SECOND = TuningConstants.LOOPS_PER_SECOND * TuningConstants.POWER_OVERCURRENT_SAMPLES_PER_LOOP;
    public static final double POWER_OVERCURRENT_THRESHOLD = 120.0;
    public static final double POWER_OVERCURREHT_HIGH_THRESHOLD = 160.0;

    //================================================= Vision ======================================================

    // Acceptable vision centering range values in degrees
    public static final double MAX_PID_TURNING_RANGE_DEGREES = 5.0;

    // How long the robot system must remain centered on the target when using time
    public static final double PID_TURNING_DURATION = 0.75;

    // Acceptable vision distance from tape in inches (as measured by vision system)
    public static final double MAX_VISION_ACCEPTABLE_FORWARD_DISTANCE = 1.75;
    public static final double MAX_VISION_ACCEPTABLE_STRAFE_DISTANCE = 0.5;

    // Acceptable vision distance from tape in angles 
    public static final double MAX_VISION_ACCEPTABLE_MOVING_RR_ANGLE_ERROR = 4.0;

    // PID settings for Centering the robot on a vision target from one stationary place
    public static final double STATIONARY_PID_TURNING_PID_KP = 0.025;
    public static final double STATIONARY_PID_TURNING_PID_KI = 0.0;
    public static final double STATIONARY_PID_TURNING_PID_KD = 0.01;
    public static final double STATIONARY_PID_TURNING_PID_KF = 0.0;
    public static final double STATIONARY_PID_TURNING_PID_KS = 1.0;
    public static final double STATIONARY_PID_TURNING_PID_MIN = -0.4;
    public static final double STATIONARY_PID_TURNING_PID_MAX = 0.4;

    // PID settings for rotating the robot based on a vision target while in-motion
    public static final double VISION_MOVING_TURNING_PID_KP = 0.012;
    public static final double VISION_MOVING_TURNING_PID_KI = 0.0;
    public static final double VISION_MOVING_TURNING_PID_KD = 0.0;
    public static final double VISION_MOVING_TURNING_PID_KF = 0.0;
    public static final double VISION_MOVING_TURNING_PID_KS = 1.0;
    public static final double VISION_MOVING_TURNING_PID_MIN = -0.3;
    public static final double VISION_MOVING_TURNING_PID_MAX = 0.3;

    // PID settings for translating the robot based on a vision target
    public static final double VISION_MOVING_PID_KP = 0.015;
    public static final double VISION_MOVING_PID_KI = 0.0;
    public static final double VISION_MOVING_PID_KD = 0.0;
    public static final double VISION_MOVING_PID_KF = 0.0;
    public static final double VISION_MOVING_PID_KS = 1.0;
    public static final double VISION_MOVING_PID_MIN = -0.3;
    public static final double VISION_MOVING_PID_MAX = 0.3;

    // PID settings for translating the robot slowly based on a vision target
    public static final double VISION_SLOW_MOVING_PID_KP = 0.012;
    public static final double VISION_SLOW_MOVING_PID_KI = 0.0;
    public static final double VISION_SLOW_MOVING_PID_KD = 0.0;
    public static final double VISION_SLOW_MOVING_PID_KF = 0.0;
    public static final double VISION_SLOW_MOVING_PID_KS = 1.0;
    public static final double VISION_SLOW_MOVING_PID_MIN = -0.3;
    public static final double VISION_SLOW_MOVING_PID_MAX = 0.3;

    // PID settings for translating the robot quickly based on a vision target
    public static final double VISION_FAST_MOVING_PID_KP = 0.15;
    public static final double VISION_FAST_MOVING_PID_KI = 0.0;
    public static final double VISION_FAST_MOVING_PID_KD = 0.0;
    public static final double VISION_FAST_MOVING_PID_KF = 0.0;
    public static final double VISION_FAST_MOVING_PID_KS = 1.0;
    public static final double VISION_FAST_MOVING_PID_MIN = -0.45;
    public static final double VISION_FAST_MOVING_PID_MAX = 0.45;

    public static final int VISION_MISSED_HEARTBEAT_THRESHOLD = 500;

    //================================================== Indicator Lights ========================================================

    public static final double INDICATOR_LIGHT_VISION_ACCEPTABLE_ANGLE_RANGE = 3.0;

    public static final int CANDLE_LED_COUNT = 8;
    public static final int LED_STRIP_LED_COUNT = 60; // 60 LEDs per meter-long strip from CTRE
    public static final int CANDLE_TOTAL_NUMBER_LEDS = TuningConstants.CANDLE_LED_COUNT; //+ TuningConstants.LED_STRIP_LED_COUNT; // * 2;

    public static final int CANDLE_ANIMATION_SLOT_1 = 0;
    public static final int CANDLE_ANIMATION_SLOT_2 = 1;

    // IRS1318 Purple color - Human Player Cube Substation
    public static final int INDICATOR_PURPLE_COLOR_RED = 101;
    public static final int INDICATOR_PURPLE_COLOR_GREEN = 34;
    public static final int INDICATOR_PURPLE_COLOR_BLUE = 129;
    public static final int INDICATOR_PURPLE_COLOR_WHITE = 0;

    // Bright Yellow color - Human Player Cone Substation
    public static final int INDICATOR_YELLOW_COLOR_RED = 255;
    public static final int INDICATOR_YELLOW_COLOR_GREEN = 255;
    public static final int INDICATOR_YELLOW_COLOR_BLUE = 0;
    public static final int INDICATOR_YELLOW_COLOR_WHITE = 0;

    // Bright Green color - Can see AprilTag
    public static final int INDICATOR_GREEN_COLOR_RED = 0;
    public static final int INDICATOR_GREEN_COLOR_GREEN = 255;
    public static final int INDICATOR_GREEN_COLOR_BLUE = 0;
    public static final int INDICATOR_GREEN_COLOR_WHITE = 0;

    // Bright Red color  - Can't see AprilTag
    public static final int INDICATOR_RED_COLOR_RED = 255;
    public static final int INDICATOR_RED_COLOR_GREEN = 0;
    public static final int INDICATOR_RED_COLOR_BLUE = 0;
    public static final int INDICATOR_RED_COLOR_WHITE = 0;

    //Blue - Game piece in Intake (Either Cube or Cone)
    public static final int INDICATOR_BLUE_COLOR_RED = 0;
    public static final int INDICATOR_BLUE_COLOR_GREEN = 0;
    public static final int INDICATOR_BLUE_COLOR_BLUE = 255;
    public static final double INDICATOR_BLUE_COLOR_WHITE = 0;

    //Orange - No Game Piece in Intake
    public static final int INDICATOR_ORANGE_COLOR_RED = 255;
    public static final int INDICATOR_ORANGE_COLOR_GREEN = 128;
    public static final int INDICATOR_ORANGE_COLOR_BLUE = 0;
    public static final int INDICATOR_ORANGE_COLOR_WHITE = 0;

    //RAINBOW - When balanced on charge station
    public static final int INDICATOR_RAINBOW_BRIGHTNESS = 1;
    public static final double INDICATOR_RAINBOW_SPEED = 0.25;
    public static final boolean INDICATOR_RAINBOW_REVERSE_DIRECTION = false;

    // No color
    public static final int INDICATOR_OFF_COLOR_RED = 0;
    public static final int INDICATOR_OFF_COLOR_GREEN = 0;
    public static final int INDICATOR_OFF_COLOR_BLUE = 0;
    public static final int INDICATOR_OFF_COLOR_WHITE = 0;

    //================================================== DriveTrain ==============================================================

    public static final boolean DRIVETRAIN_STEER_MOTORS_USE_MOTION_MAGIC = true;

    public static final boolean DRIVETRAIN_USE_ODOMETRY = true;
    public static final boolean DRIVETRAIN_RESET_ON_ROBOT_START = true;
    public static final boolean DRIVETRAIN_FIELD_ORIENTED_ON_ROBOT_START = true;
    public static final boolean DRIVETRAIN_MAINTAIN_ORIENTATION_ON_ROBOT_START = true;

    public static final double DRIVETRAIN_STEER_MOTOR1_ABSOLUTE_OFFSET = 79.365;
    public static final double DRIVETRAIN_STEER_MOTOR2_ABSOLUTE_OFFSET = 52.119;
    public static final double DRIVETRAIN_STEER_MOTOR3_ABSOLUTE_OFFSET = -128.935;
    public static final double DRIVETRAIN_STEER_MOTOR4_ABSOLUTE_OFFSET = -125.419;

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

    public static final double DRIVETRAIN_PATH_OMEGA_POSITION_PID_KP = 0.2;
    public static final double DRIVETRAIN_PATH_OMEGA_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_PATH_OMEGA_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_PATH_OMEGA_POSITION_PID_KF = 0.0;
    public static final double DRIVETRAIN_PATH_OMEGA_POSITION_PID_KS = 1.0;
    public static final double DRIVETRAIN_PATH_OMEGA_MAX_OUTPUT = 4.0;
    public static final double DRIVETRAIN_PATH_OMEGA_MIN_OUTPUT = -4.0;

    public static final double DRIVETRAIN_PATH_X_POSITION_PID_KP = 1.0;
    public static final double DRIVETRAIN_PATH_X_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_PATH_X_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_PATH_X_POSITION_PID_KF = 0.0;
    public static final double DRIVETRAIN_PATH_X_POSITION_PID_KS = 1.0;
    public static final double DRIVETRAIN_PATH_X_MAX_OUTPUT = 10.0;
    public static final double DRIVETRAIN_PATH_X_MIN_OUTPUT = -10.0;

    public static final double DRIVETRAIN_PATH_Y_POSITION_PID_KP = 1.1;
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

    public static final double DRIVETRAIN_EXPONENTIAL = 2.0;
    public static final double DRIVETRAIN_DEAD_ZONE_TURN = 0.1;
    public static final double DRIVETRAIN_DEAD_ZONE_VELOCITY_X = 0.1;
    public static final double DRIVETRAIN_DEAD_ZONE_VELOCITY_Y = 0.1;
    public static final double DRIVETRAIN_DEAD_ZONE_TRIGGER_AB = 0.1;

    public static final double DRIVETRAIN_ROTATION_A_MULTIPLIER = HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE / 2.0;
    public static final double DRIVETRAIN_ROTATION_B_MULTIPLIER = HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE / 2.0;

    public static final double DRIVETRAIN_MAX_VELOCITY = TuningConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS * HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND; // max velocity in inches per second
    public static final double DRIVETRAIN_SLOW_MODE_MAX_VELOCITY = 0.5 * TuningConstants.DRIVETRAIN_MAX_VELOCITY; // max velocity in inches per second
    public static final double DRIVETRAIN_VELOCITY_TO_PERCENTAGE = 1.0 / TuningConstants.DRIVETRAIN_MAX_VELOCITY;
    public static final double DRIVETRAIN_TURN_GOAL_VELOCITY = 10.0; // degrees per second for turn goal
    public static final double DRIVETRAIN_TURN_SCALE = 1.0 * Math.PI; // radians per second
    public static final double DRIVETRAIN_SLOW_MODE_TURN_SCALE = 0.5 * TuningConstants.DRIVETRAIN_TURN_SCALE; // radians per second
    public static final double DRIVETRAIN_STATIONARY_VELOCITY = 0.1;
    public static final double DRIVETRAIN_TURN_APPROXIMATION_STATIONARY = 2.0; // number of degrees off at which point we give up trying to face an angle when uncommanded
    public static final double DRIVETRAIN_TURN_APPROXIMATION = 1.0; // number of degrees off at which point we give up trying to face an angle when uncommanded
    public static final double DRIVETRAIN_MAX_MODULE_PATH_VELOCITY = 0.85 * TuningConstants.DRIVETRAIN_MAX_VELOCITY; // up to x% of our max controllable speed
    public static final double DRIVETRAIN_MAX_PATH_TURN_VELOCITY = 180.0; // in degrees per second
    public static final double DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY = 0.60 * TuningConstants.DRIVETRAIN_MAX_VELOCITY; // in inches per second
    public static final double DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION = 0.75 * TuningConstants.DRIVETRAIN_MAX_VELOCITY; // in inches per second per second
    public static final double DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY = TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY / 1.4; // in inches per second
    public static final double DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION = TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION / 1.4; // in inches per second per second
    public static final double DRIVETRAIN_LOW_PATH_TRANSLATIONAL_VELOCITY = TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY / 2.0; // in inches per second
    public static final double DRIVETRAIN_LOW_PATH_TRANSLATIONAL_ACCELERATION = TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION / 2.0; // in inches per second per second

    //================================================= Arm ================================================================================

    public static final boolean ARM_USE_SIMPLE_MODE = false;
    
    public static final double ARM_INTAKE_POWER = 0.6;
    public static final boolean ARM_INTAKE_MOTOR_INVERT_OUTPUT = true;
    public static final double ARM_INTAKE_THROUGHBEAM_THRESHOLD = 3.5;

    public static final boolean ARM_USE_MM = false;
    public static final boolean ARM_USE_IK = false;

    public static final double ARM_MAX_VELOCITY = 400.0;
    public static final double ARM_LOWER_LEFT_POSITION_MM_PID_KP = 0.0;
    public static final double ARM_LOWER_LEFT_POSITION_MM_PID_KI = 0.0;
    public static final double ARM_LOWER_LEFT_POSITION_MM_PID_KD = 0.0;
    public static final double ARM_LOWER_LEFT_POSITION_MM_PID_KF = 3.2;
    public static final double ARM_LOWER_LEFT_POSITION_MM_CRUISE_VELOCITY = 400.0;
    public static final double ARM_LOWER_LEFT_POSITION_MM_ACCELERATION = 200.0;

    public static final double ARM_LOWER_RIGHT_POSITION_MM_PID_KP = 0.0;
    public static final double ARM_LOWER_RIGHT_POSITION_MM_PID_KI = 0.0;
    public static final double ARM_LOWER_RIGHT_POSITION_MM_PID_KD = 0.0;
    public static final double ARM_LOWER_RIGHT_POSITION_MM_PID_KF = 3.2;
    public static final double ARM_LOWER_RIGHT_POSITION_MM_CRUISE_VELOCITY = 0.8 * 400.0;
    public static final double ARM_LOWER_RIGHT_POSITION_MM_ACCELERATION = 200.0;

    public static final double ARM_UPPER_POSITION_MM_PID_KP = 0.0;
    public static final double ARM_UPPER_POSITION_MM_PID_KI = 0.0;
    public static final double ARM_UPPER_POSITION_MM_PID_KD = 0.0;
    public static final double ARM_UPPER_POSITION_MM_PID_KF = 3.2;
    public static final double ARM_UPPER_POSITION_MM_CRUISE_VELOCITY = 0.8 * 400.0;
    public static final double ARM_UPPER_POSITION_MM_ACCELERATION = 200.0;

    public static final double ARM_LOWER_LEFT_POSITION_PID_KP = 1.2;
    public static final double ARM_LOWER_LEFT_POSITION_PID_KI = 0.0;
    public static final double ARM_LOWER_LEFT_POSITION_PID_KD = 0.0;
    public static final double ARM_LOWER_LEFT_POSITION_PID_KF = 0.0;

    public static final double ARM_LOWER_RIGHT_POSITION_PID_KP = 1.2;
    public static final double ARM_LOWER_RIGHT_POSITION_PID_KI = 0.0;
    public static final double ARM_LOWER_RIGHT_POSITION_PID_KD = 0.0;
    public static final double ARM_LOWER_RIGHT_POSITION_PID_KF = 0.0;

    public static final double ARM_UPPER_POSITION_PID_KP = 1.2;
    public static final double ARM_UPPER_POSITION_PID_KI = 0.0;
    public static final double ARM_UPPER_POSITION_PID_KD = 0.0;
    public static final double ARM_UPPER_POSITION_PID_KF = 0.0;

    public static final boolean ARM_LOWER_LEFT_INVERT_OUTPUT = true;
    public static final boolean ARM_LOWER_RIGHT_INVERT_OUTPUT = true;
    public static final boolean ARM_UPPER_MASTER_INVERT_OUTPUT = true;
    public static final boolean ARM_UPPER_FOLLOWER_INVERT_OUTPUT = true;

    public static final boolean ARM_LOWER_LEFT_INVERT_SENSOR = false;
    public static final boolean ARM_LOWER_RIGHT_INVERT_SENSOR = false;
    public static final boolean ARM_UPPER_INVERT_SENSOR = false;

    public static final double ARM_NEAR_FULL_EXTENSION_LENGTH = HardwareConstants.ARM_EXTENTION_LENGTH * 0.9; // in inches
    public static final double ARM_NEAR_FULL_RETRACTED_LENGTH = HardwareConstants.ARM_EXTENTION_LENGTH * 0.1; // in inches

    public static final double ARM_UPPER_MAX_EXTENSION_LENGTH = HardwareConstants.ARM_EXTENTION_LENGTH * 1.0; // in inches
    public static final double ARM_LOWER_MAX_EXTENSION_LENGTH = HardwareConstants.ARM_EXTENTION_LENGTH * 1.0; // in inches

    public static final double ARM_RETRACTION_MAX_TIME = 0.7;

    public static final double ARM_X_POSITION_ADJUSTMENT_VELOCITY = 2.0; // inches per second
    public static final double ARM_Z_POSITION_ADJUSTMENT_VELOCITY = 2.0; // inches per second

    public static final double ARM_LOWER_EXTENSION_ADJUSTMENT_VELOCITY = 2.0; // inches per second
    public static final double ARM_UPPER_EXTENSION_ADJUSTMENT_VELOCITY = 2.0; // inches per second

    public static final double ARM_MAX_REVERSE_SIMPLE_VELOCITY = -0.8; // percentage output
    public static final double ARM_MAX_FORWARD_SIMPLE_VELOCITY = 0.8; // percentage output

    public static final double ARM_MIN_IKX_EXTENSION_LENGTH = 0.0;
    public static final double ARM_MAX_IKX_EXTENSION_LENGTH = HardwareConstants.MAX_ROBOT_EXTENSION + HardwareConstants.ARM_ORIGIN_X_OFFSET - HardwareConstants.ARM_MAX_END_EFFECTOR_EXTENSION;
    public static final double ARM_MIN_IKZ_EXTENSION_HEIGHT = -5.0;
    public static final double ARM_MAX_IKZ_EXTENSION_HEIGHT = HardwareConstants.MAX_ROBOT_HEIGHT - HardwareConstants.ARM_ORIGIN_Z_OFFSET - HardwareConstants.ARM_MAX_END_EFFECTOR_HEIGHT;

    public static final double ARM_LOWER_VELOCITY_DEAZONE = 0.15;
    public static final double ARM_UPPER_VELOCITY_DEAZONE = 0.15;

    public static final double ARM_FLIPPER_EXTEND_WAIT_DURATION = 0.5;
    public static final double ARM_FLIPPER_RETRACT_WAIT_DURATION = 0.5;

    public static final double ARM_FULLY_RETRACTED_X_POSITION = 0.0; // in inches
    public static final double ARM_FULLY_RETRACTED_Z_POSITION = 0.0; // in inches

    // thresholds for auto/macro tasks for whether it has reached the desired position:
    public static final double ARM_LOWER_MM_GOAL_THRESHOLD = 0.2 * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH; // in ticks
    public static final double ARM_UPPER_MM_GOAL_THRESHOLD = 0.2 * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH; // in ticks
    public static final double ARM_X_IK_GOAL_THRESHOLD = 0.5; // in inches
    public static final double ARM_Z_IK_GOAL_THRESHOLD = 0.5; // in inches

    public static final double ARM_LOWER_MM_INTERMIDATE = HardwareConstants.ARM_EXTENTION_LENGTH * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH;
    public static final double ARM_UPPER_MM_INTERMIDATE = 1200.0;
    public static final double ARM_LOWER_MM_IN_TRESHOLD = 8000.0;
    public static final double ARM_UPPER_MM_IN_TRESHOLD = 1000.0;
    public static final double ARM_X_IK_IN_TRESHOLD = 18.0;
    public static final double ARM_Z_IK_IN_TRESHOLD = 8.0;
    public static final double ARM_X_IK_INTERMIDATE = 20.0;
    public static final double ARM_Z_IK_INTERMIDATE = 14.5;

    // Power sampling for arm
    public static final double ARM_POWER_TRACKING_DURATION = 0.25; // duration of time to keep track of the average current
    public static final double ARM_POWER_SAMPLES_PER_LOOP = 1.0; // we may want to increase this if we find our update loop duration isn't very consistent...
    public static final double ARM_POWER_SAMPLES_PER_SECOND = TuningConstants.LOOPS_PER_SECOND * TuningConstants.ARM_POWER_SAMPLES_PER_LOOP;
    public static final double ARM_NOT_MOVING_POWER_THRESHOLD = 0.25; // amount of power being "used" by the linear actuator to be considered "not moving"

    // Velocity sampling for arm
    public static final double ARM_VELOCITY_TRACKING_DURATION = TuningConstants.ARM_POWER_TRACKING_DURATION; // should match the power tracking
    public static final double ARM_VELOCITY_SAMPLES_PER_LOOP = TuningConstants.ARM_POWER_SAMPLES_PER_LOOP; // should match the power tracking
    public static final double ARM_VELOCITY_SAMPLES_PER_SECOND = TuningConstants.ARM_POWER_SAMPLES_PER_SECOND; // should match the power tracking

    // Arm stall prevention
    public static final boolean ARM_STALL_PROTECTION_ENABLED = true;
    public static final double ARM_STALLED_POWER_THRESHOLD = 3.0; // amount of power being "used" by the linear actuator (despite not moving according to the encoders) to be considered stalled
    public static final double ARM_STALLED_VELOCITY_THRESHOLD = 10.0; // 10 ticks/sec is very slow, unlikely to be really moving...

    // Set Points for Arm (linear actuator positions)
    public static final double ARM_LOWER_ZEROING_POSITION = 10.0 * HardwareConstants.ARM_FULL_EXTENSION_TICKS;
    public static final double ARM_UPPER_ZEROING_POSITION = -10.0 * HardwareConstants.ARM_FULL_EXTENSION_TICKS;
    public static final double ARM_LOWER_POSITION_STOWED = HardwareConstants.ARM_FULL_EXTENSION_TICKS;
    public static final double ARM_UPPER_POSITION_STOWED = 0.0;
    public static final double ARM_LOWER_POSITION_GROUND_PLACING = 4700.0;
    public static final double ARM_UPPER_POSITION_GROUND_PLACING = 1650.0;
    public static final double ARM_LOWER_POSITION_MIDDLE_CONE = 7550.0; 
    public static final double ARM_UPPER_POSITION_MIDDLE_CONE = 4750.0;
    public static final double ARM_LOWER_POSITION_MIDDLE_CUBE = 5775.0; 
    public static final double ARM_UPPER_POSITION_MIDDLE_CUBE = 3800.0;
    public static final double ARM_LOWER_POSITION_HIGH_CONE = 3210.0;
    public static final double ARM_UPPER_POSITION_HIGH_CONE = 7600.0;
    public static final double ARM_LOWER_POSITION_HIGH_CUBE = 4000.0;
    public static final double ARM_UPPER_POSITION_HIGH_CUBE = 6200.0;
    public static final double ARM_LOWER_POSITION_GROUND_PICKUP = 2000.0;
    public static final double ARM_UPPER_POSITION_GROUND_PICKUP = 1550.0;
    public static final double ARM_LOWER_POSITION_CONE_SUBSTATION_PICKUP = 3350.0;
    public static final double ARM_UPPER_POSITION_CONE_SUBSTATION_PICKUP = 6750.0;
    public static final double ARM_LOWER_POSITION_CUBE_SUBSTATION_PICKUP = 6750.0;
    public static final double ARM_UPPER_POSITION_CUBE_SUBSTATION_PICKUP = 3350.0;
    public static final double ARM_LOWER_POSITION_APPROACH = HardwareConstants.ARM_FULL_EXTENSION_TICKS;
    public static final double ARM_UPPER_POSITION_APPROACH = 4100.0;
    public static final double ARM_LOWER_POSITION_CONE_UPRIGHTING_MACRO = TuningConstants.ARM_LOWER_POSITION_GROUND_PLACING;
    public static final double ARM_UPPER_POSITION_CONE_UPRIGHTING_MACRO = TuningConstants.ARM_UPPER_POSITION_GROUND_PLACING;

    //============================================= ChargeStation Macro ==================================================================

    // ChargeStationTask constants (owned by Jamie and Calvin)
    public static final double CHARGE_STATION_PITCH_VARIATION = 0.3;
    public static final double CHARGE_STATION_ACCEPTABLE_PITCH_DIFF = 2.0;
    public static final double CHARGE_STATION_STARTING_SPEED = 0.5;
    public static final double CHARGE_STATION_CLIMBING_SPEED = 0.2;
    public static final double CHARGE_STATION_BALANCING_SPEED = 0.06;
    public static final double CHARGE_STATION_START_TRANSITION_PITCH = 5.0;
    public static final double CHARGE_STATION_CLIMBING_TRANSITION_PITCH = 13;
    public static final double CHARGE_STATION_CLIMBING_TRANSITION_ACCEPTABLE_VARIATION = 0.3;
    public static final double CHARGE_STATION_CLIMBING_TRANSITION_WAIT_DURATION = 1.5;

    //Version 2 constants
    public static final double CHARGE_STATION_PITCH_VARIATION_V2 = 1.0;
    public static final double CHARGE_STATION_ACCEPTABLE_PITCH_DIFF_V2 = 3.0;
    public static final double CHARGE_STATION_STARTING_SPEED_V2 = 0.5;
    public static final double CHARGE_STATION_CLIMBING_SPEED_V2 = 0.225;
    public static final double CHARGE_STATION_BALANCING_SPEED_V2 = 0.08;

    //ChargeStationTaskGyro (owned by Calvin)
    public static final double CHARGE_STATION_2_PITCH_VARIATION = 0.3;
    public static final double CHARGE_STATION_2_STARTING_SPEED = 0.5;
    public static final double CHARGE_STATION_2_START_TRANSITION_PITCH = 5.0;
    public static final double CHARGE_STATION_2_CLIMBING_TRANSITION_PITCH = 13;
    public static final double CHARGE_STATION_2_CLIMBING_TRANSITION_ACCEPTABLE_VARIATION = 0.3;
    public static final double CHARGE_STATION_2_CLIMBING_TRANSITION_WAIT_DURATION = 1.35;

    public static final double CHARGE_STATION_2_ACCEPTABLE_PITCH_DIFF = 8.0;
    public static final double CHARGE_STATION_2_TRANSITION_PITCH_DIFF = 40.0;
    public static final double CHARGE_STATION_2_APPROACH_SPEED = 0.5;
    public static final double CHARGE_STATION_2_MOUNT_SPEED = 0.25;
    public static final double CHARGE_STATION_2_CLIMBING_SPEED = 0.2;
    public static final double CHARGE_STATION_2_BALANCING_SPEED = 0.06;
    public static final double CHARGE_STATION_2_FAST_BALANCING_SPEED = 0.12;
    public static final double CHARGE_STATION_2_MOUNTING_TRANSITION_PITCH = 5.0;
    public static final double CHARGE_STATION_2_CLIMBING_TRANSITION_GYRO = 15;
    public static final double CHARGE_STATION_2_BRAKE_GYRO = 50.0;
    public static final double CHARGE_STATION_2_MIN_BRAKE_TIME = 0.5;
    public static final double CHARGE_STATION_2_COMPLETED_GYRO = 1.0;

    //============================================= Retro Reflective Tape Macro ==================================================================
    public static final double RR_CENTERING_ACCEPTABLE_RIGHT_ANGLE_ERROR = 5.0; 
    public static final double RR_CENTERING_ACCEPTABLE_LEFT_ANGLE_ERROR = -5.0; 
    public static final double RR_CENTERING_MOVEMENT_SPEED_TO_CENTER_WITH_REFLECTIVE_TAPE = 0.05;
}
