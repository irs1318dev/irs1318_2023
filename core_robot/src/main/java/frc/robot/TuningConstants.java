package frc.robot;

/**
 * All constants related to tuning the operation of the robot.
 * 
 * @author Will
 * 
 */
public class TuningConstants
{
    public static final boolean COMPETITION_ROBOT = true;
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

    // Y Values (distance from Guardrail edge)
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
    public static final double GroundOneY = 180.19;
    public static final double GroundTwoY = 132.19;
    public static final double GroundThreeY = 84.19;
    public static final double GroundFourY = 36.19;
    public static final double LoadEdgeY = TuningConstants.StartOneGridY + 1.521; // Edge of grid - 17.5
    public static final double GuardEdgeY = 17.5;
    public static final double FullWidth = 315.5; // (Y) 26 ft. 3.5 in, from game manual
    public static final double TurnGuardY = 27.5;
    public static final double TurnLoadY = 186.53;
    // X Values
    public static final double StartGridX = 17.5;//71.765; // Edge of grid - Robot centering value
    public static final double CloseChargeStationX = 30.345;//84.61; // 12.845 inches away from the charge station and grid + Robot centering value
    public static final double FarChargeStationX = 166.875;//222.635; // 15 inches away from the charge station + Robot centering value
    public static final double FarChargeStationInBetweenX = TuningConstants.FarChargeStationX + 30.0; // 30 inches away from last point to allow for turning
    public static final double GroundPiecesX = 220.5;//253.265; // On ground pieces real value 47.36
    public static final double LoadEdgeStartX = 58.5;//110.765;
    public static final double GuardEdgeStartX = 77.75;//132.015;
    public static final double FullLength = 651.25; // (X) 54 ft. 3.25 in, from game manual
    public static final double BetweenBumpAndChargeStationFarX = 121.85; // 6 + Bump + 17.5

    // April tag array by ids - Blue alliance
    // (xPosition, yPosition, orientation)
    public static final double[][] AprilTagLocationsBlue =
        {
            { 610.77,  42.19,   0.0 }, // ID 1
            { 610.77, 108.19,   0.0 }, // ID 2
            { 610.77, 174.19,   0.0 }, // ID 3
            { 636.96, 265.74,   0.0 }, // ID 4
            {  14.25,  42.19, 180.0 }, // ID 5
            {  40.45, 108.19, 180.0 }, // ID 6
            {  40.45, 174.19, 180.0 }, // ID 7
            {  40.45, 265.74, 180.0 }, // ID 8
        };

    // April tag array by ids - Red alliance
    // (xPosition, yPosition, orientation)
    public static final double[][] AprilTagLocationsRed =
        {
            {  40.45, 273.31, 180.0 }, // ID 1
            {  40.45, 207.31, 180.0 }, // ID 2
            {  40.45, 141.31, 180.0 }, // ID 3
            {  14.25,  49.36, 180.0 }, // ID 4
            { 636.96, 273.31,   0.0 }, // ID 5
            { 610.77, 207.31,   0.0 }, // ID 6
            { 610.77, 141.31,   0.0 }, // ID 7
            { 610.77,  49.36,   0.0 }, // ID 8
        };
    //================================================= Power ======================================================

    public static final double POWER_OVERCURRENT_TRACKING_DURATION = 5.0; // duration of time to keep track of the average current
    public static final double POWER_OVERCURRENT_SAMPLES_PER_LOOP = 1.0; // we may want to increase this if we find our update loop duration isn't very consistent...
    public static final double POWER_OVERCURRENT_SAMPLES_PER_SECOND = TuningConstants.LOOPS_PER_SECOND * TuningConstants.POWER_OVERCURRENT_SAMPLES_PER_LOOP;
    public static final double POWER_OVERCURRENT_THRESHOLD = 140.0;
    public static final double POWER_OVERCURREHT_HIGH_THRESHOLD = 180.0;

    //================================================= Vision ======================================================

    // Finding AprilTags to determine if theres enough valid data to translate 
    public static final int TAGS_MISSED_THRESHOLD = 30;
    public static final int TAGS_FOUND_THRESHOLD = 5;
    public static final double ACCEPTABLE_RANGE_IN_X_AND_Y_FOR_ALIGNMENT_TRANSLATE = 1.0; // in inches
    public static final double APRILTAG_TO_CONE_NODE_HORIZONTAL_DISTANCE = 22.0; // in inches
    public static final double APRILTAG_TO_DESIRED_SCORING_X_POSITION_DISTANCE = 40.0; // from apriltag location to center of robot, in inches

    // Acceptable vision centering range values in degrees
    public static final double MAX_PID_TURNING_RANGE_DEGREES = 7.0;

    // How long the robot system must remain centered on the target when using time
    public static final double PID_TURNING_DURATION = 0.75;

    // Acceptable vision distance from tape in inches (as measured by vision system)
    public static final double MAX_VISION_ACCEPTABLE_FORWARD_DISTANCE = 1.75;
    public static final double MAX_VISION_ACCEPTABLE_STRAFE_DISTANCE = 0.7;

    // Acceptable vision distance from tape in angles 
    public static final double MAX_VISION_ACCEPTABLE_MOVING_RR_ANGLE_ERROR = 4.0;

    // PID settings for Centering the robot on a vision target from one stationary place
    public static final double STATIONARY_PID_TURNING_PID_KP = 0.027;
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

    // PID settings for translating the robot based on a vision target
    public static final double VISION_AT_TRANSLATION_X_PID_KP = 0.02;
    public static final double VISION_AT_TRANSLATION_X_PID_KI = 0.0;
    public static final double VISION_AT_TRANSLATION_X_PID_KD = 0.0;
    public static final double VISION_AT_TRANSLATION_X_PID_KF = 0.0;
    public static final double VISION_AT_TRANSLATION_X_PID_KS = 1.0;
    public static final double VISION_AT_TRANSLATION_X_PID_MIN = -0.3;
    public static final double VISION_AT_TRANSLATION_X_PID_MAX = 0.3;

    // PID settings for translating the robot based on a vision target
    public static final double VISION_AT_TRANSLATION_Y_PID_KP = 0.02;
    public static final double VISION_AT_TRANSLATION_Y_PID_KI = 0.0;
    public static final double VISION_AT_TRANSLATION_Y_PID_KD = 0.0;
    public static final double VISION_AT_TRANSLATION_Y_PID_KF = 0.0;
    public static final double VISION_AT_TRANSLATION_Y_PID_KS = 1.0;
    public static final double VISION_AT_TRANSLATION_Y_PID_MIN = -0.3;
    public static final double VISION_AT_TRANSLATION_Y_PID_MAX = 0.3;

    // PID settings for translating the robot slowly based on a vision target
    public static final double VISION_SLOW_MOVING_PID_KP = 0.013;
    public static final double VISION_SLOW_MOVING_PID_KI = 0.0;
    public static final double VISION_SLOW_MOVING_PID_KD = 0.0;
    public static final double VISION_SLOW_MOVING_PID_KF = 0.0;
    public static final double VISION_SLOW_MOVING_PID_KS = 1.0;
    public static final double VISION_SLOW_MOVING_PID_MIN = -0.3;
    public static final double VISION_SLOW_MOVING_PID_MAX = 0.3;

    // PID settings for translating the robot quickly based on a vision target
    public static final double VISION_FAST_MOVING_PID_KP = 0.17;
    public static final double VISION_FAST_MOVING_PID_KI = 0.0;
    public static final double VISION_FAST_MOVING_PID_KD = 0.0;
    public static final double VISION_FAST_MOVING_PID_KF = 0.0;
    public static final double VISION_FAST_MOVING_PID_KS = 1.0;
    public static final double VISION_FAST_MOVING_PID_MIN = -0.45;
    public static final double VISION_FAST_MOVING_PID_MAX = 0.45;

    public static final int VISION_MISSED_HEARTBEAT_THRESHOLD = 500;

    //================================================== Indicator Lights ========================================================

    public static final double INDICATOR_LIGHT_VISION_ACCEPTABLE_ANGLE_RANGE = 3.0;

    public static final int CANDLE_LED_START = 0;
    public static final int CANDLE_LED_COUNT = 8;
    public static final int LED_STRIP_LED_START = TuningConstants.CANDLE_LED_COUNT;
    public static final int LED_STRIP_LED_COUNT = 60; // 60 LEDs per meter-long strip from CTRE
    public static final int CANDLE_TOTAL_NUMBER_LEDS = TuningConstants.CANDLE_LED_COUNT + TuningConstants.LED_STRIP_LED_COUNT;

    public static final int CANDLE_ANIMATION_SLOT_0 = 0;
    public static final int CANDLE_ANIMATION_SLOT_1 = 1;
    public static final int CANDLE_ANIMATION_SLOT_2 = 2;
    public static final int CANDLE_ANIMATION_SLOT_3 = 3;

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
    public static final int INDICATOR_BLUE_COLOR_WHITE = 0;

    //Orange - No Game Piece in Intake
    public static final int INDICATOR_ORANGE_COLOR_RED = 255;
    public static final int INDICATOR_ORANGE_COLOR_GREEN = 165;
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

    public static final double DRIVETRAIN_STEER_MOTOR1_ABSOLUTE_OFFSET = 128.056;
    public static final double DRIVETRAIN_STEER_MOTOR2_ABSOLUTE_OFFSET = -16.259;
    public static final double DRIVETRAIN_STEER_MOTOR3_ABSOLUTE_OFFSET = 2.725;
    public static final double DRIVETRAIN_STEER_MOTOR4_ABSOLUTE_OFFSET = -30.586;

    public static final boolean DRIVETRAIN_USE_TRANSLATIONAL_RATE_LIMITING = true;
    public static final double DRIVETRAIN_TRANSLATIONAL_VELOCITY_MAX_NEGATIVE_RATE = -3.0 * TuningConstants.DRIVETRAIN_MAX_VELOCITY;
    public static final double DRIVETRAIN_TRANSLATIONAL_VELOCITY_MAX_POSITIVE_RATE = 3.0 * TuningConstants.DRIVETRAIN_MAX_VELOCITY;
    public static final boolean DRIVETRAIN_USE_ROTATIONAL_RATE_LIMITING = true;
    public static final double DRIVETRAIN_ROTATIONAL_VELOCITY_MAX_NEGATIVE_RATE = -4.0 * TuningConstants.DRIVETRAIN_TURN_SCALE;
    public static final double DRIVETRAIN_ROTATIONAL_VELOCITY_MAX_POSITIVE_RATE = 4.0 * TuningConstants.DRIVETRAIN_TURN_SCALE;

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

    public static final double DRIVETRAIN_PATH_X_POSITION_PID_KP = 1.0;
    public static final double DRIVETRAIN_PATH_X_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_PATH_X_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_PATH_X_POSITION_PID_KF = 0.0;
    public static final double DRIVETRAIN_PATH_X_POSITION_PID_KS = 1.0;
    public static final double DRIVETRAIN_PATH_X_MAX_OUTPUT = 10.0;
    public static final double DRIVETRAIN_PATH_X_MIN_OUTPUT = -10.0;

    public static final double DRIVETRAIN_PATH_Y_POSITION_PID_KP = 1.0;
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
    public static final double DRIVETRAIN_SKIP_OMEGA_ON_ZERO_DELTA = 0.36;

    public static final double DRIVETRAIN_EXPONENTIAL = 2.0;
    public static final double DRIVETRAIN_DEAD_ZONE_TURN = 0.1;
    public static final double DRIVETRAIN_DEAD_ZONE_VELOCITY_X = 0.1;
    public static final double DRIVETRAIN_DEAD_ZONE_VELOCITY_Y = 0.1;
    public static final double DRIVETRAIN_DEAD_ZONE_TRIGGER_AB = 0.1;

    public static final double DRIVETRAIN_ROTATION_A_MULTIPLIER = HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE / 2.0;
    public static final double DRIVETRAIN_ROTATION_B_MULTIPLIER = HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE / 2.0;

    public static final double DRIVETRAIN_MAX_VELOCITY = TuningConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS * HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND; // max velocity in inches per second
    public static final double DRIVETRAIN_SLOW_MODE_MAX_VELOCITY = 0.3 * TuningConstants.DRIVETRAIN_MAX_VELOCITY; // max velocity in inches per second
    public static final double DRIVETRAIN_VELOCITY_TO_PERCENTAGE = 1.0 / TuningConstants.DRIVETRAIN_MAX_VELOCITY;
    public static final double DRIVETRAIN_TURN_GOAL_VELOCITY = 10.0; // degrees per second for turn goal
    public static final double DRIVETRAIN_TURN_SCALE = 1.6 * Math.PI; // radians per second
    public static final double DRIVETRAIN_SLOW_MODE_TURN_SCALE = 0.3 * TuningConstants.DRIVETRAIN_TURN_SCALE; // radians per second
    public static final double DRIVETRAIN_STATIONARY_VELOCITY = 0.1;
    public static final double DRIVETRAIN_TURN_APPROXIMATION_STATIONARY = 2.0; // number of degrees off at which point we give up trying to face an angle when uncommanded
    public static final double DRIVETRAIN_TURN_APPROXIMATION = 1.0; // number of degrees off at which point we give up trying to face an angle when uncommanded
    public static final double DRIVETRAIN_MAX_MODULE_PATH_VELOCITY = 0.85 * TuningConstants.DRIVETRAIN_MAX_VELOCITY; // up to x% of our max controllable speed
    public static final double DRIVETRAIN_MAX_PATH_TURN_VELOCITY = 180.0; // in degrees per second
    public static final double DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY = 0.60 * TuningConstants.DRIVETRAIN_MAX_VELOCITY; // in inches per second    
    public static final double DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION = 0.75 * TuningConstants.DRIVETRAIN_MAX_VELOCITY; // in inches per second per second
    public static final double DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY = TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY / 1.4; // in inches per second
    public static final double DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION = TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION / 1.25; // in inches per second per second
    public static final double DRIVETRAIN_LOW_PATH_TRANSLATIONAL_VELOCITY = TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY / 2.0; // in inches per second
    public static final double DRIVETRAIN_LOW_PATH_TRANSLATIONAL_ACCELERATION = TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION / 2.0; // in inches per second per second

    //================================================= Arm ================================================================================

    public static final boolean ARM_USE_SIMPLE_MODE = false;

    public static final double ARM_INTAKE_CUBE_POWER = 0.8; // Also, eject cone
    public static final double ARM_INTAKE_CONE_POWER = -0.6; // Also, eject cube
    public static final boolean ARM_INTAKE_MOTOR_INVERT_OUTPUT = true;

    public static final boolean ARM_USE_MM = false;
    public static final boolean ARM_USE_IK = true;

    public static final double ARM_MAX_VELOCITY = 400.0;
    public static final double ARM_LOWER_LEFT_POSITION_MM_PID_KP = 0.5;
    public static final double ARM_LOWER_LEFT_POSITION_MM_PID_KI = 0.0;
    public static final double ARM_LOWER_LEFT_POSITION_MM_PID_KD = 0.0;
    public static final double ARM_LOWER_LEFT_POSITION_MM_PID_KF = 3.2;
    public static final double ARM_LOWER_LEFT_POSITION_MM_CRUISE_VELOCITY = 400.0;
    public static final double ARM_LOWER_LEFT_POSITION_MM_ACCELERATION = 400.0;

    public static final double ARM_LOWER_RIGHT_POSITION_MM_PID_KP = 0.5;
    public static final double ARM_LOWER_RIGHT_POSITION_MM_PID_KI = 0.0;
    public static final double ARM_LOWER_RIGHT_POSITION_MM_PID_KD = 0.0;
    public static final double ARM_LOWER_RIGHT_POSITION_MM_PID_KF = 3.2;
    public static final double ARM_LOWER_RIGHT_POSITION_MM_CRUISE_VELOCITY = 0.8 * 400.0;
    public static final double ARM_LOWER_RIGHT_POSITION_MM_ACCELERATION = 400.0;

    public static final double ARM_UPPER_POSITION_MM_PID_KP = 0.5;
    public static final double ARM_UPPER_POSITION_MM_PID_KI = 0.0;
    public static final double ARM_UPPER_POSITION_MM_PID_KD = 0.0;
    public static final double ARM_UPPER_POSITION_MM_PID_KF = 3.2;
    public static final double ARM_UPPER_POSITION_MM_CRUISE_VELOCITY = 0.8 * 400.0;
    public static final double ARM_UPPER_POSITION_MM_ACCELERATION = 400.0;

    public static final double ARM_LOWER_LEFT_POSITION_PID_KP = 1.9;//1.8;
    public static final double ARM_LOWER_LEFT_POSITION_PID_KI = 0.0;
    public static final double ARM_LOWER_LEFT_POSITION_PID_KD = 0.0;
    public static final double ARM_LOWER_LEFT_POSITION_PID_KF = 0.0;

    public static final double ARM_LOWER_RIGHT_POSITION_PID_KP = 1.9;//1.8;
    public static final double ARM_LOWER_RIGHT_POSITION_PID_KI = 0.0;
    public static final double ARM_LOWER_RIGHT_POSITION_PID_KD = 0.0;
    public static final double ARM_LOWER_RIGHT_POSITION_PID_KF = 0.0;

    public static final double ARM_UPPER_POSITION_PID_KP = 1.3;//1.2;
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

    public static final double ARM_MAX_TWIST_AMOUNT = 800.0; // in ticks - the largest allowed tick difference between lower left and lower right actuators

    public static final double ARM_NEAR_FULL_EXTENSION_LENGTH = HardwareConstants.ARM_EXTENTION_LENGTH * 0.9; // in inches
    public static final double ARM_NEAR_FULL_RETRACTED_LENGTH = HardwareConstants.ARM_EXTENTION_LENGTH * 0.1; // in inches

    public static final double ARM_UPPER_MAX_EXTENSION_LENGTH = HardwareConstants.ARM_EXTENTION_LENGTH * 1.0; // in inches
    public static final double ARM_LOWER_MAX_EXTENSION_LENGTH = HardwareConstants.ARM_EXTENTION_LENGTH * 1.0; // in inches

    public static final double ARM_RETRACTION_MAX_TIME = 0.7;

    public static final double ARM_X_POSITION_ADJUSTMENT_VELOCITY = 4.0; // inches per second
    public static final double ARM_Z_POSITION_ADJUSTMENT_VELOCITY = 4.0; // inches per second

    public static final double ARM_MAX_REVERSE_SIMPLE_VELOCITY = -0.8; // percentage output
    public static final double ARM_MAX_FORWARD_SIMPLE_VELOCITY = 0.8; // percentage output

    public static final double ARM_MIN_IKX_EXTENSION_LENGTH = 0.0;
    public static final double ARM_MAX_IKX_EXTENSION_LENGTH = HardwareConstants.MAX_ROBOT_EXTENSION + HardwareConstants.ARM_ORIGIN_X_OFFSET - HardwareConstants.ARM_MAX_END_EFFECTOR_EXTENSION;
    public static final double ARM_MIN_IKZ_EXTENSION_HEIGHT = -5.0;
    public static final double ARM_MAX_IKZ_EXTENSION_HEIGHT = HardwareConstants.MAX_ROBOT_HEIGHT - HardwareConstants.ARM_ORIGIN_Z_OFFSET - HardwareConstants.ARM_MAX_END_EFFECTOR_HEIGHT;

    public static final double ARM_LOWER_VELOCITY_DEAZONE = 0.15;
    public static final double ARM_UPPER_VELOCITY_DEAZONE = 0.15;
    public static final double ARM_TWIST_DEAZONE = 0.10;

    public static final double ARM_FULLY_RETRACTED_X_POSITION = 0.0; // in inches
    public static final double ARM_FULLY_RETRACTED_Z_POSITION = 0.0; // in inches

    // thresholds for auto/macro tasks for whether it has reached the desired position:
    public static final double ARM_LOWER_MM_GOAL_THRESHOLD = 0.2 * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH; // in ticks
    public static final double ARM_UPPER_MM_GOAL_THRESHOLD = 0.2 * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH; // in ticks
    public static final double ARM_X_IK_GOAL_THRESHOLD = 0.5; // in inches
    public static final double ARM_Z_IK_GOAL_THRESHOLD = 0.5; // in inches

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
    public static final double ARM_STALLED_CURRENT_THRESHOLD = 3.5; // amount of current being "used" by the linear actuator (despite not moving according to the encoders) to be considered stalled
    public static final double BATTERY_AVERAGE_EXPECTED_VOLTAGE = 12.0; // expected voltage of battery
    public static final double ARM_STALLED_POWER_THRESHOLD = TuningConstants.ARM_STALLED_CURRENT_THRESHOLD * TuningConstants.BATTERY_AVERAGE_EXPECTED_VOLTAGE; // amount of power that can be "used" by the linear actuators to be considered stalled
    public static final double ARM_STALLED_VELOCITY_THRESHOLD = 8.0; // 8 ticks/sec is very slow, unlikely to be really moving...

    // Set Points for Arm (linear actuator positions)
    public static final double ARM_LOWER_ZEROING_POSITION = 10.0 * HardwareConstants.ARM_FULL_EXTENSION_TICKS;
    public static final double ARM_UPPER_ZEROING_POSITION = -10.0 * HardwareConstants.ARM_FULL_EXTENSION_TICKS;
    public static final double ARM_LOWER_POSITION_STOWED = HardwareConstants.ARM_FULL_EXTENSION_TICKS;
    public static final double ARM_UPPER_POSITION_STOWED = 0.0;
    public static final double ARM_LOWER_POSITION_MIDDLE_CONE = 8240.0;
    public static final double ARM_UPPER_POSITION_MIDDLE_CONE = 3260.0;
    public static final double ARM_LOWER_POSITION_MIDDLE_CUBE = 5780.0;
    public static final double ARM_UPPER_POSITION_MIDDLE_CUBE = 4320.0;
    public static final double ARM_LOWER_POSITION_HIGH_CONE_UP = 4650.0;
    public static final double ARM_UPPER_POSITION_HIGH_CONE_UP = 6100.0;
    public static final double ARM_LOWER_POSITION_HIGH_CONE_DOWN = 5560.0;
    public static final double ARM_UPPER_POSITION_HIGH_CONE_DOWN = 6140.0;
    public static final double ARM_LOWER_POSITION_HIGH_CUBE = 3680.0;
    public static final double ARM_UPPER_POSITION_HIGH_CUBE = 7250.0;
    public static final double ARM_LOWER_POSITION_CONE_GROUND_PICKUP = 2350.0;
    public static final double ARM_UPPER_POSITION_CONE_GROUND_PICKUP = 1800.0;
    public static final double ARM_LOWER_POSITION_CUBE_GROUND_PICKUP = 3015.0;
    public static final double ARM_UPPER_POSITION_CUBE_GROUND_PICKUP = 1525.0;
    public static final double ARM_LOWER_POSITION_CONE_SUBSTATION_PICKUP_APPROACH = 6140.0;
    public static final double ARM_UPPER_POSITION_CONE_SUBSTATION_PICKUP_APPROACH = 6050.0;
    public static final double ARM_LOWER_POSITION_CONE_SUBSTATION_PICKUP = 6100.0;
    public static final double ARM_UPPER_POSITION_CONE_SUBSTATION_PICKUP = 6000.0;
    public static final double ARM_LOWER_POSITION_CUBE_SUBSTATION_PICKUP = 8300.0;
    public static final double ARM_UPPER_POSITION_CUBE_SUBSTATION_PICKUP = 4020.0;
    public static final double ARM_LOWER_POSITION_APPROACH = HardwareConstants.ARM_FULL_EXTENSION_TICKS;
    public static final double ARM_UPPER_POSITION_APPROACH = 4100.0;
    public static final double ARM_LOWER_POSITION_CONE_UPRIGHTING_MACRO = 4400;
    public static final double ARM_UPPER_POSITION_CONE_UPRIGHTING_MACRO = 2050;

    public static final double ARM_IKX_POSITION_STOWED = 2.7725594688249453;
    public static final double ARM_IKZ_POSITION_STOWED = 9.226618272316927;
    public static final double ARM_IKX_POSITION_GROUND_PLACING = 23.409486778127867;
    public static final double ARM_IKZ_POSITION_GROUND_PLACING = 8.108920118845923;
    public static final double ARM_IKX_POSITION_MIDDLE_CONE = 37.26065704303803;
    public static final double ARM_IKZ_POSITION_MIDDLE_CONE = 34.72708676253013;
    public static final double ARM_IKX_POSITION_MIDDLE_CUBE = 37.82742755893932;
    public static final double ARM_IKZ_POSITION_MIDDLE_CUBE = 21.045492744242054;
    public static final double ARM_IKX_POSITION_HIGH_CONE = 55.810758222247195;
    public static final double ARM_IKZ_POSITION_HIGH_CONE = 48.02010779021726;
    public static final double ARM_IKX_POSITION_HIGH_CUBE = 52.39322170289499;
    public static final double ARM_IKZ_POSITION_HIGH_CUBE = 33.932579125363674;
    public static final double ARM_IKX_POSITION_GROUND_PICKUP = 23.706159602313825;
    public static final double ARM_IKZ_POSITION_GROUND_PICKUP = 2.579351786849614;
    public static final double ARM_IKX_POSITION_CONE_SUBSTATION_PICKUP = 55.25973368669209;
    public static final double ARM_IKZ_POSITION_CONE_SUBSTATION_PICKUP = 35.34859476648403;
    public static final double ARM_IKX_POSITION_CUBE_SUBSTATION_PICKUP = 33.3687854421636;
    public static final double ARM_IKZ_POSITION_CUBE_SUBSTATION_PICKUP = 21.298834191380973;
    public static final double ARM_IKX_POSITION_APPROACH = 32.6997990497486;
    public static final double ARM_IKZ_POSITION_APPROACH = 31.95776639264709;
    public static final double ARM_IKX_POSITION_CONE_UPRIGHTING_MACRO = 23.409486778127867;
    public static final double ARM_IKZ_POSITION_CONE_UPRIGHTING_MACRO = 8.108920118845923;

    public static final double ARM_LOWER_POSITION_LOWER_INTERMIDATE = HardwareConstants.ARM_FULL_EXTENSION_TICKS;
    public static final double ARM_UPPER_POSITION_LOWER_INTERMIDATE = 1200.0;
    public static final double ARM_LOWER_POSITION_HIGH_INTERMIDATE = TuningConstants.ARM_LOWER_POSITION_APPROACH;
    public static final double ARM_UPPER_POSITION_HIGH_INTERMIDATE = TuningConstants.ARM_UPPER_POSITION_APPROACH;
    public static final double ARM_LOWER_POSITION_INSIDE_TRESHOLD = 8000.0;
    public static final double ARM_UPPER_POSITION_INSIDE_TRESHOLD = 1000.0;
    public static final double ARM_LOWER_POSITION_HIGH_TRESHOLD = 8000.0;
    public static final double ARM_UPPER_POSITION_HIGH_TRESHOLD = 4000.0;
    public static final double ARM_USE_UPPER_POSITION_HIGH_INTERMIDATE_THRESHOLD = 6250.0;
    public static final double ARM_X_IK_LOWER_INTERMIDATE = 20.0;
    public static final double ARM_Z_IK_LOWER_INTERMIDATE = 14.5;
    public static final double ARM_X_IK_HIGH_INTERMIDATE = TuningConstants.ARM_IKX_POSITION_APPROACH;
    public static final double ARM_Z_IK_HIGH_INTERMIDATE = TuningConstants.ARM_IKZ_POSITION_APPROACH;
    public static final double ARM_X_IK_INSIDE_TRESHOLD = 18.0;
    public static final double ARM_Z_IK_INSIDE_TRESHOLD = 12.0;
    public static final double ARM_X_IK_HIGH_TRESHOLD = 33.5;
    public static final double ARM_Z_IK_HIGH_TRESHOLD = 30.0;

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

    //TODO: edit to be more sensitive
    public static final double CHARGE_STATION_ACCEPTABLE_PITCH_DIFF_V2 = 3.0;
    public static final double CHARGE_STATION_STARTING_SPEED_V2 = 0.85;
    public static final double CHARGE_STATION_CLIMBING_SPEED_V2 = 0.25;
    public static final double CHARGE_STATION_BALANCING_SPEED_V2 = 0.08;

    //TODO: edit to be shorter, because climbing velocity has increased.
    public static final double CHARGE_STATION_CLIMBING_TRANSITION_WAIT_DURATION_V2 = 1.25;
}
