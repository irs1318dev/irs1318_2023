package frc.robot;

import frc.robot.common.Helpers;
import frc.robot.common.robotprovider.TalonFXInvertType;

/**
 * All constants describing the physical structure of the robot (distances and sizes of things).
 * 
 * @author Will
 * 
 */
public class HardwareConstants
{
    public static final double MAX_ROBOT_HEIGHT = 78.0; // inches, max overall height
    public static final double MAX_ROBOT_EXTENSION = 48.0; // inches, max extension beyond frame perimeter

    //================================================== DriveTrain ==============================================================

    public static final TalonFXInvertType DRIVETRAIN_STEER_MOTOR1_INVERT = TalonFXInvertType.Clockwise;
    public static final TalonFXInvertType DRIVETRAIN_STEER_MOTOR2_INVERT = TalonFXInvertType.Clockwise;
    public static final TalonFXInvertType DRIVETRAIN_STEER_MOTOR3_INVERT = TalonFXInvertType.Clockwise;
    public static final TalonFXInvertType DRIVETRAIN_STEER_MOTOR4_INVERT = TalonFXInvertType.Clockwise;

    public static final TalonFXInvertType DRIVETRAIN_DRIVE_MOTOR1_INVERT = TalonFXInvertType.CounterClockwise;
    public static final TalonFXInvertType DRIVETRAIN_DRIVE_MOTOR2_INVERT = TalonFXInvertType.CounterClockwise;
    public static final TalonFXInvertType DRIVETRAIN_DRIVE_MOTOR3_INVERT = TalonFXInvertType.CounterClockwise;
    public static final TalonFXInvertType DRIVETRAIN_DRIVE_MOTOR4_INVERT = TalonFXInvertType.CounterClockwise;

    public static final double DRIVETRAIN_STEER_TICKS_PER_REVOLUTION = 2048.0;
    public static final double DRIVETRAIN_STEER_GEAR_RATIO = 150.0 / 7.0; // According to SDS Mk4i code: (50.0 / 14.0) * (60.0 / 10.0) == ~21.43 : 1
    public static final double DRIVETRAIN_STEER_DEGREES = 360.0;
    public static final double DRIVETRAIN_STEER_TICK_DISTANCE = HardwareConstants.DRIVETRAIN_STEER_DEGREES / (HardwareConstants.DRIVETRAIN_STEER_GEAR_RATIO * HardwareConstants.DRIVETRAIN_STEER_TICKS_PER_REVOLUTION); // in degrees
    public static final double DRIVETRAIN_STEER_TICKS_PER_DEGREE = (HardwareConstants.DRIVETRAIN_STEER_GEAR_RATIO * HardwareConstants.DRIVETRAIN_STEER_TICKS_PER_REVOLUTION) / HardwareConstants.DRIVETRAIN_STEER_DEGREES; // in ticks

    public static final double DRIVETRAIN_DRIVE_TICKS_PER_REVOLUTION = 2048.0;
    public static final double DRIVETRAIN_DRIVE_GEAR_RATIO = 36000.0 / 5880; // According to SDS Mk4i Very Fast code: (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0) == ~6.12 : 1
    public static final double DRIVETRAIN_DRIVE_WHEEL_DIAMETER = 3.75; // SDS Mk4i code claims their 4-inch wheels are actually 3.95 inches now (in inches) We think its 3.95
    public static final double DRIVETRAIN_DRIVE_WHEEL_CIRCUMFERENCE = Math.PI * HardwareConstants.DRIVETRAIN_DRIVE_WHEEL_DIAMETER;
    public static final double DRIVETRAIN_DRIVE_TICK_DISTANCE = HardwareConstants.DRIVETRAIN_DRIVE_WHEEL_CIRCUMFERENCE / (HardwareConstants.DRIVETRAIN_DRIVE_GEAR_RATIO * HardwareConstants.DRIVETRAIN_DRIVE_TICKS_PER_REVOLUTION);
    public static final double DRIVETRAIN_DRIVE_TICKS_PER_INCH = (HardwareConstants.DRIVETRAIN_DRIVE_GEAR_RATIO * HardwareConstants.DRIVETRAIN_DRIVE_TICKS_PER_REVOLUTION) / HardwareConstants.DRIVETRAIN_DRIVE_WHEEL_CIRCUMFERENCE;
    public static final double DRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND = 10.0 * HardwareConstants.DRIVETRAIN_DRIVE_TICK_DISTANCE; // converts #ticks per 100ms into inches per second.
    public static final double DRIVETRAIN_DRIVE_INCHES_PER_SECOND_TO_MOTOR_VELOCITY = 0.1 * HardwareConstants.DRIVETRAIN_DRIVE_TICKS_PER_INCH; // converts inches per second into #ticks per 100ms.

    public static final double DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE = 22.75; // (in inches) 35" side-to-side with bumpers
    public static final double DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE = 22.75; // (in inches) 38" front-to-back with bumpers
    public static final double DRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE = HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE / 2.0; // (in inches)
    public static final double DRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE = HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE / 2.0; // (in inches)

    //================================================= Arm =================================================================================================================

    public static final double ARM_EXTENTION_LENGTH = 8.0; // in inches

    public static final double ARM_LINEAR_ACTUATOR_RETRACTED_LENGTH = 16.93; // in inches
    public static final double ARM_LINEAR_ACTUATOR_EXTENDED_LENGTH = HardwareConstants.ARM_LINEAR_ACTUATOR_RETRACTED_LENGTH + HardwareConstants.ARM_EXTENTION_LENGTH; // in inches

    // values may change:
    public static final double ARM_LOWER_ARM_LENGTH = 46.0; // Diagram D11, in inches
    public static final double ARM_UPPER_ARM_LENGTH = 37.0; // Diagram D12, in inches

    // max retraction/extension degrees for lower and upper arm joints
    public static final double ARM_LOWER_JOINT_CONSTRAINT_MIN = 45.0; // in degrees
    public static final double ARM_LOWER_JOINT_CONSTRAINT_MAX = 90.0; // in degrees
    public static final double ARM_UPPER_JOINT_CONSTRAINT_MIN = 10.0; // in degrees
    public static final double ARM_UPPER_JOINT_CONSTRAINT_MAX = 170.0; // in degrees

    public static final double ARM_MAX_END_EFFECTOR_HEIGHT = 1.5; // inches, max height of the end-effector above the end of the upper arm
    public static final double ARM_MAX_END_EFFECTOR_EXTENSION = 8.0; // inches, max extension of the end-effector past the end of the upper arm

    public static final double ARM_ORIGIN_X_OFFSET = 16.0; // inches, distance from the front of the robot to the origin for the arm
    public static final double ARM_ORIGIN_Z_OFFSET = 7.5; // inches, distance from the floor to the origin for the arm

    public static final double ARM_STRING_ENCODER_TICKS_PER_INCH = (4096.0 / 100.0) * 25.4;
    public static final double ARM_STRING_ENCODER_INCHES_PER_TICK = (100.0 / 4096.0) / 25.4;

    public static final double ARM_FULL_EXTENSION_TICKS = HardwareConstants.ARM_EXTENTION_LENGTH * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH; // in ticks

    // ======Lower Arm Linear Actuator================
    public static final double ARM_LOWER_ARM_TOP_PIN_OF_LINEAR_ACTUATOR_TO_PIN_ON_LOWER_ARM = 19.857; // Diagram L10
    public static final double ARM_LOWER_ARM_BOTTOM_PIN_OF_LINEAR_ACTUATOR_TO_PIN_ON_LOWER_ARM = 12.0234; // Diagram L11
    public static final double ARM_LOWER_ARM_LINEAR_ACTUATOR_RIGHT_ANGLE_OFFSET = Helpers.atan2d(HardwareConstants.ARM_UPPER_ARM_D9, HardwareConstants.ARM_UPPER_ARM_D10); // Diagram rho, Place holder until verified from CAD
    public static final double ARM_LOWER_ARM_LINEAR_ACTUATOR_LEFT_ANGLE_OFFSET = Helpers.atan2d(HardwareConstants.ARM_UPPER_ARM_D8, HardwareConstants.ARM_UPPER_ARM_D7); // Diagram lambda, Place holder
    public static final double ARM_UPPER_ARM_D7 = 12.0;
    public static final double ARM_UPPER_ARM_D8 = 0.75;
    public static final double ARM_UPPER_ARM_D9 = 4.0;
    public static final double ARM_UPPER_ARM_D10 = 19.45;
    // ======Upper Arm Linear Actuator================
    public static final double ARM_UPPER_ARM_FOUR_BAR_FOLLOWER_PIN_DISTANCE = 9.5; // Diagram L1
    public static final double ARM_UPPER_ARM_FOUR_BAR_COUPLER_PIN_DISTANCE = 13.0; // Diagram L2
    public static final double ARM_UPPER_ARM_FOUR_BAR_DRIVER_PIN_DISTANCE = 12.3; // Diagram L3
    public static final double ARM_UPPER_ARM_FOUR_BAR_GROUND_PIN_DISTANCE = 10.30776; // Diagram L4
    public static final double ARM_UPPER_ARM_D1 = 5.055; // Diagram D1
    public static final double ARM_UPPER_ARM_D2 = -1.055; // Diagram D2
    public static final double ARM_UPPER_ARM_D3 = 21.0; // Diagram D3
    public static final double ARM_UPPER_ARM_D4 = 9.5; // Diagram D4
    public static final double ARM_UPPER_ARM_D5 = 4.5; // Diagram D5
    public static final double ARM_UPPER_ARM_D6 = 2.0; // Diagram D6
    public static final double ARM_UPPER_ARM_L7 = 4.924; // Diagram L7
    public static final double ARM_UPPER_ARM_L8 = 21.026; // Diagram L8
    public static final double ARM_UPPER_ARM_SIGMA_ANGLE = Helpers.atan2d(HardwareConstants.ARM_UPPER_ARM_D6, HardwareConstants.ARM_UPPER_ARM_D5); // Diagram sigma
    public static final double ARM_UPPER_ARM_PSI_ANGLE = Helpers.atan2d(HardwareConstants.ARM_UPPER_ARM_D2, HardwareConstants.ARM_UPPER_ARM_D3); // Diagram psi
    public static final double ARM_UPPER_ARM_PHI_ANGLE = Helpers.atan2d(HardwareConstants.ARM_UPPER_ARM_D1 + HardwareConstants.ARM_UPPER_ARM_D2, HardwareConstants.ARM_UPPER_ARM_D4); // Diagram phi
}