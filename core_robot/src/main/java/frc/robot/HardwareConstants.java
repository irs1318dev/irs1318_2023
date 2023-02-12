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
    public static final double DRIVETRAIN_DRIVE_WHEEL_DIAMETER = 3.95; // SDS Mk4i code claims their 4-inch wheels are actually 3.95 inches now (in inches)
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

    // values may change:
    public static final double ARM_LOWER_ARM_LENGTH = 46.0; // in inches
    public static final double ARM_UPPER_ARM_LENGTH = 37.0; // in inches

    public static final double ARM_MAX_END_EFFECTOR_HEIGHT = 8.0; // inches, max height of the end-effector above the end of the upper arm
    public static final double ARM_MAX_END_EFFECTOR_EXTENSION = 8.0; // inches, max extension of the end-effector past the end of the upper arm

    public static final double ARM_ORIGIN_X_OFFSET = 8.0; // inches, distance from the front of the robot to the origin for the arm
    public static final double ARM_ORIGIN_Z_OFFSET = 8.0; // inches, distance from the floor to the origin for the arm

    //======Lower Arm Linear Actuator================
   
    public static final double LOWER_ARM_TOP_PIN_OF_LINEAR_ACTUATOR_TO_PIN_ON_LOWER_ARM = 18.347; // There has to be a better name for this
    public static final double LOWER_ARM_BOTTOM_PIN_OF_LINEAR_ACTUATOR_TO_PIN_ON_LOWER_ARM = 12.876;
    public static final double LOWER_ARM_LINEAR_ACTUATOR_RIGHT_ANGLE_OFFSET = 15; // Place holder until verified from CAD
    public static final double LOWER_ARM_LINEAR_ACTUATOR_LEFT_ANGLE_OFFSET = 10; // Place holder
    
    //======Upper Arm Linear Actuator================
    public static final double LENGTH_ONE = 8.963; // All Place holders
    public static final double LENGTH_TWO = 13.073;
    public static final double LENGTH_THREE = 12.515;
    public static final double LENGTH_FOUR = 10.308;
    public static final double DISTANCE_ONE = 3.885;
    public static final double DISTANCE_TWO = 0.087;
    public static final double DISTANCE_THREE = 21.049;
    public static final double DISTANCE_FOUR = 15.075;
    public static final double DISTANCE_FIVE = 4;
    public static final double DISTANCE_SIX = 2;
    public static final double SIGMA_ANGLE = Helpers.atan2d(HardwareConstants.DISTANCE_SIX, HardwareConstants.DISTANCE_FIVE); // Can calculate and hard code after we get measurements
    public static final double PSI_ANGLE = Helpers.atan2d(HardwareConstants.DISTANCE_TWO, HardwareConstants.DISTANCE_THREE); // Hard code later
    public static final double phiAngle = Helpers.atan2d(HardwareConstants.DISTANCE_ONE + HardwareConstants.DISTANCE_TWO, HardwareConstants.DISTANCE_FOUR);

    public static final double LINEAR_ACTUATOR_LENGTH = 17.63;
}