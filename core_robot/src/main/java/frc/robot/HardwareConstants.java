package frc.robot;

import frc.robot.common.robotprovider.TalonFXInvertType;

/**
 * All constants describing the physical structure of the robot (distances and sizes of things).
 * 
 * @author Will
 * 
 */
public class HardwareConstants
{
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
    public static final double DRIVETRAIN_DRIVE_GEAR_RATIO = 1275.0 / 189.0; // According to SDS Mk4i fast code: (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0) == ~6.75 : 1
    public static final double DRIVETRAIN_DRIVE_WHEEL_DIAMETER = 3.95; // SDS Mk4i code claims their 4-inch wheels are actually 3.95 inches now (in inches)
    public static final double DRIVETRAIN_DRIVE_WHEEL_CIRCUMFERENCE = Math.PI * HardwareConstants.DRIVETRAIN_DRIVE_WHEEL_DIAMETER;
    public static final double DRIVETRAIN_DRIVE_TICK_DISTANCE = HardwareConstants.DRIVETRAIN_DRIVE_WHEEL_CIRCUMFERENCE / (HardwareConstants.DRIVETRAIN_DRIVE_GEAR_RATIO * HardwareConstants.DRIVETRAIN_DRIVE_TICKS_PER_REVOLUTION);
    public static final double DRIVETRAIN_DRIVE_TICKS_PER_INCH = (HardwareConstants.DRIVETRAIN_DRIVE_GEAR_RATIO * HardwareConstants.DRIVETRAIN_DRIVE_TICKS_PER_REVOLUTION) / HardwareConstants.DRIVETRAIN_DRIVE_WHEEL_CIRCUMFERENCE;
    public static final double DRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND = 10.0 * HardwareConstants.DRIVETRAIN_DRIVE_TICK_DISTANCE; // converts #ticks per 100ms into inches per second.
    public static final double DRIVETRAIN_DRIVE_INCHES_PER_SECOND_TO_MOTOR_VELOCITY = 0.1 * HardwareConstants.DRIVETRAIN_DRIVE_TICKS_PER_INCH; // converts inches per second into #ticks per 100ms.

    public static final double DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE = 22.75; // (in inches) 35" side-to-side with bumpers
    public static final double DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE = 26.75; // (in inches) 38" front-to-back with bumpers
    public static final double DRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE = HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE / 2.0; // (in inches)
    public static final double DRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE = HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE / 2.0; // (in inches)

    //================================================== Climber ==============================================================

    public static final TalonFXInvertType CLIMBER_WINCH_MOTOR_MASTER_INVERT = TalonFXInvertType.CounterClockwise;
    public static final TalonFXInvertType CLIMBER_WINCH_MOTOR_FOLLOWER_INVERT = TalonFXInvertType.FollowMaster;

    public static final double CLIMBER_WINCH_MAX_POSITION = 100.0; // units for (typical) maximum extension of the winch

    //================================================== Cargo ==============================================================

    public static final TalonFXInvertType CARGO_FLYWHEEL_MOTOR_INVERT = TalonFXInvertType.Clockwise;
    public static final TalonFXInvertType CARGO_FLYWHEEL_FOLLOWER_MOTOR_INVERT = TalonFXInvertType.OpposeMaster;
    public static final boolean CARGO_INTAKE_MOTOR_INVERT_OUTPUT = false;
    public static final boolean CARGO_FEEDER_MOTOR_INVERT_OUTPUT = true;
    public static final boolean CARGO_CONVEYOR_MOTOR_INVERT_OUTPUT = true;

    public static final double CARGO_SHOOTER_POINTBLANK_ANGLE = 74.0; // degrees
    public static final double CARGO_SHOOTER_SHORT_ANGLE = 68.5; // degrees
    public static final double CARGO_SHOOTER_MEDIUM_ANGLE = 66.0; // degrees
    public static final double CARGO_SHOOTER_LONG_ANGLE = 59.0; // degrees
    public static final double CARGO_SHOOTER_HEIGHT = 19.81; // inches
    public static final double CARGO_FLYWHEEL_DIAMETER = 4.0; // inches
    public static final double CARGO_FLYWHEEL_TICKS_PER_REVOLUTION = 2048.0;
    public static final double CARGO_FLYWHEEL_GEAR_RATIO = 1.0; // right now, shooter is 1:1
    public static final double CARGO_FLYWHEEL_CIRCUMFERENCE = Math.PI * HardwareConstants.CARGO_FLYWHEEL_DIAMETER;
    public static final double CARGO_FLYWHEEL_TICK_DISTANCE = HardwareConstants.CARGO_FLYWHEEL_CIRCUMFERENCE / (HardwareConstants.CARGO_FLYWHEEL_GEAR_RATIO * HardwareConstants.CARGO_FLYWHEEL_TICKS_PER_REVOLUTION);
    public static final double CARGO_FLYWHEEL_TICKS_PER_INCH = (HardwareConstants.CARGO_FLYWHEEL_GEAR_RATIO * HardwareConstants.CARGO_FLYWHEEL_TICKS_PER_REVOLUTION) / HardwareConstants.CARGO_FLYWHEEL_CIRCUMFERENCE;
    public static final double CARGO_FLYWHEEL_MOTOR_VELOCITY_TO_INCHES_PER_SECOND = 10.0 * HardwareConstants.CARGO_FLYWHEEL_TICK_DISTANCE; // multiplying by this value converts #ticks per 100ms into inches per second.
    public static final double CARGO_FLYWHEEL_INCHES_PER_SECOND_TO_MOTOR_VELOCITY = 0.1 * HardwareConstants.CARGO_FLYWHEEL_TICKS_PER_INCH; // multiplying by this value converts inches per second into #ticks per 100ms.
}