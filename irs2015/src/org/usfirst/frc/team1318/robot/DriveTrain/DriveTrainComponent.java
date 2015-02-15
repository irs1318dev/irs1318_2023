package org.usfirst.frc.team1318.robot.DriveTrain;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;
import org.usfirst.frc.team1318.robot.HardwareConstants;
import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.Common.SmartDashboardLogger;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;

/**
 * The drivetrain component class describes the electronics of the drivetrain and defines the abstract way to control it.
 * The drivetrain electronics include two talons (left and right), and two encoders (left and right). 
 *  
 * @author Will
 *
 */
public class DriveTrainComponent implements IDriveTrainComponent
{
    // logging constants
    public static final String LEFT_TALON_POWER_LOG_KEY = "dt.leftPower";
    public static final String RIGHT_TALON_POWER_LOG_KEY = "dt.rightPower";
    public static final String LEFT_ENCODER_VELOCITY_LOG_KEY = "dt.leftEncoderVelocity";
    public static final String RIGHT_ENCODER_VELOCITY_LOG_KEY = "dt.rightEncoderVelocity";
    public static final String LEFT_ENCODER_DISTANCE_LOG_KEY = "dt.leftEncoderDistance";
    public static final String RIGHT_ENCODER_DISTANCE_LOG_KEY = "dt.rightEncoderDistance";
    public static final String LEFT_ENCODER_TICKS_LOG_KEY = "dt.leftEncoderTicks";
    public static final String RIGHT_ENCODER_TICKS_LOG_KEY = "dt.rightEncoderTicks";

    private static final double MinTimeStep = 0.01;

    private final Victor leftTalon;
    private final Victor rightTalon;

    private final Encoder leftEncoder;
    private final Encoder rightEncoder;

    // For adjusting rate based on our own timing:
    private final Timer timer;
    private double prevTime;

    private int prevLeftTicks;
    private double prevLeftRate;
    private int prevRightTicks;
    private double prevRightRate;

    /**
     * Initializes a new DriveTrainComponent
     */
    public DriveTrainComponent()
    {
        this.leftTalon = new Victor(
            ElectronicsConstants.DRIVETRAIN_LEFT_TALON_CHANNEL);

        this.rightTalon = new Victor(
            ElectronicsConstants.DRIVETRAIN_RIGHT_TALON_CHANNEL);

        this.leftEncoder = new Encoder(
            ElectronicsConstants.DRIVETRAIN_LEFT_ENCODER_CHANNEL_A,
            ElectronicsConstants.DRIVETRAIN_LEFT_ENCODER_CHANNEL_B);

        this.rightEncoder = new Encoder(
            ElectronicsConstants.DRIVETRAIN_RIGHT_ENCODER_CHANNEL_A,
            ElectronicsConstants.DRIVETRAIN_RIGHT_ENCODER_CHANNEL_B);

        this.leftEncoder.setDistancePerPulse(HardwareConstants.DRIVETRAIN_LEFT_PULSE_DISTANCE);
        this.rightEncoder.setDistancePerPulse(HardwareConstants.DRIVETRAIN_RIGHT_PULSE_DISTANCE);

        this.timer = new Timer();
        this.timer.start();

        this.prevTime = this.timer.get();
        this.prevLeftTicks = -this.leftEncoder.get();
        this.prevRightTicks = this.rightEncoder.get();
    }

    /**
     * set the power levels to the drive train
     * @param leftPower level to apply to the left motor
     * @param rightPower level to apply to the right motor
     */
    public void setDriveTrainPower(double leftPower, double rightPower)
    {
        double outLeftPower = leftPower;
        double outRightPower = -rightPower;

        if (outLeftPower < 0)
        {
            outLeftPower *= TuningConstants.DRIVETRAIN_REVERSE_LEFT_SCALE_FACTOR;
        }
        if (outRightPower < 0)
        {
            outRightPower *= TuningConstants.DRIVETRAIN_REVERSE_RIGHT_SCALE_FACTOR;
        }

        this.leftTalon.set(outLeftPower);
        this.rightTalon.set(outRightPower); // note: right motors are oriented facing "backwards"

        SmartDashboardLogger.putNumber(DriveTrainComponent.LEFT_TALON_POWER_LOG_KEY, leftPower);
        SmartDashboardLogger.putNumber(DriveTrainComponent.RIGHT_TALON_POWER_LOG_KEY, rightPower);
    }

    /**
     * get the velocity from the left encoder
     * @return a value indicating the velocity
     */
    public double getLeftEncoderVelocity()
    {
        // if we haven't hit our minimum time step, default to our previously reported rate
        double leftVelocity = this.prevLeftRate;

        double currentTime = this.timer.get();
        double dt = currentTime - this.prevTime;
        if (dt > DriveTrainComponent.MinTimeStep)
        {
            int currentLeftTicks = -this.leftEncoder.get();
            double deltaLeftTicks = currentLeftTicks - this.prevLeftTicks;
            leftVelocity = deltaLeftTicks / dt;

            this.prevLeftTicks = currentLeftTicks;
        }

        SmartDashboardLogger.putNumber(DriveTrainComponent.LEFT_ENCODER_VELOCITY_LOG_KEY, leftVelocity);

        return leftVelocity;
    }

    /**
     * get the velocity from the right encoder
     * @return a value indicating the velocity
     */
    public double getRightEncoderVelocity()
    {
        // if we haven't hit our minimum time step, default to our previously reported rate
        double rightVelocity = this.prevRightRate;

        double currentTime = this.timer.get();
        double dt = currentTime - this.prevTime;
        if (dt > DriveTrainComponent.MinTimeStep)
        {
            int currentRightTicks = this.rightEncoder.get();
            double deltaRightTicks = currentRightTicks - this.prevRightTicks;
            rightVelocity = deltaRightTicks / dt;

            this.prevRightTicks = currentRightTicks;
        }

        SmartDashboardLogger.putNumber(DriveTrainComponent.RIGHT_ENCODER_VELOCITY_LOG_KEY, rightVelocity);

        return rightVelocity;
    }

    /**
     * get the distance from the left encoder
     * @return a value indicating the distance
     */
    public double getLeftEncoderDistance()
    {
        double leftDistance = -this.leftEncoder.getDistance();

        SmartDashboardLogger.putNumber(DriveTrainComponent.LEFT_ENCODER_DISTANCE_LOG_KEY, leftDistance);

        return leftDistance;
    }

    /**
     * get the distance from the right encoder
     * @return a value indicating the distance
     */
    public double getRightEncoderDistance()
    {
        double rightDistance = this.rightEncoder.getDistance();

        SmartDashboardLogger.putNumber(DriveTrainComponent.RIGHT_ENCODER_DISTANCE_LOG_KEY, rightDistance);

        return rightDistance;
    }

    /**
     * get the ticks from the left encoder
     * @return a value indicating the number of ticks we are at
     */
    public int getLeftEncoderTicks()
    {
        int leftTicks = -this.leftEncoder.get();

        SmartDashboardLogger.putNumber(DriveTrainComponent.LEFT_ENCODER_TICKS_LOG_KEY, leftTicks);

        return leftTicks;
    }

    /**
     * get the ticks from the right encoder
     * @return a value indicating the number of ticks we are at
     */
    public int getRightEncoderTicks()
    {
        int rightTicks = this.rightEncoder.get();

        SmartDashboardLogger.putNumber(DriveTrainComponent.RIGHT_ENCODER_TICKS_LOG_KEY, rightTicks);

        return rightTicks;
    }
}
