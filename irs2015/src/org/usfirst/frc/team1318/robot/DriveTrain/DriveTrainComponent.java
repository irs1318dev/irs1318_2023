package org.usfirst.frc.team1318.robot.DriveTrain;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;
import org.usfirst.frc.team1318.robot.HardwareConstants;
import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.Common.SmartDashboardLogger;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
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
    public static final String PROXIMITY_SENSOR_BACK_LOG_KEY = "dt.proximitySensorBack";
    public static final String PROXIMITY_SENSOR_FRONT_LOG_KEY = "dt.proximitySensorFront";

    private final Victor leftTalon;
    private final Victor rightTalon;

    private final Encoder leftEncoder;
    private final Encoder rightEncoder;

    private final AnalogInput proximitySensorFront;
    private final AnalogInput proximitySensorBack;

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

        this.proximitySensorBack = new AnalogInput(ElectronicsConstants.DRIVETRAIN_PROXIMITY_SENSOR_BACK_PORT);
        this.proximitySensorFront = new AnalogInput(ElectronicsConstants.DRIVETRAIN_PROXIMITY_SENSOR_FRONT_PORT);
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

        if (outLeftPower > 0)
        {
            outLeftPower /= TuningConstants.DRIVETRAIN_REVERSE_LEFT_SCALE_FACTOR;
        }
        if (outRightPower > 0)
        {
            outRightPower /= TuningConstants.DRIVETRAIN_REVERSE_RIGHT_SCALE_FACTOR;
        }

        outLeftPower = Math.min(outLeftPower, 1);
        outLeftPower = Math.max(outLeftPower, -1);
        outRightPower = Math.min(outRightPower, 1);
        outRightPower = Math.max(outRightPower, -1);

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
        double leftVelocity = -this.leftEncoder.getRate();

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
        double rightVelocity = this.rightEncoder.getRate();

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

    @Override
    public double getProximitySensorFront()
    {
        double value = this.proximitySensorFront.getVoltage();
        SmartDashboardLogger.putNumber(DriveTrainComponent.PROXIMITY_SENSOR_FRONT_LOG_KEY, value);
        return value;
    }

    @Override
    public double getProximitySensorBack()
    {
        double value = this.proximitySensorBack.getVoltage();
        SmartDashboardLogger.putNumber(DriveTrainComponent.PROXIMITY_SENSOR_BACK_LOG_KEY, value);
        return value;
    }

    public void reset()
    {
        this.leftEncoder.reset();
        this.rightEncoder.reset();
    }
}
