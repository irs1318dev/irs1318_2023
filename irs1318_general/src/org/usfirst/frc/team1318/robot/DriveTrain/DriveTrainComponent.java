package org.usfirst.frc.team1318.robot.DriveTrain;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;
import org.usfirst.frc.team1318.robot.HardwareConstants;
import org.usfirst.frc.team1318.robot.Common.DashboardLogger;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Victor;

/**
 * The drivetrain component class describes the electronics of the drivetrain and defines the abstract way to control it.
 * The drivetrain electronics include two talons (left and right), and two encoders (left and right). 
 * 
 */
public class DriveTrainComponent
{
    private final Victor leftTalon;
    private final Victor rightTalon;

    private final Encoder leftEncoder;
    private final Encoder rightEncoder;

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
    }

    /**
     * set the power levels to the drive train
     * @param leftPower level to apply to the left motor
     * @param rightPower level to apply to the right motor
     */
    public void setDriveTrainPower(double leftPower, double rightPower)
    {
        DashboardLogger.putDouble("leftPower", leftPower);
        DashboardLogger.putDouble("rightPower", rightPower);

        double outLeftPower = leftPower;
        double outRightPower = -rightPower;// note: right motors are oriented facing "backwards"

        this.leftTalon.set(outLeftPower);
        this.rightTalon.set(outRightPower);
    }

    /**
     * get the velocity from the left encoder
     * @return a value indicating the velocity
     */
    public double getLeftEncoderVelocity()
    {
        double leftVelocity = -this.leftEncoder.getRate();
        DashboardLogger.putDouble("leftVelocity", leftVelocity);
        return leftVelocity;
    }

    /**
     * get the velocity from the right encoder
     * @return a value indicating the velocity
     */
    public double getRightEncoderVelocity()
    {
        double rightVelocity = this.rightEncoder.getRate();
        DashboardLogger.putDouble("rightVelocity", rightVelocity);
        return rightVelocity;
    }

    /**
     * get the distance from the left encoder
     * @return a value indicating the distance
     */
    public double getLeftEncoderDistance()
    {
        double leftDistance = -this.leftEncoder.getDistance();
        DashboardLogger.putDouble("leftDistance", leftDistance);
        return leftDistance;
    }

    /**
     * get the distance from the right encoder
     * @return a value indicating the distance
     */
    public double getRightEncoderDistance()
    {
        double rightDistance = this.rightEncoder.getDistance();
        DashboardLogger.putDouble("rightDistance", rightDistance);
        return rightDistance;
    }

    /**
     * get the ticks from the left encoder
     * @return a value indicating the number of ticks we are at
     */
    public int getLeftEncoderTicks()
    {
        int leftTicks = -this.leftEncoder.get();
        DashboardLogger.putDouble("leftTicks", leftTicks);
        return leftTicks;
    }

    /**
     * get the ticks from the right encoder
     * @return a value indicating the number of ticks we are at
     */
    public int getRightEncoderTicks()
    {
        int rightTicks = this.rightEncoder.get();
        DashboardLogger.putDouble("rightTicks", rightTicks);
        return rightTicks;
    }

    /**
     * reset the encoder so it considers the current location to be "0"
     */
    public void reset()
    {
        this.leftEncoder.reset();
        this.rightEncoder.reset();
    }
}
