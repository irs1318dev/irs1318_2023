package org.usfirst.frc.team1318.robot.DriveTrain;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;
import org.usfirst.frc.team1318.robot.HardwareConstants;
import org.usfirst.frc.team1318.robot.Common.SmartDashboardLogger;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

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
    public static final String LEFT_TALON_POWER_LOG_KEY = "dt.re";
    public static final String RIGHT_TALON_POWER_LOG_KEY = "dt.le";
    public static final String LEFT_ENCODER_VELOCITY_LOG_KEY = "dt.lev";
    public static final String RIGHT_ENCODER_VELOCITY_LOG_KEY = "dt.rev";
    public static final String LEFT_ENCODER_DISTANCE_LOG_KEY = "dt.led";
    public static final String RIGHT_ENCODER_DISTANCE_LOG_KEY = "dt.red";
    public static final String SHIFTER_STATE_LOG_KEY = "dt.ss";

    private Talon leftTalon;
    private Talon rightTalon;

    private Encoder leftEncoder;
    private Encoder rightEncoder;
    
    private DoubleSolenoid shifter;

    /**
     * Initializes a new DriveTrainComponent
     */
    public DriveTrainComponent()
    {
        this.leftTalon = new Talon(
            ElectronicsConstants.DRIVETRAIN_LEFT_TALON_CHANNEL);

        this.rightTalon = new Talon(
            ElectronicsConstants.DRIVETRAIN_RIGHT_TALON_CHANNEL);

        this.leftEncoder = new Encoder(
            ElectronicsConstants.DRIVETRAIN_LEFT_ENCODER_CHANNEL_A,
            ElectronicsConstants.DRIVETRAIN_LEFT_ENCODER_CHANNEL_B);

        this.rightEncoder = new Encoder(
            ElectronicsConstants.DRIVETRAIN_RIGHT_ENCODER_CHANNEL_A,
            ElectronicsConstants.DRIVETRAIN_RIGHT_ENCODER_CHANNEL_B);

        this.leftEncoder.setDistancePerPulse(HardwareConstants.DRIVETRAIN_LEFT_PULSE_DISTANCE);
        this.rightEncoder.setDistancePerPulse(HardwareConstants.DRIVETRAIN_RIGHT_PULSE_DISTANCE);
        
        shifter = new DoubleSolenoid(ElectronicsConstants.DRIVETRAIN_SHIFTER_MODE_EXTENDER_PORT, ElectronicsConstants.DRIVETRAIN_SHIFTER_MODE_RETRACTER_PORT);
    }

    /**
     * set the power levels to the drive train
     * @param leftPower level to apply to the left motor
     * @param rightPower level to apply to the right motor
     */
    public void setDriveTrainPower(double leftPower, double rightPower)
    {
        this.leftTalon.set(-leftPower); // note: left motors are oriented facing "backwards"
        this.rightTalon.set(rightPower);

        SmartDashboardLogger.putNumber(DriveTrainComponent.LEFT_TALON_POWER_LOG_KEY, leftPower);
        SmartDashboardLogger.putNumber(DriveTrainComponent.RIGHT_TALON_POWER_LOG_KEY, rightPower);
    }

    /**
     * set the state of the shifter solenoid 
     * @param state state to set the solenoid to 
     */
    public void setShifterState(boolean state)
    {
    	if(state)
    	{
    		shifter.set(Value.kForward);
    	}
    	else
    	{
    		shifter.set(Value.kReverse);
    	}
    	
    	SmartDashboardLogger.putBoolean(DriveTrainComponent.SHIFTER_STATE_LOG_KEY, state);
    }
    
    /**
     * get the velocity from the left encoder
     * @return a value indicating the velocity
     */
    public double getLeftEncoderVelocity()
    {
        double leftRate = this.leftEncoder.getRate();

        SmartDashboardLogger.putNumber(DriveTrainComponent.LEFT_ENCODER_VELOCITY_LOG_KEY, leftRate);

        return leftRate;
    }

    /**
     * get the velocity from the right encoder
     * @return a value indicating the velocity
     */
    public double getRightEncoderVelocity()
    {
        double rightRate = this.rightEncoder.getRate();

        SmartDashboardLogger.putNumber(DriveTrainComponent.RIGHT_ENCODER_VELOCITY_LOG_KEY, rightRate);

        return rightRate;
    }

    /**
     * get the distance from the left encoder
     * @return a value indicating the distance
     */
    public double getLeftEncoderDistance()
    {
        double leftDistance = this.leftEncoder.getDistance(); 

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
}
