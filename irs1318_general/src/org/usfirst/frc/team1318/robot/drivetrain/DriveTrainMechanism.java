package org.usfirst.frc.team1318.robot.drivetrain;

import javax.inject.Singleton;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;
import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.common.Helpers;
import org.usfirst.frc.team1318.robot.common.IDashboardLogger;
import org.usfirst.frc.team1318.robot.common.IMechanism;
import org.usfirst.frc.team1318.robot.common.PIDHandler;
import org.usfirst.frc.team1318.robot.common.wpilib.IEncoder;
import org.usfirst.frc.team1318.robot.common.wpilib.IMotor;
import org.usfirst.frc.team1318.robot.common.wpilib.ITimer;
import org.usfirst.frc.team1318.robot.common.wpilib.IWpilibProvider;
import org.usfirst.frc.team1318.robot.driver.Operation;
import org.usfirst.frc.team1318.robot.driver.common.Driver;

import com.google.inject.Inject;

/**
 * Drivetrain mechanism.
 * The mechanism defines the logic that controls a mechanism given inputs and operator-requested actions, and 
 * translates those into the abstract functions that should be applied to the outputs.
 * 
 */
@Singleton
public class DriveTrainMechanism implements IMechanism
{
    private final static String LogName = "dt";

    private static final double POWERLEVEL_MIN = -1.0;
    private static final double POWERLEVEL_MAX = 1.0;

    private final IDashboardLogger logger;
    private final ITimer timer;

    private final IMotor leftMotor;
    private final IMotor rightMotor;

    private final IEncoder leftEncoder;
    private final IEncoder rightEncoder;

    private Driver driver;

    private boolean usePID;
    private boolean usePositionalMode;
    private PIDHandler leftPID;
    private PIDHandler rightPID;

    /**
     * Initializes a new DriveTrainMechanism
     * @param logger to use
     * @param usePID indicates whether we should use PID control
     * @param provider for obtaining electronics objects
     */
    @Inject
    public DriveTrainMechanism(
        IDashboardLogger logger,
        ITimer timer,
        IWpilibProvider provider)
    {
        this.logger = logger;
        this.timer = timer;

        this.leftMotor = provider.getTalon(ElectronicsConstants.DRIVETRAIN_LEFT_TALON_CHANNEL);
        this.rightMotor = provider.getTalon(ElectronicsConstants.DRIVETRAIN_RIGHT_TALON_CHANNEL);
        this.leftEncoder = provider.getEncoder(ElectronicsConstants.DRIVETRAIN_LEFT_ENCODER_CHANNEL_A, ElectronicsConstants.DRIVETRAIN_LEFT_ENCODER_CHANNEL_B);
        this.rightEncoder = provider.getEncoder(ElectronicsConstants.DRIVETRAIN_RIGHT_ENCODER_CHANNEL_A, ElectronicsConstants.DRIVETRAIN_RIGHT_ENCODER_CHANNEL_B);

        this.usePID = TuningConstants.DRIVETRAIN_USE_PID;
        this.usePositionalMode = false;

        this.createPIDHandler();
    }

    /**
     * get the velocity from the left encoder
     * @return a value indicating the velocity
     */
    public double getLeftEncoderVelocity()
    {
        double leftVelocity = -this.leftEncoder.getRate();
        this.logger.logNumber(DriveTrainMechanism.LogName, "leftVelocity", leftVelocity);
        return leftVelocity;
    }

    /**
     * get the velocity from the right encoder
     * @return a value indicating the velocity
     */
    public double getRightEncoderVelocity()
    {
        double rightVelocity = this.rightEncoder.getRate();
        this.logger.logNumber(DriveTrainMechanism.LogName, "rightVelocity", rightVelocity);
        return rightVelocity;
    }

    /**
     * get the distance from the left encoder
     * @return a value indicating the distance
     */
    public double getLeftEncoderDistance()
    {
        double leftDistance = -this.leftEncoder.getDistance();
        this.logger.logNumber(DriveTrainMechanism.LogName, "leftDistance", leftDistance);
        return leftDistance;
    }

    /**
     * get the distance from the right encoder
     * @return a value indicating the distance
     */
    public double getRightEncoderDistance()
    {
        double rightDistance = this.rightEncoder.getDistance();
        this.logger.logNumber(DriveTrainMechanism.LogName, "rightDistance", rightDistance);
        return rightDistance;
    }

    /**
     * get the ticks from the left encoder
     * @return a value indicating the number of ticks we are at
     */
    public int getLeftEncoderTicks()
    {
        int leftTicks = -this.leftEncoder.get();
        this.logger.logNumber(DriveTrainMechanism.LogName, "leftTicks", leftTicks);
        return leftTicks;
    }

    /**
     * get the ticks from the right encoder
     * @return a value indicating the number of ticks we are at
     */
    public int getRightEncoderTicks()
    {
        int rightTicks = this.rightEncoder.get();
        this.logger.logNumber(DriveTrainMechanism.LogName, "rightTicks", rightTicks);
        return rightTicks;
    }

    /**
     * set the driver that the mechanism should use
     * @param driver to use
     */
    @Override
    public void setDriver(Driver driver)
    {
        this.driver = driver;

        // switch to default velocity PID mode whenever we switch drivers (defense-in-depth)
        if (!this.usePID || this.usePositionalMode)
        {
            this.usePID = TuningConstants.DRIVETRAIN_USE_PID;
            this.usePositionalMode = false;

            this.createPIDHandler();
        }
    }

    /**
     * calculate the various outputs to use based on the inputs and apply them to the outputs for the relevant mechanism
     */
    @Override
    public void update()
    {
        if (this.driver.getDigital(Operation.DriveTrainEnablePID))
        {
            this.usePID = true;
            this.createPIDHandler();
        }
        else if (this.driver.getDigital(Operation.DriveTrainDisablePID))
        {
            this.usePID = false;
            this.createPIDHandler();
        }

        // check our desired PID mode (needed for positional mode or break mode)
        boolean newUsePositionalMode = this.driver.getDigital(Operation.DriveTrainUsePositionalMode);
        if (newUsePositionalMode != this.usePositionalMode)
        {
            this.usePositionalMode = newUsePositionalMode;

            // re-create PID handler
            this.createPIDHandler();
        }

        // calculate desired power setting for the current mode
        PowerSetting powerSetting;
        if (!this.usePositionalMode)
        {
            powerSetting = this.calculateVelocityModePowerSetting();
        }
        else
        {
            powerSetting = this.calculatePositionModePowerSetting();
        }

        double leftPower = powerSetting.getLeftPower();
        double rightPower = powerSetting.getRightPower();

        if (leftPower > 0)
        {
            leftPower /= TuningConstants.DRIVETRAIN_REVERSE_LEFT_SCALE_FACTOR;
        }

        if (rightPower > 0)
        {
            rightPower /= TuningConstants.DRIVETRAIN_REVERSE_RIGHT_SCALE_FACTOR;
        }

        leftPower = this.applyPowerLevelRange(leftPower);
        rightPower = this.applyPowerLevelRange(rightPower);

        // apply the power settings to the motors
        this.leftMotor.set(leftPower);
        this.rightMotor.set(-rightPower); // note: the right motor is oriented "backwards"
    }

    /**
     * stop the relevant mechanism
     */
    @Override
    public void stop()
    {
        this.leftMotor.set(0.0);
        this.rightMotor.set(0.0);

        this.leftEncoder.reset();
        this.rightEncoder.reset();
        if (this.leftPID != null)
        {
            this.leftPID.reset();
        }

        if (this.rightPID != null)
        {
            this.rightPID.reset();
        }
    }

    /**
     * create a PIDHandler based on our current settings
     */
    private void createPIDHandler()
    {
        if (!this.usePID)
        {
            this.leftPID = null;
            this.rightPID = null;
        }
        else
        {
            if (this.usePositionalMode)
            {
                this.leftPID = new PIDHandler(
                    TuningConstants.DRIVETRAIN_POSITION_PID_LEFT_KP,
                    TuningConstants.DRIVETRAIN_POSITION_PID_LEFT_KI,
                    TuningConstants.DRIVETRAIN_POSITION_PID_LEFT_KD,
                    TuningConstants.DRIVETRAIN_POSITION_PID_LEFT_KF,
                    1.0,
                    -TuningConstants.DRIVETRAIN_POSITIONAL_MAX_POWER_LEVEL,
                    TuningConstants.DRIVETRAIN_POSITIONAL_MAX_POWER_LEVEL,
                    this.timer);

                this.rightPID = new PIDHandler(
                    TuningConstants.DRIVETRAIN_POSITION_PID_RIGHT_KP,
                    TuningConstants.DRIVETRAIN_POSITION_PID_RIGHT_KI,
                    TuningConstants.DRIVETRAIN_POSITION_PID_RIGHT_KD,
                    TuningConstants.DRIVETRAIN_POSITION_PID_RIGHT_KF,
                    1.0,
                    -TuningConstants.DRIVETRAIN_POSITIONAL_MAX_POWER_LEVEL,
                    TuningConstants.DRIVETRAIN_POSITIONAL_MAX_POWER_LEVEL,
                    this.timer);
            }
            else
            {
                this.leftPID = new PIDHandler(
                    TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KP,
                    TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KI,
                    TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KD,
                    TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KF,
                    TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KS,
                    -TuningConstants.DRIVETRAIN_VELOCITY_MAX_POWER_LEVEL,
                    TuningConstants.DRIVETRAIN_VELOCITY_MAX_POWER_LEVEL,
                    "leftDT",
                    this.logger,
                    this.timer);

                this.rightPID = new PIDHandler(
                    TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KP,
                    TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KI,
                    TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KD,
                    TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KF,
                    TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KS,
                    -TuningConstants.DRIVETRAIN_VELOCITY_MAX_POWER_LEVEL,
                    TuningConstants.DRIVETRAIN_VELOCITY_MAX_POWER_LEVEL,
                    "rightDT",
                    this.logger,
                    this.timer);
                ;
            }
        }
    }

    /**
     * Calculate the power setting to use based on the inputs when in velocity mode
     * @return power settings for left and right motor
     */
    private PowerSetting calculateVelocityModePowerSetting()
    {
        // velocity goals represent the desired percentage of the max velocity
        double leftVelocityGoal = 0.0;
        double rightVelocityGoal = 0.0;

        // read the encoder distance just in case we want it output in smart dashboard
        this.getLeftEncoderDistance();
        this.getRightEncoderDistance();
        this.getLeftEncoderVelocity();
        this.getRightEncoderVelocity();

        int currentLeftTicks = this.getLeftEncoderTicks();
        int currentRightTicks = this.getRightEncoderTicks();

        // get a value indicating that we should be in simple mode...
        boolean simpleDriveModeEnabled = this.driver.getDigital(Operation.DriveTrainSimpleMode);

        // get the X and Y values from the operator.  We expect these to be between -1.0 and 1.0,
        // with this value representing the forward velocity percentage and right turn percentage (of max speed)
        double turnAmount = this.driver.getAnalog(Operation.DriveTrainTurn);
        double forwardVelocity = this.driver.getAnalog(Operation.DriveTrainMoveForward);

        // Negate the x and y if DriveTrainSwapFrontOrientation is true
        if (this.driver.getDigital(Operation.DriveTrainSwapFrontOrientation))
        {
            turnAmount *= -1.0;
            forwardVelocity *= -1.0;
        }

        // adjust the intensity of the input
        if (simpleDriveModeEnabled)
        {
            if (Math.abs(forwardVelocity) < Math.abs(turnAmount))
            {
                // in-place turn
                leftVelocityGoal = turnAmount;
                rightVelocityGoal = -turnAmount;
            }
            else
            {
                // forward/backward
                leftVelocityGoal = forwardVelocity;
                rightVelocityGoal = forwardVelocity;
            }
        }
        else
        {
            leftVelocityGoal = (TuningConstants.DRIVETRAIN_K1 * forwardVelocity) + (TuningConstants.DRIVETRAIN_K2 * turnAmount);
            rightVelocityGoal = (TuningConstants.DRIVETRAIN_K1 * forwardVelocity) + (-TuningConstants.DRIVETRAIN_K2 * turnAmount);
        }

        // decrease the desired velocity based on the configured max power level
        leftVelocityGoal = leftVelocityGoal * TuningConstants.DRIVETRAIN_MAX_POWER_LEVEL;
        rightVelocityGoal = rightVelocityGoal * TuningConstants.DRIVETRAIN_MAX_POWER_LEVEL;

        this.logger.logNumber(DriveTrainMechanism.LogName, "leftVelocityGoal", leftVelocityGoal);
        this.logger.logNumber(DriveTrainMechanism.LogName, "rightVelocityGoal", rightVelocityGoal);

        // convert velocity goal to power level...
        double leftPower;
        double rightPower;
        if (this.usePID)
        {
            leftPower = this.leftPID.calculateVelocity(
                leftVelocityGoal,
                currentLeftTicks);

            rightPower = this.rightPID.calculateVelocity(
                rightVelocityGoal,
                currentRightTicks);
        }
        else
        {
            leftPower = leftVelocityGoal;
            rightPower = rightVelocityGoal;
        }

        // ensure that our algorithms are correct and don't give values outside
        // the appropriate range
        leftPower = this.applyPowerLevelRange(leftPower);
        rightPower = this.applyPowerLevelRange(rightPower);

        return new PowerSetting(leftPower, rightPower);
    }

    /**
     * Calculate the power setting to use based on the inputs when in position mode
     * @return power settings for left and right motor
     */
    private PowerSetting calculatePositionModePowerSetting()
    {
        // get the desired left and right values from the driver.
        double leftPosition = this.driver.getAnalog(Operation.DriveTrainLeftPosition);
        double rightPosition = this.driver.getAnalog(Operation.DriveTrainRightPosition);

        // get the current distance from the encoders.
        double leftDistance = this.getLeftEncoderDistance();
        double rightDistance = this.getRightEncoderDistance();

        // read the encoder velocity just in case we want it output in smart dashboard
        this.getLeftEncoderVelocity();
        this.getRightEncoderVelocity();

        this.logger.logNumber(DriveTrainMechanism.LogName, "leftPositionGoal", leftPosition);
        this.logger.logNumber(DriveTrainMechanism.LogName, "rightPositionGoal", rightPosition);

        double leftPower;
        double rightPower;
        if (this.usePID)
        {
            // use positional PID to get the relevant value
            leftPower = this.leftPID.calculatePosition(leftPosition, leftDistance);
            rightPower = this.rightPID.calculatePosition(rightPosition, rightDistance);
        }
        else
        {
            // calculate a desired power level
            leftPower = leftPosition - leftDistance;
            rightPower = rightPosition - rightDistance;
            if (Math.abs(leftPower) < 0.1)
            {
                leftPower = 0.0;
            }

            if (Math.abs(rightPower) < 0.1)
            {
                rightPower = 0.0;
            }

            // ensure that we are within our power level range, and then scale it down
            leftPower = this.applyPowerLevelRange(leftPower) * TuningConstants.DRIVETRAIN_MAX_POWER_POSITIONAL_NON_PID;
            rightPower = this.applyPowerLevelRange(rightPower) * TuningConstants.DRIVETRAIN_MAX_POWER_POSITIONAL_NON_PID;
        }

        this.assertPowerLevelRange(leftPower, "left velocity (goal)");
        this.assertPowerLevelRange(rightPower, "right velocity (goal)");

        return new PowerSetting(leftPower, rightPower);
    }

    /**
     * Assert that the power level is within the required range
     * @param powerLevel to verify
     * @param side indicator for the exception message if incorrect
     */
    private void assertPowerLevelRange(double powerLevel, String side)
    {
        if (powerLevel < DriveTrainMechanism.POWERLEVEL_MIN)
        {
            if (TuningConstants.THROW_EXCEPTIONS)
            {
                throw new RuntimeException(side + " power level too low!");
            }

            return;
        }

        if (powerLevel > DriveTrainMechanism.POWERLEVEL_MAX)
        {
            if (TuningConstants.THROW_EXCEPTIONS)
            {
                throw new RuntimeException(side + " power level too high!");
            }

            return;
        }
    }

    /**
     * Reset the power level to be within the required range
     * @param powerLevel to reset
     * @return power level
     */
    private double applyPowerLevelRange(double powerLevel)
    {
        return Helpers.EnforceRange(powerLevel, DriveTrainMechanism.POWERLEVEL_MIN, DriveTrainMechanism.POWERLEVEL_MAX);
    }

    /**
     * Simple holder of power setting information for the left and right motor
     * (This exists only to allow splitting out common code and have only one return value, because Java doesn't support multi-return)
     */
    private class PowerSetting
    {
        private double leftPower;
        private double rightPower;

        /**
         * Initializes a new PowerSetting
         * @param leftPower to apply
         * @param rightPower to apply
         */
        public PowerSetting(double leftPower, double rightPower)
        {
            this.leftPower = leftPower;
            this.rightPower = rightPower;
        }

        /**
         * gets the left power setting 
         * @return value between -1.0 and 1.0
         */
        public double getLeftPower()
        {
            return this.leftPower;
        }

        /**
         * gets the right power setting 
         * @return value between -1.0 and 1.0
         */
        public double getRightPower()
        {
            return this.rightPower;
        }
    }
}
