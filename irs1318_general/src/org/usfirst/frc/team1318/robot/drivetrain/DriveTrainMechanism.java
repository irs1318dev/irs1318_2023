package org.usfirst.frc.team1318.robot.drivetrain;

import javax.inject.Singleton;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;
import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.common.Helpers;
import org.usfirst.frc.team1318.robot.common.IDashboardLogger;
import org.usfirst.frc.team1318.robot.common.IMechanism;
import org.usfirst.frc.team1318.robot.common.wpilib.CANTalonControlMode;
import org.usfirst.frc.team1318.robot.common.wpilib.ICANTalon;
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

    private final ICANTalon leftMotor;
    private final ICANTalon rightMotor;

    private Driver driver;

    private boolean usePID;
    private boolean usePositionalMode;

    private double leftVelocity;
    private double leftError;
    private int leftTicks;
    private double rightVelocity;
    private double rightError;
    private int rightTicks;

    /**
     * Initializes a new DriveTrainMechanism
     * @param logger to use
     * @param usePID indicates whether we should use PID control
     * @param provider for obtaining electronics objects
     */
    @Inject
    public DriveTrainMechanism(
        IDashboardLogger logger,
        IWpilibProvider provider)
    {
        this.logger = logger;

        this.leftMotor = provider.getCANTalon(ElectronicsConstants.DRIVETRAIN_LEFT_MOTOR_CHANNEL);
        this.leftMotor.enableBrakeMode(false);
        this.leftMotor.reverseOutput(false);
        this.leftMotor.reverseSensor(true);

        ICANTalon leftFollowerMotor = provider.getCANTalon(ElectronicsConstants.DRIVETRAIN_LEFT_FOLLOWER_CHANNEL);
        leftFollowerMotor.enableBrakeMode(false);
        leftFollowerMotor.reverseOutput(false);
        leftFollowerMotor.changeControlMode(CANTalonControlMode.Follower);
        leftFollowerMotor.set(ElectronicsConstants.DRIVETRAIN_LEFT_MOTOR_CHANNEL);

        this.rightMotor = provider.getCANTalon(ElectronicsConstants.DRIVETRAIN_RIGHT_MOTOR_CHANNEL);
        this.rightMotor.enableBrakeMode(false);
        this.rightMotor.reverseOutput(true);
        this.rightMotor.reverseSensor(false);

        ICANTalon rightFollowerMotor = provider.getCANTalon(ElectronicsConstants.DRIVETRAIN_RIGHT_FOLLOWER_CHANNEL);
        rightFollowerMotor.changeControlMode(CANTalonControlMode.Follower);
        rightFollowerMotor.enableBrakeMode(false);
        rightFollowerMotor.reverseOutput(true);
        rightFollowerMotor.set(ElectronicsConstants.DRIVETRAIN_RIGHT_MOTOR_CHANNEL);

        this.usePID = TuningConstants.DRIVETRAIN_USE_PID;
        this.usePositionalMode = false;

        this.leftVelocity = 0.0;
        this.leftError = 0.0;
        this.leftTicks = 0;
        this.rightVelocity = 0.0;
        this.rightError = 0.0;
        this.rightTicks = 0;

        this.setControlMode();
    }

    /**
     * get the velocity from the left encoder
     * @return a value indicating the velocity
     */
    public double getLeftVelocity()
    {
        return this.leftVelocity;
    }

    /**
     * get the velocity from the right encoder
     * @return a value indicating the velocity
     */
    public double getRightVelocity()
    {
        return this.rightVelocity;
    }

    /**
     * get the distance from the left encoder
     * @return a value indicating the distance
     */
    public double getLeftError()
    {
        return this.leftError;
    }

    /**
     * get the distance from the right encoder
     * @return a value indicating the distance
     */
    public double getRightError()
    {
        return this.rightError;
    }

    /**
     * get the ticks from the left encoder
     * @return a value indicating the number of ticks we are at
     */
    public int getLeftTicks()
    {
        return this.leftTicks;
    }

    /**
     * get the ticks from the right encoder
     * @return a value indicating the number of ticks we are at
     */
    public int getRightTicks()
    {
        return this.rightTicks;
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
        }

        this.setControlMode();
    }

    /**
     * calculate the various outputs to use based on the inputs and apply them to the outputs for the relevant mechanism
     */
    @Override
    public void update()
    {
        this.leftVelocity = this.leftMotor.getSpeed();
        this.leftError = this.leftMotor.getError();
        this.leftTicks = this.leftMotor.getTicks();
        this.rightVelocity = this.rightMotor.getSpeed();
        this.rightError = this.rightMotor.getError();
        this.rightTicks = this.rightMotor.getTicks();

        this.logger.logNumber(DriveTrainMechanism.LogName, "leftVelocity", this.leftVelocity);
        this.logger.logNumber(DriveTrainMechanism.LogName, "leftError", this.leftError);
        this.logger.logNumber(DriveTrainMechanism.LogName, "leftTicks", this.leftTicks);
        this.logger.logNumber(DriveTrainMechanism.LogName, "rightVelocity", this.rightVelocity);
        this.logger.logNumber(DriveTrainMechanism.LogName, "rightError", this.rightError);
        this.logger.logNumber(DriveTrainMechanism.LogName, "rightTicks", this.rightTicks);

        if (this.driver.getDigital(Operation.DriveTrainEnablePID))
        {
            this.usePID = true;
            this.setControlMode();
        }
        else if (this.driver.getDigital(Operation.DriveTrainDisablePID))
        {
            this.usePID = false;
            this.setControlMode();
        }

        // check our desired PID mode (needed for positional mode or break mode)
        boolean newUsePositionalMode = this.driver.getDigital(Operation.DriveTrainUsePositionalMode);
        if (newUsePositionalMode != this.usePositionalMode)
        {
            this.usePositionalMode = newUsePositionalMode;

            // re-create PID handler
            this.setControlMode();
        }

        // calculate desired power setting for the current mode
        Setpoint setpoint;
        if (!this.usePositionalMode)
        {
            setpoint = this.calculateVelocityModeSetpoint();
        }
        else
        {
            setpoint = this.calculatePositionModeSetpoint();
        }

        double leftSetpoint = setpoint.getLeft();
        double rightSetpoint = setpoint.getRight();

        // apply the power settings to the motors
        this.leftMotor.set(leftSetpoint);
        this.rightMotor.set(rightSetpoint);
    }

    /**
     * stop the relevant mechanism
     */
    @Override
    public void stop()
    {
        this.leftMotor.changeControlMode(CANTalonControlMode.PercentVbus);
        this.rightMotor.changeControlMode(CANTalonControlMode.PercentVbus);

        this.leftMotor.set(0.0);
        this.rightMotor.set(0.0);

        this.leftMotor.reset();
        this.rightMotor.reset();

        this.leftVelocity = 0.0;
        this.leftError = 0.0;
        this.leftTicks = 0;
        this.rightVelocity = 0.0;
        this.rightError = 0.0;
        this.rightTicks = 0;
    }

    /**
     * create a PIDHandler based on our current settings
     */
    private void setControlMode()
    {
        CANTalonControlMode mode = CANTalonControlMode.PercentVbus;
        if (this.usePID)
        {
            double leftKp;
            double leftKi;
            double leftKd;
            double leftKf;
            double rightKp;
            double rightKi;
            double rightKd;
            double rightKf;

            if (this.usePositionalMode)
            {
                mode = CANTalonControlMode.Position;
                leftKp = TuningConstants.DRIVETRAIN_POSITION_PID_LEFT_KP;
                leftKi = TuningConstants.DRIVETRAIN_POSITION_PID_LEFT_KI;
                leftKd = TuningConstants.DRIVETRAIN_POSITION_PID_LEFT_KD;
                leftKf = TuningConstants.DRIVETRAIN_POSITION_PID_LEFT_KF;
                rightKp = TuningConstants.DRIVETRAIN_POSITION_PID_RIGHT_KP;
                rightKi = TuningConstants.DRIVETRAIN_POSITION_PID_RIGHT_KI;
                rightKd = TuningConstants.DRIVETRAIN_POSITION_PID_RIGHT_KD;
                rightKf = TuningConstants.DRIVETRAIN_POSITION_PID_RIGHT_KF;
            }
            else
            {
                mode = CANTalonControlMode.Speed;
                leftKp = TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KP;
                leftKi = TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KI;
                leftKd = TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KD;
                leftKf = TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KF;
                rightKp = TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KP;
                rightKi = TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KI;
                rightKd = TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KD;
                rightKf = TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KF;
            }

            this.leftMotor.setPIDF(leftKp, leftKi, leftKd, leftKf);
            this.rightMotor.setPIDF(rightKp, rightKi, rightKd, rightKf);
        }

        this.leftMotor.changeControlMode(mode);
        this.rightMotor.changeControlMode(mode);
    }

    /**
     * Calculate the power setting to use based on the inputs when in velocity mode
     * @return power settings for left and right motor
     */
    private Setpoint calculateVelocityModeSetpoint()
    {
        // velocity goals represent the desired percentage of the max velocity
        double leftVelocityGoal = 0.0;
        double rightVelocityGoal = 0.0;

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

        // ensure that we don't give values outside the appropriate range
        double left = this.applyPowerLevelRange(leftVelocityGoal);
        double right = this.applyPowerLevelRange(rightVelocityGoal);

        this.assertPowerLevelRange(left, "left");
        this.assertPowerLevelRange(right, "right");

        return new Setpoint(left, right);
    }

    /**
     * Calculate the power setting to use based on the inputs when in position mode
     * @return power settings for left and right motor
     */
    private Setpoint calculatePositionModeSetpoint()
    {
        // get the desired left and right values from the driver.
        double leftPositionGoal = this.driver.getAnalog(Operation.DriveTrainLeftPosition);
        double rightPositionGoal = this.driver.getAnalog(Operation.DriveTrainRightPosition);

        this.logger.logNumber(DriveTrainMechanism.LogName, "leftPositionGoal", leftPositionGoal);
        this.logger.logNumber(DriveTrainMechanism.LogName, "rightPositionGoal", rightPositionGoal);

        return new Setpoint(leftPositionGoal, rightPositionGoal);
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
     * Simple holder of setpoint information for the left and right sides
     */
    private class Setpoint
    {
        private double left;
        private double right;

        /**
         * Initializes a new Setpoint
         * @param left value to apply
         * @param right value to apply
         */
        public Setpoint(double left, double right)
        {
            this.left = left;
            this.right = right;
        }

        /**
         * gets the left setpoint
         * @return left setpoint value
         */
        public double getLeft()
        {
            return this.left;
        }

        /**
         * gets the right setpoint
         * @return right setpoint value
         */
        public double getRight()
        {
            return this.right;
        }
    }
}
